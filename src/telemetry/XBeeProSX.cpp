#include "XBeeProSX.h"
#include "CStringBuilder.h"
#include "Command.pb.h"
#include "CommandResponse.pb.h"
#include "Packet.pb.h"
#include "Telemetry.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <cstdarg>

XbeeProSX::XbeeProSX(Context *ctx, uint8_t cs_pin, uint8_t attn_pin,
                     long long gs_addr, SPIClass *spi_dev, size_t send_delay)
    : XBeeDevice(SerialInterface::SPI), ctx(ctx), _cs_pin(cs_pin),
      _attn_pin(attn_pin), gs_addr(gs_addr), spi_dev(spi_dev),
      send_delay(send_delay),
      telem_packet(&final_telem_packet.Message.rocketPacket),
      rx_command(&rx_packet.Message.command),
      sb(CStringBuilder((char *)ls_buf, sizeof(tx_buf))) {
    sendTransmitRequestsImmediately = true;
    sendFramesImmediately = true;

    final_telem_packet.which_Message = HPRC_Telemetry_rocketPacket_tag;
}

void XbeeProSX::start() {
    pinMode(_cs_pin, OUTPUT);
    pinMode(_attn_pin, INPUT);
    digitalWrite(_cs_pin, HIGH);

    last_sent = millis();
}

void XbeeProSX::loop() {
    size_t now = millis();
    if (now - last_sent > send_delay) {
        last_sent = now;
        // Write packet
        telem_packet->timestamp = now;
        telem_packet->altitude = ctx->baro.getData().altitude;
        telem_packet->servoPosition = ctx->airbrakes.read();

        // Send packet
        final_packet.which_Message = HPRC_Packet_telemetry_tag;
        final_packet.Message.telemetry = final_telem_packet;
        ostream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
        pb_encode(&ostream, &HPRC_Packet_msg, &final_packet);
        spi_dev->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        sendTransmitRequestCommand(gs_addr, tx_buf, ostream.bytes_written);
        spi_dev->endTransaction();
    }
    // For now, we won't even attempt to receive when in flight mode (although
    // we will still read bytes when writing)
    if (!ctx->flightMode) {
        spi_dev->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        receive();
        spi_dev->endTransaction();
    }
}

void XbeeProSX::handleReceivePacket(XBee::ReceivePacket::Struct *frame) {
    // We need this here since we will still receive bytes while writing
    if (ctx->flightMode) {
        return;
    }

    for (int i = 0; i < frame->dataLength_bytes; i++) {
        Serial.printf("0x%x ", frame->data[i]);
    }
    Serial.println();

    istream = pb_istream_from_buffer(frame->data, frame->dataLength_bytes);
    if (pb_decode(&istream, &HPRC_Packet_msg, &rx_packet)) {
        bool response_to_send = false;
        switch (rx_command->which_Message) {
        case HPRC_Command_setFlightMode_tag:
            ctx->flightMode = rx_command->Message.setFlightMode.flightModeOn;

            tx_command_response.which_Message =
                HPRC_CommandResponse_setFlightMode_tag;
            tx_command_response.Message.setFlightMode.success = true;
            response_to_send = true;
            break;
        case HPRC_Command_actuateAirbrakes_tag:
            Serial.printf("Servo command value: %u\n",
                          rx_command->Message.actuateAirbrakes.servoValue);
            ctx->airbrakes.write(
                rx_command->Message.actuateAirbrakes.servoValue);
            break;
        case HPRC_Command_readSDDirectory_tag:
            Serial.println("Reading SD Directory");
#if defined(MARS)
        {
            sb.reset();
            ctx->sd.ls(&sb, LS_SIZE);
            tx_command_response.which_Message =
                HPRC_CommandResponse_readSDDirectory_tag;
            tx_command_response.Message.readSDDirectory.filename.arg = ls_buf;
            tx_command_response.Message.readSDDirectory.filename.funcs.encode =
                [](pb_ostream_t *s, const pb_field_t *f,
                   void *const *arg) -> bool {
                char *buf = (char *)*arg;
                char *real_end = strchr(buf, '\0');
                char *end;
                while (buf < real_end) {
                    end = strchr(buf, '\n');
                    if (!pb_encode_tag_for_field(s, f)) {
                        return false;
                    }
                    if (!pb_encode_string(s, (const pb_byte_t *)buf,
                                          end - buf)) {
                        return false;
                    }
                    buf = end + 1;
                }
                return true;
            };
            response_to_send = true;
        }
#elif defined(POLARIS)
        {
            File root = SD.open("/");

            int i = 0;
            while (i < 200) {
               File file = root.openNextFile();

               if (!entry) {break;}

               file.name(); // maybe use file.getName()?

               i++;
            }
        }
#endif
        break;
        case HPRC_Command_readSDFile_tag:
            break;
        case HPRC_Command_clearSD_tag: {
            ctx->logFile.close();
            bool success = ctx->sd.format();
            success &= ctx->sd.begin(SD_CS, SD_SPI_SPEED);

            ctx->logFile =
                ctx->sd.open("flightData0.bin", O_RDWR | O_CREAT | O_TRUNC);

            tx_command_response.which_Message =
                HPRC_CommandResponse_clearSD_tag;
            tx_command_response.Message.clearSD.success = success;
            response_to_send = true;
        } break;
        case HPRC_Command_setVideoActive_tag:
            tx_command_response.which_Message =
                HPRC_CommandResponse_setVideoActive_tag;
            tx_command_response.Message.setVideoActive.success = false;
            response_to_send = true;
            break;
        default:
            break;
        }

        if (response_to_send) {
            final_packet.which_Message = HPRC_Packet_commandResponse_tag;
            final_packet.Message.commandResponse = tx_command_response;
            ostream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
            pb_encode(&ostream, &HPRC_Packet_msg, &final_packet);
            Serial.println();
            spi_dev->beginTransaction(
                SPISettings(1000000, MSBFIRST, SPI_MODE0));
            sendTransmitRequestCommand(gs_addr, tx_buf, ostream.bytes_written);
            spi_dev->endTransaction();
        }
    }
}

void XbeeProSX::writeBytes_spi(char *data_io, size_t length_bytes) {
    digitalWrite(_cs_pin, LOW);

    spi_dev->transfer(data_io, data_io, length_bytes);

    digitalWrite(_cs_pin, HIGH);
}

void XbeeProSX::readBytes_spi(uint8_t *buffer, size_t length_bytes) {
    digitalWrite(_cs_pin, LOW);

    spi_dev->transfer(nullptr, buffer, length_bytes);

    digitalWrite(_cs_pin, HIGH);
}

bool XbeeProSX::canReadSPI() {
    return !ctx->flightMode && digitalRead(_attn_pin) == LOW;
}

void XbeeProSX::handleReceivePacket64Bit(
    XBee::ReceivePacket64Bit::Struct *frame) {}

void XbeeProSX::incorrectChecksum(uint8_t calculated, uint8_t received) {}

void XbeeProSX::didCycle() {}

void XbeeProSX::log(const char *format, ...) {
    va_list args;
    va_start(args, format);

    va_end(args);
}
