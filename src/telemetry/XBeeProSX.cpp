#include "XBeeProSX.h"
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
      telem_packet(&final_telem_packet.Message.rocketPacket) {
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

    Serial.println("recvd");

    istream = pb_istream_from_buffer(frame->data, frame->dataLength_bytes);
    if (pb_decode(&istream, &HPRC_Command_msg, &rx_command_packet)) {
        bool response_to_send = false;
        switch (rx_command_packet.which_Message) {
        case HPRC_Command_setFlightMode_tag:
            ctx->flightMode =
                rx_command_packet.Message.setFlightMode.flightModeOn;

            tx_command_response.which_Message =
                HPRC_CommandResponse_setFlightMode_tag;
            tx_command_response.Message.setFlightMode.success = true;
            response_to_send = true;
            break;
        case HPRC_Command_actuateAirbrakes_tag:
            Serial.printf(
                "Servo command value: %ud\n",
                rx_command_packet.Message.actuateAirbrakes.servoValue);
            ctx->airbrakes.write(
                rx_command_packet.Message.actuateAirbrakes.servoValue);
            break;
        case HPRC_Command_readSDDirectory_tag:
            break;
        case HPRC_Command_readSDFile_tag:
            break;
        case HPRC_Command_clearSD_tag:
            break;
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
