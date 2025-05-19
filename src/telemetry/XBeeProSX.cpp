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
      rx_command(&rx_packet.Message.command) {
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

    // Reading File
    // if first char is not null term. then we have a file to send
    if (fileToSend[0] != 0) { 
        // open file (inspired from implimentation of reading all files in sd)
        #if defined(MARS)
        FsFile file = ctx->sd.open(fileToSend, O_READ);
        #elif defined(POLARIS)
        FsFile file = SD.sdfs.open(fileToSend, O_READ);
        #endif

        // check that file was opened and set the place to read from to were we got to in the last iteration
        if (file || file.seekSet(numBytesSent)) {
            byte data[maxBytesToSend] = {0}; // don't know if this needs to be null terminated or not
            size_t bytesRead = 0; // bytes read in this iteration, not same as `numBytesSent`

            // don't know if this is different for MARS and POLARIS
            bytesRead = file.read(data, maxBytesToSend);

            // this means no more file left to read in next iteration
            if (bytesRead < maxBytesToSend) {
                fileToSend[256] = {0};
                numBytesSent = 0;
            }

            file.close();

            // SOMEHOW ENCODE AND SEND DATA

            numBytesSent += bytesRead;
        }
        else { // give up
            fileToSend[256] = {0};
            numBytesSent = 0;
        }
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
        case HPRC_Command_readSDDirectory_tag: {
            Serial.println("Reading SD Directory");
            tx_command_response.which_Message =
                HPRC_CommandResponse_readSDDirectory_tag;
            #if defined(MARS)
            sd_root = ctx->sd.open("/");
            #elif defined(POLARIS)
            sd_root = SD.sdfs.open("/");
            #endif
            tx_command_response.Message.readSDDirectory.filename.arg = &sd_root;
            tx_command_response.Message.readSDDirectory.filename.funcs.encode =
                [](pb_ostream_t *s, const pb_field_t *f, void *const *arg) -> bool {
                FsFile *root = (FsFile *)*arg;
                root->rewindDirectory();
                FsFile file;
                static char name_buffer[256];
                
                while ((file = root->openNextFile())) {
                    if (!pb_encode_tag_for_field(s, f)) {
                        return false;
                    }
                    
                    size_t name_len = file.getName(name_buffer, 256);
                    if (!pb_encode_string(s, (const pb_byte_t *)name_buffer, name_len)) {
                        return false;
                    }

                    file.close();
                }

                return true;
            };
            response_to_send = true;
        } break;
        
        case HPRC_Command_readSDFile_tag:
            rx_command->Message.readSDFile.filename.arg = fileToSend; // pass the file-name buffer as argument
            rx_command->Message.readSDFile.filename.funcs.decode = 
                [](pb_istream_t *stream, const pb_field_t *field, void **arg) -> bool {
                    size_t len = stream->bytes_left;
                    if (len >= sizeof(arg)) {len = sizeof(arg) - 1;} // make sure we dont read more than 256 - 1 = 255 bytes

                    if (!pb_read(stream, (pb_byte_t*)arg, len-1)) { // read 1 less byte then in buffer (`arg`) leaving 1 for null term.
                        return false;
                    }

                    arg[len] = '\0'; // write null term.

                    // NOTE: since fileToSend is initialized to all 0s (so filled with null terminators),
                    // we might not have to worry about writing the null term. but this might be safer
                };

            Serial.printf("Reading SD File: %s", fileToSend);
            // now that fileToSend is set, we will send file in the next iteration of `loop()`
            break;
        case HPRC_Command_clearSD_tag: {
            ctx->logFile.close();
#if defined(MARS)
            bool success = ctx->sd.format();
            success &= ctx->sd.begin(SD_CS, SD_SPI_SPEED);

            ctx->logFile =
                ctx->sd.open("flightData0.csv", O_RDWR | O_CREAT | O_TRUNC);
#elif defined(POLARIS)
            bool success = SD.format();
            success &= SD.begin(SD_CS);

            ctx->logFile =
                SD.open("flightData0.csv", FILE_WRITE_BEGIN);
#endif

            ctx->logCsvHeader();

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

        sd_root.close();
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
