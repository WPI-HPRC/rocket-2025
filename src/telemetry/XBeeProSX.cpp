#include "Command.pb.h"
#include "CommandResponse.pb.h"
#include "Packet.pb.h"
#include "Telemetry.pb.h"
#include "XBeeProSX.h"
#include "boilerplate/StateEstimator/AttEkf.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "stm32h753xx.h"
#include "stm32h7xx_hal_cortex.h"
#include <cstdarg>

XbeeProSX::XbeeProSX(Context *ctx, uint8_t cs_pin, uint8_t attn_pin,
                     long long gs_addr, SPIClass *spi_dev)
    : XBeeDevice(SerialInterface::SPI), ctx(ctx), _cs_pin(cs_pin),
      _attn_pin(attn_pin), gs_addr(gs_addr), spi_dev(spi_dev),
      telem_packet(&final_telem_packet.Message.rocketPacket),
      rx_command(&rx_packet.Message.command) {
    sendTransmitRequestsImmediately = true;
    sendFramesImmediately = true;

    final_telem_packet.which_Message = HPRC_Telemetry_rocketPacket_tag;
    telem_packet->loopCount = 0;
}

void XbeeProSX::start() {
    pinMode(_cs_pin, OUTPUT);
    pinMode(_attn_pin, INPUT);
    digitalWrite(_cs_pin, HIGH);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    last_sent = millis();
}

void XbeeProSX::loop() {
    size_t now = millis();
    if (now - last_sent >= ctx->xbeeLoggingDelay) {
        last_sent = now;
        // Write packet
        telem_packet->timestamp = now;
        telem_packet->loopCount++;

        telem_packet->pressure = ctx->baro.getData()->pressure;
        telem_packet->temperature = ctx->baro.getData()->temperature;
        telem_packet->altitude = ctx->baro.getData()->altitude;

        telem_packet->accelX = ctx->mag.getData()->accelX;
        telem_packet->accelY = ctx->mag.getData()->accelY;
        telem_packet->accelZ = ctx->mag.getData()->accelZ;

        telem_packet->gyroX = ctx->mag.getData()->gyrX;
        telem_packet->gyroY = ctx->mag.getData()->gyrY;
        telem_packet->gyroZ = ctx->mag.getData()->gyrZ;

        telem_packet->magX = ctx->mag.getData()->magX;
        telem_packet->magY = ctx->mag.getData()->magY;
        telem_packet->magZ = ctx->mag.getData()->magZ;

        telem_packet->servoPosition = ctx->airbrakes.read();

        telem_packet->gpsLat = ctx->gps.getData()->lat;
        telem_packet->gpsLong = ctx->gps.getData()->lon;
        telem_packet->satellites = ctx->gps.getData()->satellites;
        telem_packet->gpsLock = ctx->gps.getData()->gpsLockType == 3;
        telem_packet->gpsAltMSL = ctx->gps.getData()->altMSL;

        telem_packet->w = ctx->attEkfLogger.getState()(AttKFInds::q_w);
        telem_packet->i = ctx->attEkfLogger.getState()(AttKFInds::q_x);
        telem_packet->j = ctx->attEkfLogger.getState()(AttKFInds::q_y);
        telem_packet->k = ctx->attEkfLogger.getState()(AttKFInds::q_z);

        telem_packet->posX = ctx->pvKFLogger.getState()(0);
        telem_packet->posY = ctx->pvKFLogger.getState()(1);
        telem_packet->posZ = ctx->pvKFLogger.getState()(2);
        telem_packet->velX = ctx->pvKFLogger.getState()(3);
        telem_packet->velY = ctx->pvKFLogger.getState()(4);
        telem_packet->velZ = ctx->pvKFLogger.getState()(5);

        // Send packet

        final_packet.which_Message = HPRC_Packet_telemetry_tag;
        final_packet.Message.telemetry = final_telem_packet;
        ostream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
        pb_encode(&ostream, &HPRC_Packet_msg, &final_packet);
        spi_dev->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        sendTransmitRequestCommand(gs_addr, enable_acks, 0x83, 0x00, tx_buf,
                                   ostream.bytes_written);
        spi_dev->endTransaction();

        spi_dev->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        receive();
        spi_dev->endTransaction();
    }
}

void XbeeProSX::handleReceivePacket(XBee::ReceivePacket::Struct *frame) {
    istream = pb_istream_from_buffer(frame->data, frame->dataLength_bytes);
    if (pb_decode(&istream, &HPRC_Packet_msg, &rx_packet)) {
        bool response_to_send = false;
        if (ctx->flightMode) {
            if (rx_command->which_Message == HPRC_Command_setFlightMode_tag &&
                !rx_command->Message.setFlightMode.flightModeOn) {
                reenable_flightmode_counter += 1;
                last_reenable_flightmode = millis();
            }
            if (reenable_flightmode_counter >= 3) {
                ctx->flightMode = false;
                reenable_flightmode_counter = 0;

                tx_command_response.which_Message =
                    HPRC_CommandResponse_setFlightMode_tag;
                tx_command_response.Message.setFlightMode.success = true;
                response_to_send = true;

                goto send_response;
            }
            if (millis() - last_reenable_flightmode >= 5000) {
                reenable_flightmode_counter = 0;

                tx_command_response.which_Message =
                    HPRC_CommandResponse_setFlightMode_tag;
                tx_command_response.Message.setFlightMode.success = false;
                response_to_send = true;

                goto send_response;
            }
            return;
        }

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
        case HPRC_Command_powerCycle_tag: {
            HAL_NVIC_SystemReset();
            break;
        }
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
                [](pb_ostream_t *s, const pb_field_t *f,
                   void *const *arg) -> bool {
                FsFile *root = (FsFile *)*arg;
                root->rewindDirectory();
                FsFile file;
                static char name_buffer[256];

                while ((file = root->openNextFile())) {
                    if (!pb_encode_tag_for_field(s, f)) {
                        return false;
                    }

                    size_t name_len = file.getName(name_buffer, 256);
                    if (!pb_encode_string(s, (const pb_byte_t *)name_buffer,
                                          name_len)) {
                        return false;
                    }

                    file.close();
                }

                return true;
            };
            response_to_send = true;
        } break;
        case HPRC_Command_readSDFile_tag:
            break;
        case HPRC_Command_clearSD_tag: {
            ctx->logFile.close();
#if defined(MARS)
            bool success = ctx->sd.format();
            success &= ctx->sd.begin(SD_CS, SD_SPI_SPEED);

            ctx->logFile =
                ctx->sd.open("flightData0.csv", O_RDWR | O_CREAT | O_TRUNC);
            ctx->errorLogFile =
                ctx->sd.open("errorLog0.txt", O_RDWR | O_CREAT | O_TRUNC);
#elif defined(POLARIS)
            bool success = SD.format();
            success &= SD.begin(SD_CS);

            ctx->logFile = SD.open("flightData0.csv", FILE_WRITE_BEGIN);
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
            tx_command_response.Message.setVideoActive.success = true;
            response_to_send = true;

            Serial.printf("Writing relay pin to %d",
                          rx_command->Message.setVideoActive.videoActive ? 1
                                                                         : 0);

            digitalWrite(RELAY_PIN,
                         rx_command->Message.setVideoActive.videoActive ? HIGH
                                                                        : LOW);
        case HPRC_Command_setAcksEnabled_tag:
                setAcks(rx_command->Message.setAcksEnabled.acksEnabled);
            break;
        
        default:
            break;
        }

    send_response:
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
    return digitalRead(_attn_pin) == LOW;
}

void XbeeProSX::setAcks(bool acks_enabled) { enable_acks = acks_enabled; }

void XbeeProSX::handleReceivePacket64Bit(
    XBee::ReceivePacket64Bit::Struct *frame) {}

void XbeeProSX::incorrectChecksum(uint8_t calculated, uint8_t received) {}

void XbeeProSX::didCycle() {}

void XbeeProSX::log(const char *format, ...) {
    va_list args;
    va_start(args, format);

    va_end(args);
}
