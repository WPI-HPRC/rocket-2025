#include "XBeeProSX.h"
#include <cstdarg>

XbeeProSX::XbeeProSX(uint8_t cs_pin, SPIClass* spi_dev)
    : XBeeDevice(SerialInterface::SPI), _cs_pin(cs_pin), spi_dev(spi_dev) {
    sendTransmitRequestsImmediately = true;
    sendFramesImmediately = true;
}

void XbeeProSX::start() {
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
}

void XbeeProSX::writeBytes_spi(char *data_io, size_t length_bytes) {
    digitalWrite(_cs_pin, LOW);

    spi_dev->transfer((void *)data_io, length_bytes);

    digitalWrite(_cs_pin, HIGH);
}

void XbeeProSX::_handleTransmitStatus(uint8_t frameID, uint8_t statusCode) {
    Serial.println("Received Transmit Status!?");
}

void XbeeProSX::handleReceivePacket(XBee::ReceivePacket::Struct *frame) {}

void XbeeProSX::handleReceivePacket64Bit(
    XBee::ReceivePacket64Bit::Struct *frame) {}

void XbeeProSX::incorrectChecksum(uint8_t calculated, uint8_t received) {}

void XbeeProSX::didCycle() {}

void XbeeProSX::log(const char *format, ...) {
    va_list args;
    va_start(args, format);

    va_end(args);
}
