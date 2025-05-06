#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "CStringBuilder.h"
#include "Context.h"
#include "Packet.pb.h"
#include "RocketTelemetryPacket.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "xbee/XBeeDevice.h"

class XbeeProSX : public XBeeDevice {
  public:
    XbeeProSX(Context *ctx, uint8_t cs_pin, uint8_t attn_pin, long long gs_addr,
              SPIClass *spi_dev, size_t send_delay = 25);

    void writeBytes_spi(char *data_io, size_t length_bytes) override;

    void readBytes_spi(uint8_t *buffer, size_t length_bytes) override;

    bool canReadSPI() override;

    void handleReceivePacket(XBee::ReceivePacket::Struct *frame) override;

    void
    handleReceivePacket64Bit(XBee::ReceivePacket64Bit::Struct *frame) override;

    void didCycle() override;

    void start() override;

    void loop();

    void incorrectChecksum(uint8_t calculated, uint8_t received) override;

    void log(const char *format, ...) override;

  private:
    Context *ctx;
    uint8_t _cs_pin;
    uint8_t _attn_pin;
    long long gs_addr;
    SPIClass *spi_dev;
    size_t send_delay;
    size_t last_sent;

    uint8_t tx_buf[4096] = {};
    pb_ostream_t ostream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
    pb_istream_t istream;

    HPRC_Packet final_packet;
    HPRC_Telemetry final_telem_packet;
    HPRC_RocketTelemetryPacket *telem_packet;

    HPRC_Packet rx_packet;
    HPRC_Command *rx_command;
    HPRC_CommandResponse tx_command_response;
    bool command_response_to_send = false;

    FsFile sd_root;

    uint64_t subscribers[64];
    size_t num_subscribers;

    void add_subscriber(uint64_t address);
};
