#pragma once

#if defined(MARS)
    #include "boilerplate/Sensors/Impl/ASM330.h"
    #include "boilerplate/Sensors/Impl/ICM20948.h"
    #include "boilerplate/Sensors/Impl/LPS22.h"
    #include "boilerplate/Sensors/Impl/MAX10S.h"

    #include "SdFat.h"

    #define SD_CS PA15
    #define SD_SCLK PB3
    #define SD_MISO PB4
    #define SD_MOSI PB5

    #define XBEE_CS PA4
    #define XBEE_SCLK PA5
    #define XBEE_MISO PA6
    #define XBEE_MOSI PA7

    #define SENSOR_SCL PB6
    #define SENSOR_SDA PB7

    #define LED_PIN PB9
#elif defined(POLARIS)
    #include "boilerplate/Sensors/Impl/MAX10S.h"
    #include "boilerplate/Sensors/Impl/Polaris/ICM42688.h"
    #include "boilerplate/Sensors/Impl/Polaris/MS5611.h"
    #include "boilerplate/Sensors/Impl/Polaris/MMC5983.h"

    #include "SD.h"

    #define SD_CS 31
    #define SD_SCLK 13
    #define SD_MISO 12
    #define SD_MOSI 11

    #define SENSOR_SCL 19
    #define SENSOR_SDA 18
    
    #define LED_PIN 6
#endif
