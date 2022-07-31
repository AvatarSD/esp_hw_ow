/*
 * DallasTemp.cpp
 *
 *  Created on: 1 вер. 2015 р.
 *      Author: sd
 */

#include "ow_temp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char* TAG = "ow_temp";

// Model IDs
#define DS18S20MODEL 0x10
#define DS18B20MODEL 0x28
#define DS1822MODEL 0x22

// Scratchpad locations
#define TEMP_LSB 0
#define TEMP_MSB 1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP 3
#define CONFIGURATION 4
#define INTERNAL_BYTE 5
#define COUNT_REMAIN 6
#define COUNT_PER_C 7
#define SCRATCHPAD_CRC 8

// Device resolution
#define TEMP_9_BIT 0x1F   //  9 bit
#define TEMP_10_BIT 0x3F  // 10 bit
#define TEMP_11_BIT 0x5F  // 11 bit
#define TEMP_12_BIT 0x7F  // 12 bit

// reads scratchpad and returns the temperature in degrees C
static float temp_calculate_temp(const struct rom_t* deviceAddress, unsigned char* scratchPad) {
    int16_t rawTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 8) | scratchPad[TEMP_LSB];
    ESP_LOGV(TAG, "Calculating Temperature..");
    switch (deviceAddress->no[0]) {
        case DS18B20MODEL:
        case DS1822MODEL:
            if (deviceAddress->no[0] == DS18B20MODEL)
                ESP_LOGD(TAG, "DS18B20 MODEL");
            else
                ESP_LOGD(TAG, "DS1822 MODEL");
            switch (scratchPad[CONFIGURATION]) {
                case TEMP_12_BIT:
                    ESP_LOGD(TAG, "TEMP_12_BIT");
                    return (float)rawTemperature * 0.0625;
                    break;
                case TEMP_11_BIT:
                    ESP_LOGD(TAG, "TEMP_11_BIT");
                    return (float)(rawTemperature >> 1) * 0.125;
                    break;
                case TEMP_10_BIT:
                    ESP_LOGD(TAG, "TEMP_10_BIT");
                    return (float)(rawTemperature >> 2) * 0.25;
                    break;
                case TEMP_9_BIT:
                    ESP_LOGD(TAG, "TEMP_9_BIT");
                    return (float)(rawTemperature >> 3) * 0.5;
                    break;
            }
            break;
        case DS18S20MODEL:
            ESP_LOGD(TAG, "DS18S20 MODEL");
            /*
             Resolutions greater than 9 bits can be calculated using the data from
             the temperature, COUNT REMAIN and COUNT PER �C registers in the
             scratchpad. Note that the COUNT PER �C register is hard-wired to 16
             (10h). After reading the scratchpad, the TEMP_READ value is obtained
             by truncating the 0.5*C bit (bit 0) from the temperature data. The
             extended resolution temperature can then be calculated using the
             following equation:

             COUNT_PER_C - COUNT_REMAIN
             TEMPERATURE = TEMP_READ - 0.25 + --------------------------
             COUNT_PER_C
             */

            // Good spot. Thanks Nic Johns for your contribution
            return (float)(rawTemperature >> 1) - 0.25 +
                   ((float)(scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) /
                    (float)scratchPad[COUNT_PER_C]);
            break;
    }
    return -127;
}

bool ow_temp_reading_init(hw_ow_t* hw_ow) {
    if (!hw_ow) {
        ESP_LOGE(TAG, "%s:%u hw_ow is NILL", __FILE__, __LINE__);
        return false;
    }
    if (OWReset(hw_ow) == FALSE) {
        ESP_LOGW(TAG, "%s:%u OWReset fail!", __FILE__, __LINE__);
        return false;
    }
    return true;
}

bool ow_temp_read_once(hw_ow_t* hw_ow, struct ow_temp_data_t* data) {
    if (!hw_ow) {
        ESP_LOGE(TAG, "%s:%u hw_ow is NILL", __FILE__, __LINE__);
        return false;
    }
    if (!data) {
        ESP_LOGE(TAG, "%s:%u ow_temp_data_t is NILL", __FILE__, __LINE__);
        return false;
    }

    unsigned char sendpacket[10];
    int sendlen = 0;
    bool retVal = true;

    do {
        if (OWNext(hw_ow)) {
            // verify correct type
            if ((rom_is_match_family(&hw_ow->rom, 0x28)) ||
                (rom_is_match_family(&hw_ow->rom, 0x22)) ||
                (rom_is_match_family(&hw_ow->rom, 0x10))) {
                OWWriteBytePower(hw_ow, 0x44);  //)

                // sleep for 1 second
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                if (OWReadByte(hw_ow) != 0xFF)
                    ESP_LOGE(TAG, "Temperature conversion was not complete");

                // select the device
                sendpacket[0] = 0x55;  // match command
                for (int i = 0; i < 8; i++) sendpacket[i + 1] = hw_ow->rom.no[i];

                // Reset 1-Wire
                if (OWReset(hw_ow)) {
                    // MATCH ROM sequence
                    OWBlock(hw_ow, sendpacket, 9);

                    // Read Scratch pad
                    sendlen = 0;
                    sendpacket[sendlen++] = 0xBE;
                    for (int i = 0; i < 9; i++) sendpacket[sendlen++] = 0xFF;

                    if (OWBlock(hw_ow, sendpacket, sendlen)) {
                        double temp = temp_calculate_temp(&hw_ow->rom, sendpacket + 1);
                        data->_rom.raw = hw_ow->rom.raw;
                        data->_temp = temp;

                        ESP_LOGI(TAG, "%s temp: %3.1f`C", rom_to_string(&hw_ow->rom), temp);
                        return true;
                    }
                }
            } else {
                ESP_LOGW(TAG, "Found else devise: %s", rom_to_string(&hw_ow->rom));
            }
        } else
            retVal = false;
    } while (retVal);
    return false;
}

bool ow_temp_read_sensor(hw_ow_t* hw_ow, const struct rom_t* sensorRom, float* retTemp) {
    if (!hw_ow) {
        ESP_LOGE(TAG, "%s:%u hw_ow is NILL", __FILE__, __LINE__);
        return false;
    }
    if (!sensorRom) {
        ESP_LOGE(TAG, "%s:%u sensorRom is NILL", __FILE__, __LINE__);
        return false;
    }
    if (!retTemp) {
        ESP_LOGE(TAG, "%s:%u retTemp is NILL", __FILE__, __LINE__);
        return false;
    }

    unsigned char sendpacket[10];
    int sendlen = 0;

    ESP_LOGD(TAG, "Try to read: %s", rom_to_string(sensorRom));

    // verify correct type
    if ((rom_is_match_family(sensorRom, 0x28)) || (rom_is_match_family(sensorRom, 0x22)) ||
        (rom_is_match_family(sensorRom, 0x10))) {
        // match sequence for select the device
        sendpacket[0] = 0x55;  // match command
        for (int i = 0; i < 8; i++) sendpacket[i + 1] = sensorRom->no[i];

        // Reset 1-Wire
        if (OWReset(hw_ow))
            if (OWBlock(hw_ow, sendpacket, 9) == 0) {
                ESP_LOGW(TAG, "Device not available");
                return false;
            }

        // start conversion
        OWWriteBytePower(hw_ow, 0x44);

        // sleep for 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if (OWReadByte(hw_ow) != 0xFF) {
            ESP_LOGE(TAG, "Temperature conversion was not complete\n\r");
            return false;
        }

        // match sequence for select the device
        sendpacket[0] = 0x55;  // match command
        for (int i = 0; i < 8; i++) sendpacket[i + 1] = sensorRom->no[i];

        // Reset 1-Wire
        if (OWReset(hw_ow)) {
            // select device
            OWBlock(hw_ow, sendpacket, 9);

            // Read Scratch pad
            sendlen = 0;
            sendpacket[sendlen++] = 0xBE;
            for (int i = 0; i < 9; i++) sendpacket[sendlen++] = 0xFF;

            if (OWBlock(hw_ow, sendpacket, sendlen)) {
                *retTemp = temp_calculate_temp(sensorRom, sendpacket + 1);

                if (*retTemp == -127) {
                    ESP_LOGW(TAG, "Device reading error: answer 0xFF");
                    return false;
                }

                ESP_LOGD(TAG, "Temp is: %3.1f`C", *retTemp);

                return true;
            } else {
                ESP_LOGW(TAG, "Device not available to read");
                return false;
            }
        }
        ESP_LOGW(TAG, "Some error while reading sensor");
    } else
        ESP_LOGW(TAG, "This is not temp sensor");
    return false;
}

uint16_t ow_temp_search_all_temp_sensors(hw_ow_t* hw_ow, struct rom_t* rom_arr, uint16_t size) {
    if (!hw_ow) {
        ESP_LOGE(TAG, "%s:%u hw_ow is NILL", __FILE__, __LINE__);
        return false;
    }
    if (!rom_arr) {
        ESP_LOGE(TAG, "%s:%u rom_arr is NILL", __FILE__, __LINE__);
        return false;
    }

    for (int i = 0; i < size; i++) rom_zeroing(&rom_arr[i]);

    uint16_t newSensorsCount = 0, sensorsCount = 0;

    ESP_LOGI(TAG, "Searching sensors...");

    if (OWReset(hw_ow) == FALSE) {
        ESP_LOGW(TAG, "1-Wire bus reset error");
        return 0;
    }

    while (OWNext(hw_ow)) {
        // verify correct type
        if ((rom_is_match_family(&hw_ow->rom, 0x28)) || (rom_is_match_family(&hw_ow->rom, 0x22)) ||
            (rom_is_match_family(&hw_ow->rom, 0x10))) {
            ESP_LOGV(TAG, "Device: %s", rom_to_string(&hw_ow->rom));
            sensorsCount++;
            for (uint16_t i = 0; i < size; i++) {
                /* skip newSensorsCount if allready exist in array */
                if (rom_arr[i].raw == hw_ow->rom.raw) break;

                /* find last free place */
                if (rom_is_null(&rom_arr[i])) {
                    rom_arr[i].raw = hw_ow->rom.raw;
                    newSensorsCount++;

                    ESP_LOGI(TAG, "New sensor: %s", rom_to_string(&hw_ow->rom));
                    break;
                }
            }
        } else {
            ESP_LOGW(TAG, "Found else device: %s", rom_to_string(&hw_ow->rom));
        }
    }

    ESP_LOGI(TAG, "Sensor founded: %d, total new: %u", sensorsCount, newSensorsCount);

    return newSensorsCount;
}
