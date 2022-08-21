/*
 * DallasTemp.h
 *
 *  Created on: 1 вер. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_DALLASTEMP_H_
#define DALLASONEWIRE_DALLASTEMP_H_

#include <stdbool.h>

#include "hw_ow.h"
#include "ow_rom.h"
#include "ow_temp_data.h"

/* First algoritm*/
bool ow_temp_reading_init(hw_ow_t* hw_ow);
bool ow_temp_read_once(hw_ow_t* hw_ow, struct ow_temp_data_t* data);

/* Second algoritm*/
bool ow_temp_read_sensor(hw_ow_t* hw_ow, const rom_t* sensorRom, float* retTemp);
uint16_t ow_temp_search_all_temp_sensors(hw_ow_t* hw_ow, rom_t* buff, uint16_t size);

#endif /* DALLASONEWIRE_DALLASTEMP_H_ */
