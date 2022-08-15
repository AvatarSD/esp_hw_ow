/*
 * DallasSensorData.h
 *
 *  Created on: 28 вер. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_DALLASSENSORDATA_H_
#define DALLASONEWIRE_DALLASSENSORDATA_H_

#include "ow_rom.h"

const char *ow_temp_data_get_temp_str(float _temp);

struct ow_temp_data_t {
    struct rom_t _rom;
    double _temp;
};

#endif /* DALLASONEWIRE_DALLASSENSORDATA_H_ */
