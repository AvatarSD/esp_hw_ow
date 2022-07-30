/*
 * DallasSensorData.h
 *
 *  Created on: 28 вер. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_DALLASSENSORDATA_H_
#define DALLASONEWIRE_DALLASSENSORDATA_H_

#include "ow_rom.h"

// class DallasSensorData
// {
// public:
// 	DallasSensorData();
// 	DallasSensorData(ROM rom, float temp);
// 	void* operator()(const ROM &rom, const float &temp);
// 	const ROM& getROM();
// 	const double& getTemp();
// 	const char * getTempStr();
// private:
struct ow_temp_data_t {
	rom_t _rom;
	double _temp;
};

#endif /* DALLASONEWIRE_DALLASSENSORDATA_H_ */
