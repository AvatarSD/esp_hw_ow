/*
 * DallasTemp.h
 *
 *  Created on: 1 вер. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_DALLASTEMP_H_
#define DALLASONEWIRE_DALLASTEMP_H_


//#include <list>
#include "hw_ow.h"
#include "ow_temp_data.h"


// class DallasTemp {
// public:
	// DallasTemp(DallasOneWire& iface);

	/*Second algoritm*/
	/*const std::list<DallasSensorData>&  readAllTempSerial(bool isCurr = true);*/
	//const std::list<DallasSensorData>&  readAllTempParalel(char attemptNum);

	/*Second algoritm*/
	void ow_temp_reading_init();
	bool ow_temp_read_once(ow_temp_data_t* data);

	/*Third algoritm*/
	/*const std::list<ROM>&  searchAllTempSensors();*/
	bool ow_temp_read_sensor(const rom_t* sensorRom, double * retTemp);

	/****************/
	uint16_t ow_temp_search_all_temp_sensors(rom_t * buff, uint16_t size);


// private:
	//int justStartConversion();
	//const std::list<DallasSensorData>& justGetTemp();
	float calculateTemperature(const rom_t deviceAddress, unsigned char * scratchPad);
//
//	std::list<DallasSensorData> _sensorsRes;
//	std::list<ROM> _sensors;
	// hw_ow_t* hw_ow& _iface;
// };

#endif /* DALLASONEWIRE_DALLASTEMP_H_ */
