/*
 * DallasTemp.h
 *
 *  Created on: 1 вер. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_DALLASTEMP_H_
#define DALLASONEWIRE_DALLASTEMP_H_


//#include <list>
#include "DallasSensorData.h"


class DallasTemp {
public:
	DallasTemp(DallasOneWire& iface);

	/*Second algoritm*/
	/*const std::list<DallasSensorData>&  readAllTempSerial(bool isCurr = true);*/
	//const std::list<DallasSensorData>&  readAllTempParalel(char attemptNum);

	/*Second algoritm*/
	void readingInit();
	bool readOnce(DallasSensorData & data);

	/*Third algoritm*/
	/*const std::list<ROM>&  searchAllTempSensors();*/
	bool readSensor(const ROM & sensorRom, double & retTemp);

	/****************/
	uint16_t searchAllTempSensors(ROM * buff, uint16_t size);


private:
	//int justStartConversion();
	//const std::list<DallasSensorData>& justGetTemp();
	float calculateTemperature(const ROM & deviceAddress, unsigned char * scratchPad);
//
//	std::list<DallasSensorData> _sensorsRes;
//	std::list<ROM> _sensors;
	DallasOneWire& _iface;
};

#endif /* DALLASONEWIRE_DALLASTEMP_H_ */
