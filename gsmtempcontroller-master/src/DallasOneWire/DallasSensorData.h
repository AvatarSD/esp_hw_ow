/*
 * DallasSensorData.h
 *
 *  Created on: 28 вер. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_DALLASSENSORDATA_H_
#define DALLASONEWIRE_DALLASSENSORDATA_H_

#include "DallasOneWire.h"

class DallasSensorData
{
public:
	DallasSensorData();
	DallasSensorData(ROM rom, float temp);
	void* operator()(const ROM &rom, const float &temp);
	const ROM& getROM();
	const double& getTemp();
	const char * getTempStr();
private:
	ROM _rom;
	double _temp;
};

#endif /* DALLASONEWIRE_DALLASSENSORDATA_H_ */
