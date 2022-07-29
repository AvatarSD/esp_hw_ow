/*
 * DallasSensorData.cpp
 *
 *  Created on: 28 вер. 2015 р.
 *      Author: sd
 */

#include "DallasSensorData.h"
#include <stdio.h>

DallasSensorData::DallasSensorData() :
_rom(), _temp(0)
{

}

DallasSensorData::DallasSensorData(ROM rom, float temp) :
_rom(rom), _temp(temp)
{

}

void* DallasSensorData::operator()(const ROM &rom, const float &temp)
{
	_rom = rom;
	_temp = temp;
	return 0;
}

const ROM& DallasSensorData::getROM()
{
	return _rom;
}

const double& DallasSensorData::getTemp()
{
	return _temp;
}

const char * DallasSensorData::getTempStr()
{
	static char buff[8];
	sprintf(buff, "%3.1f", _temp);
	return buff;
}
