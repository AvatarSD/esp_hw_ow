/*
 * DallasTemp.cpp
 *
 *  Created on: 1 вер. 2015 р.
 *      Author: sd
 */

#include "DallasTemp.h"
#include "util/delay.h"

#include "../LOG/debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void * operator new(size_t size)
{
	return malloc(size);
}

void operator delete(void * ptr)
{
	free(ptr);
}

// Model IDs
#define DS18S20MODEL 0x10
#define DS18B20MODEL 0x28
#define DS1822MODEL  0x22

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

// ow_temp_DallasTemp(DallasOneWire& iface) :
// 		_iface(iface)
// {

// }
/*
const std::list<DallasSensorData>& DallasTemp::readAllTempSerial(bool isCurr)
{
	INFO(F("Reading sensors with serial conversion..."));
	if (isCurr)
		DEBUG(F("Reading with a pause for each sensor"));

	//TODO repeat reading if have any error

	if (_iface.OWReset() == FALSE)
	{
		WARNING(F("1-Wire bus reset error"));
		return _sensorsRes;
	}

	_sensorsRes.clear();

	unsigned char sendpacket[10];
	int sendlen = 0;

	while (_iface.OWNext())
	{
		// verify correct type
		if ((_iface.ROM_NO.isMathFamily(0x28))
				|| (_iface.ROM_NO.isMathFamily(0x22))
				|| (_iface.ROM_NO.isMathFamily(0x10)))
		{
			_iface.OWWriteBytePower(0x44); //)

			if (isCurr)
			{
				// sleep for 1 second
				_delay_ms(1000);
				if (_iface.OWReadByte() != 0xFF)
					WARNING(
							F("ERROR, temperature conversion was not complete\n\r"));
			}

			// select the device
			sendpacket[0] = 0x55; // match command
			for (int i = 0; i < 8; i++)
				sendpacket[i + 1] = _iface.ROM_NO[i];

			// Reset 1-Wire
			if (_iface.OWReset())
			{
				// MATCH ROM sequence
				_iface.OWBlock(sendpacket, 9);

				// Read Scratch pad
				sendlen = 0;
				sendpacket[sendlen++] = 0xBE;
				for (int i = 0; i < 9; i++)
					sendpacket[sendlen++] = 0xFF;

				if (_iface.OWBlock(sendpacket, sendlen))
				{
					double temp = ((double) ((int) (sendpacket[2] << 8)
							| (sendpacket[1]))) / 0xf;
					_sensorsRes.push_back(
							DallasSensorData(_iface.ROM_NO, temp));

#ifdef LEVEL_INFO
					char buff[40];
					strcpy(buff, _iface.ROM_NO.toString());
					sprintf(buff + 16, "  Temperature is: %3.1f`C", temp);
					INFO(buff);
#endif
				}
			}
		}
		else
		{
			WARNING(F("Found else devise"));
			WARNING(_iface.ROM_NO.toString());
		}
	}

#ifdef LEVEL_INFO
	char buf[20];
	sprintf(buf, "%d sensors readed", _sensorsRes.size());
	INFO(buf);
#endif

	return _sensorsRes;

}

*/
/*
 const std::list<DallasSensorData>&  DallasTemp::readAllTempParalel(char attemptNum)
 {
 //TODO do several attempt reading times

 * 3 попытки поиска(+намАттемп) + старт конверсии, если в какойто меньше датчиков чем в остальных - в лог
 * попытка считывания(+намАттемп) - если датчиков меньше - до 3 попыток(+намАттемп) повторного считывания
 * +в лог варнинги.
 * +критикал если датчиков ноль
 *

 INFO(F("Reading sensors with parallel conversion..."));
 while(_iface.OWReset() == FALSE);
 justStartConversion();
 return justGetTemp();

 }
 */
/*
 int DallasTemp::justStartConversion()
 {
 int numtemp=0;

 while (_iface.OWNext())
 {
 // verify correct type
 if ((_iface.ROM_NO.isMathFamily(0x28))||
 (_iface.ROM_NO.isMathFamily(0x22))||
 (_iface.ROM_NO.isMathFamily(0x10)))
 {
 _iface.OWWriteBytePower(0x44);
 numtemp++;
 }
 else
 {
 WARNING("Found else devise");
 WARNING(_iface.ROM_NO.toString());
 }
 }

 #ifdef LEVEL_INFO
 char buf[20];
 sprintf(buf, "%d sensors send packets to start conversions", _sensorsRes.size());
 INFO(buf);
 #endif

 return numtemp;
 }*/
/*
 const std::list<DallasSensorData>& DallasTemp::justGetTemp()
 {
 unsigned char sendpacket[10];
 int sendlen=0;

 while (_iface.OWNext())
 {
 // verify correct type
 if ((_iface.ROM_NO.isMathFamily(0x28))||
 (_iface.ROM_NO.isMathFamily(0x22))||
 (_iface.ROM_NO.isMathFamily(0x10)))
 {
 // verify complete
 //			if (_iface.OWReadByte() != 0xFF)
 //			{
 //				WARNING("ERROR, temperature conversion was not complete at:");
 //				WARNING(_iface.ROM_NO.toString());
 //			}

 // select the device
 sendpacket[0] = 0x55; // match command
 for (int i = 0; i < 8; i++)
 sendpacket[i+1] = _iface.ROM_NO[i];

 // Reset 1-Wire
 if (_iface.OWReset())
 {
 // MATCH ROM sequence
 _iface.OWBlock(sendpacket,9);



 // Read Scratch pad
 sendlen = 0;
 sendpacket[sendlen++] = 0xBE;
 for (int i = 0; i < 9; i++)
 sendpacket[sendlen++] = 0xFF;

 if (_iface.OWBlock(sendpacket,sendlen))
 {
 double temp = ((double)((int)(sendpacket[2]<<8)|(sendpacket[1])))/0xf;
 _sensorsRes.push_back(DallasSensorData(_iface.ROM_NO, temp));

 #ifdef LEVEL_INFO
 char buff[40];
 strcpy(buff, _iface.ROM_NO.toString());
 sprintf(buff+16, "  Temperature is: %3.1f`C", temp);
 INFO(buff);
 #endif
 }
 }
 }
 else
 {
 WARNING(F("Found else devise"));
 WARNING(_iface.ROM_NO.toString());
 }
 }

 #ifdef LEVEL_INFO
 char buf[20];
 sprintf(buf, "%d sensors readed", _sensorsRes.size());
 INFO(buf);
 #endif

 return _sensorsRes;
 }
 */

bool ow_temp_read_once(DallasSensorData & data)
{
	unsigned char sendpacket[10];
	int sendlen = 0;
	bool retVal = true;

	do
	{
		if (_iface.OWNext())
		{
			// verify correct type
			if ((_iface.ROM_NO.isMathFamily(0x28))
					|| (_iface.ROM_NO.isMathFamily(0x22))
					|| (_iface.ROM_NO.isMathFamily(0x10)))
			{
				_iface.OWWriteBytePower(0x44); //)

				// sleep for 1 second
				_delay_ms(1000);
				if (_iface.OWReadByte() != 0xFF)
					WARNING(
							F(
									"ERROR, temperature conversion was not complete\n\r"));

				// select the device
				sendpacket[0] = 0x55; // match command
				for (int i = 0; i < 8; i++)
					sendpacket[i + 1] = _iface.ROM_NO[i];

				// Reset 1-Wire
				if (_iface.OWReset())
				{
					// MATCH ROM sequence
					_iface.OWBlock(sendpacket, 9);

					// Read Scratch pad
					sendlen = 0;
					sendpacket[sendlen++] = 0xBE;
					for (int i = 0; i < 9; i++)
						sendpacket[sendlen++] = 0xFF;

					if (_iface.OWBlock(sendpacket, sendlen))
					{
						double temp = calculateTemperature(_iface.ROM_NO,
								sendpacket + 1);
						data(_iface.ROM_NO, temp);

#ifdef LEVEL_INFO
						char buff[40];
						strcpy(buff, _iface.ROM_NO.toString());
						sprintf(buff + 16, "  Temp: %3.1f`C", temp);
						INFO(buff);
#endif
						return true;
					}
				}
			}
			else
			{
				WARNING(F("Found else devise"));
				WARNING(_iface.ROM_NO.toString());
			}
		}
		else
			retVal = false;
	} while (retVal);
	return false;
}

void ow_temp_reading_init()
{
	while (_iface.OWReset() == FALSE)
		;

	//_sensorsRes.clear();
}

/*const std::list<ROM>& DallasTemp::searchAllTempSensors()
{
	INFO(F("Searching sensors..."));

	//TODO ...
	_sensors.clear();

	DATA(F("clear _sensors"));

	if (_iface.OWReset() == FALSE)
	{
		WARNING(F("1-Wire bus reset error"));
		return _sensors;
	}

	while (_iface.OWNext())
	{
		// verify correct type
		if ((_iface.ROM_NO.isMathFamily(0x28))
				|| (_iface.ROM_NO.isMathFamily(0x22))
				|| (_iface.ROM_NO.isMathFamily(0x10)))
		{
			DEBUG(_iface.ROM_NO.toString());
			_sensors.push_back(_iface.ROM_NO);
		}
		else
		{
			WARNING(F("Found else devise"));
			WARNING(_iface.ROM_NO.toString());
		}
	}

#ifdef LEVEL_INFO
	char buf[20];
	sprintf(buf, "%d sensors founded", _sensors.size());
	INFO(buf);
#endif

	return _sensors;

}*/

bool ow_temp_read_sensor(const ROM& sensorRom, double& retTemp)
{
	unsigned char sendpacket[10];
	int sendlen = 0;

#ifdef LEVEL_INFO
	char charbuf[40];
	sprintf(charbuf, "Try to read: %s", sensorRom.toString());
	INFO(charbuf);
#endif

	// verify correct type
	if ((sensorRom.isMathFamily(0x28)) || (sensorRom.isMathFamily(0x22))
			|| (sensorRom.isMathFamily(0x10)))
	{

		// match sequence for select the device
		sendpacket[0] = 0x55; // match command
		for (int i = 0; i < 8; i++)
			sendpacket[i + 1] = sensorRom[i];

		// Reset 1-Wire
		if (_iface.OWReset())
			if (_iface.OWBlock(sendpacket, 9) == 0)
			{
				WARNING(F("Device not available"));
				return false;
			}

		//start conversion
		_iface.OWWriteBytePower(0x44); //)

		// sleep for 1 second
		_delay_ms(1000);

		if (_iface.OWReadByte() != 0xFF)
		{
			WARNING(F("ERROR, temperature conversion was not complete\n\r"));
			return false;
		}

		// match sequence for select the device
		sendpacket[0] = 0x55; // match command
		for (int i = 0; i < 8; i++)
			sendpacket[i + 1] = sensorRom[i];

		// Reset 1-Wire
		if (_iface.OWReset())
		{
			// select device
			_iface.OWBlock(sendpacket, 9);

			// Read Scratch pad
			sendlen = 0;
			sendpacket[sendlen++] = 0xBE;
			for (int i = 0; i < 9; i++)
				sendpacket[sendlen++] = 0xFF;

			if (_iface.OWBlock(sendpacket, sendlen))
			{
				retTemp = calculateTemperature(sensorRom, sendpacket + 1);

				if (retTemp == -127)
				{
					WARNING(F("Device reading error: answer 0xFF"));
					return false;
				}

#ifdef LEVEL_INFO
				char buff[20];
				sprintf(buff, "Temp is: %3.1f`C", retTemp);
				INFO(buff);
#endif
				return true;
			}
			else
			{
				WARNING(F("Device not available to read"));
				return false;
			}
		}
		WARNING(F("Some error while reading sensor"));
	}
	else
		WARNING(F("This is not temp sensor"));
	return false;
}

// reads scratchpad and returns the temperature in degrees C
float ow_temp_calculate_temperature(const ROM & deviceAddress,
		unsigned char * scratchPad)
{
	int16_t rawTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 8)
			| scratchPad[TEMP_LSB];
	DATA(F("Calculating Temperature.."));
	switch (deviceAddress[0])
	{
	case DS18B20MODEL:
	case DS1822MODEL:
		if (deviceAddress[0] == DS18B20MODEL)
			DEBUG(F("DS18B20 MODEL"));
		else
			DEBUG(F("DS1822 MODEL"));
		switch (scratchPad[CONFIGURATION])
		{
		case TEMP_12_BIT:
			DEBUG(F("TEMP_12_BIT"));
			return (float) rawTemperature * 0.0625;
			break;
		case TEMP_11_BIT:
			DEBUG(F("TEMP_11_BIT"));
			return (float) (rawTemperature >> 1) * 0.125;
			break;
		case TEMP_10_BIT:
			DEBUG(F("TEMP_10_BIT"));
			return (float) (rawTemperature >> 2) * 0.25;
			break;
		case TEMP_9_BIT:
			DEBUG(F("TEMP_9_BIT"));
			return (float) (rawTemperature >> 3) * 0.5;
			break;
		}
		break;
	case DS18S20MODEL:
		DEBUG(F("DS18S20 MODEL"));
		/*
		 Resolutions greater than 9 bits can be calculated using the data from
		 the temperature, COUNT REMAIN and COUNT PER �C registers in the
		 scratchpad. Note that the COUNT PER �C register is hard-wired to 16
		 (10h). After reading the scratchpad, the TEMP_READ value is obtained
		 by truncating the 0.5�C bit (bit 0) from the temperature data. The
		 extended resolution temperature can then be calculated using the
		 following equation:

		 COUNT_PER_C - COUNT_REMAIN
		 TEMPERATURE = TEMP_READ - 0.25 + --------------------------
		 COUNT_PER_C
		 */

		// Good spot. Thanks Nic Johns for your contribution
		return (float) (rawTemperature >> 1) - 0.25
				+ ((float) (scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN])
						/ (float) scratchPad[COUNT_PER_C]);
		break;
	}
	return -127;
}

uint16_t ow_temp_search_all_temp_sensors(ROM* _mainbuf, uint16_t size)
{
	uint16_t newSensorsCount = 0, sensorsCount = 0;

	INFO(F("Searching sensors..."));

	if (_iface.OWReset() == FALSE)
	{
		WARNING(F("1-Wire bus reset error"));
		return 0;
	}

	while (_iface.OWNext())
	{
		// verify correct type
		if ((_iface.ROM_NO.isMathFamily(0x28))
				|| (_iface.ROM_NO.isMathFamily(0x22))
				|| (_iface.ROM_NO.isMathFamily(0x10)))
		{
			DEBUG(_iface.ROM_NO.toString());
			sensorsCount++;
			for (uint16_t i = 0; i < size; i++)
			{
				if (_mainbuf[i] == _iface.ROM_NO)
					break;

				if (_mainbuf[i].isNull())
				{
					_mainbuf[i] = _iface.ROM_NO;
					newSensorsCount++;
#ifdef LEVEL_INFO
					char buff[30];
					sprintf(buff, "New sensor: %s", _iface.ROM_NO.toString());
					INFO(buff);
#endif
					break;
				}

			}
		}
		else
		{
			WARNING(F("Found else devise"));
			WARNING(_iface.ROM_NO.toString());
		}
	}

#ifdef LEVEL_INFO
	char buf[20];
	sprintf(buf, "Sensor founded: %d", sensorsCount);
	INFO(buf);
#endif

	return newSensorsCount;
}
