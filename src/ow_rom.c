/*
 * ROM.cpp
 *
 *  Created on: 31 серп. 2015 р.
 *      Author: sd
 */

#include "ROM.h"
#include "stdio.h"

ROM::ROM()
{
	zeroing();
}

unsigned char& ROM::operator [](int i)
{
	if ((i >= 0) && (i < 8))
		return ROM_NO[i];
	else
	{
		static unsigned char null;
		null = 0;
		return null;
	}
}

const unsigned char& ROM::operator [](int i) const
{
	if ((i >= 0) && (i < 8))
		return ROM_NO[i];
	else
	{
		static unsigned char null;
		null = 0;
		return null;
	}
}

bool ROM::isMathFamily(unsigned char family) const
{
	return (family == ROM_NO[0]);
}

bool ROM::operator <(const ROM& cmp) const
{
	return (operator &() < &cmp);
}

const unsigned long long int& ROM::operator &() const
{
	return *((unsigned long long int*) ROM_NO);
}

const char * ROM::toString() const
{
	static char buf[17];
	for (auto i = 0; i < 8; i++)
		sprintf(&buf[i * 2], "%02X", ROM_NO[i]);
	return buf;
}

bool ROM::operator ==(const ROM& cmp) const
{
	for (uint8_t i = 0; i < 8; i++)
		if (ROM_NO[i] != cmp[i])
			return false;
	return true;
}

bool ROM::isNull() const
{
	for (uint8_t i = 0; i < 8; i++)
		if (ROM_NO[i] != 0)
			return false;
	return true;
}

void ROM::zeroing()
{
	for (auto i = 0; i < 8; i++)
		ROM_NO[i] = 0x00;
}

bool ROM::operator =(const char * str)
{

		sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &ROM_NO[0], &ROM_NO[1],
				&ROM_NO[2], &ROM_NO[3], &ROM_NO[4], &ROM_NO[5], &ROM_NO[6],
				&ROM_NO[7]);

//	sscanf(str, "%llx", (unsigned long long *)ROM_NO);

	return true;
}
