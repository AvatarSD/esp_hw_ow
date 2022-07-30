/*
 * ROM.h
 *
 *  Created on: 31 серп. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_ROM_H_
#define DALLASONEWIRE_ROM_H_

// class ROM {
// public:
// 	ROM();

// 	bool isMathFamily(unsigned char family) const;
// 	unsigned char& operator [](int i);
// 	const unsigned char& operator [](int i) const;
// 	bool operator < (const ROM& cmp) const;
// 	bool operator == (const ROM& cmp) const;
// 	bool isNull() const;
// 	const unsigned long long int& operator &() const;
// 	const char* toString() const;
// 	void zeroing();
// 	bool operator = (const char * str);
// private:
struct rom_t {
	unsigned char ROM_NO[8];
};

#endif /* DALLASONEWIRE_ROM_H_ */
