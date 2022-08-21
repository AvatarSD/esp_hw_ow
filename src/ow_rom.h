/*
 * ROM.h
 *
 *  Created on: 31 серп. 2015 р.
 *      Author: sd
 */

#ifndef DALLASONEWIRE_ROM_H_
#define DALLASONEWIRE_ROM_H_

#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>

typedef struct {
    union {
        unsigned char no[8];
        uint64_t raw;
    };
} rom_t;

static_assert(sizeof(rom_t) == 8);

bool rom_is_match_family(const rom_t *rom, unsigned char family);
bool rom_less_then(const rom_t *rom, const rom_t *cmp);
const char *rom_to_string(const rom_t *rom);
bool rom_is_equal(const rom_t *rom, const rom_t *cmp);
bool rom_is_null(const rom_t *rom);

void rom_zeroing(rom_t *rom);
bool rom_from_string(rom_t *rom, const char *str);

#endif /* DALLASONEWIRE_ROM_H_ */
