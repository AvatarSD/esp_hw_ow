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

struct rom_t {
    union {
        unsigned char no[8];
        uint64_t raw;
    };
};

static_assert(sizeof(struct rom_t) == 8);

bool rom_is_match_family(const struct rom_t *rom, unsigned char family);
bool rom_less_then(const struct rom_t *rom, const struct rom_t *cmp);
const char *rom_to_string(const struct rom_t *rom);
bool rom_is_equal(const struct rom_t *rom, const struct rom_t *cmp);
bool rom_is_null(const struct rom_t *rom);

void rom_zeroing(struct rom_t *rom);
bool rom_from_string(struct rom_t *rom, const char *str);

#endif /* DALLASONEWIRE_ROM_H_ */
