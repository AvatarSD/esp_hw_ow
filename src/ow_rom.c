/*
 * ROM.cpp
 *
 *  Created on: 31 серп. 2015 р.
 *      Author: sd
 */

#include "ow_rom.h"

#include <stdbool.h>

#include "stdio.h"

#define ROM_CELLS_COUNT(x) (sizeof(x->no) / sizeof(x->no[0]))

bool rom_is_match_family(const struct rom_t *rom, unsigned char family) {
    return (family == rom->no[0]);
}

bool rom_less_then(const struct rom_t *rom, const struct rom_t *cmp) { return rom->raw < cmp->raw; }

const char *rom_to_string(const struct rom_t *rom) {
    static char buf[17];
    for (unsigned char i = 0; i < ROM_CELLS_COUNT(rom); i++)
        sprintf(&buf[i * 2], "%02X", rom->no[i]);
    return buf;
}

bool rom_is_equal(const struct rom_t *rom,const struct rom_t *cmp) {
    for (unsigned char i = 0; i < ROM_CELLS_COUNT(rom); i++)
        if (rom->no[i] != cmp->no[i]) return false;
    return true;
}

bool rom_is_null(const struct rom_t *rom) {
    for (unsigned char i = 0; i < ROM_CELLS_COUNT(rom); i++)
        if (rom->no[i] != 0) return false;
    return true;
}

void rom_zeroing(struct rom_t *rom) {
    for (unsigned char i = 0; i < sizeof(rom->no) / sizeof(rom->no[0]); i++) rom->no[i] = 0x00;
}

bool rom_from_string(struct rom_t *rom, const char *str) {
    sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", rom->no[0], rom->no[1], rom->no[2],
           rom->no[3], rom->no[4], rom->no[5], rom->no[6], rom->no[7]);

    //	sscanf(str, "%llx", (unsigned long long *)rom);

    return true;
}
