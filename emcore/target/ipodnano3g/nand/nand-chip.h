/*
 * Copyright (C) 2026 Vikotr Krapivin
 *
 * This file is part of emCORE.
 *
 * emCORE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * emCORE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with emCORE. If not, see <http://www.gnu.org/licenses/>.
 */

/* Chip table and geometry shared by the nano 3G NAND backends. The table
 * lives in nand-swnand.c; both backends match a chip with nand_chip_find()
 * and derive the geometry with nand_chip_geometry(). */
#ifndef __NAND_CHIP_H__
#define __NAND_CHIP_H__

#include "nand-target.h"

struct nand_chip_info
{
    uint32_t id;            /* first four ID bytes, little endian */
    uint8_t banks;          /* chip enables holding this part */
    uint8_t mode;           /* the original firmware's layout/programming mode */
    uint16_t blocks;        /* per chip enable */
    uint16_t pagesperblock;
    uint16_t pagesize;      /* bytes of data per page */
    uint16_t userblocks;    /* per chip enable, as Whimory counts them */
    bool validated;         /* read, written and remounted on hardware */
    /* Bus timing figures in ns {cycle, low, high} for reads/erases and for
     * programs: the original firmware's table values */
    uint8_t read_ns[3];
    uint8_t write_ns[3];
};

/* The row matching the id and the number of chip enables answering with it,
 * or NULL */
const struct nand_chip_info *nand_chip_find(uint32_t id, unsigned int banks);
/* index of that row in the table, -1 if none */
int nand_chip_row(uint32_t id, unsigned int banks);

/* Geometry of a matched row (the same for every backend) */
void nand_chip_geometry(const struct nand_chip_info *chip, unsigned int banks,
                        struct nand_geometry *geo);

/* Enable the NAND and ECC clock gates and mux the pads */
void nand_hw_enable(void);

#endif
