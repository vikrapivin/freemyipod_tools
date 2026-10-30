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

/* Physical NAND backend interface of the nano 3G target.
 *
 * The FTL (ftl-nano3g.c), the monitor jobs (../nand3g.c) and the benchmark
 * all reach the chips through the functions of nand-target.h, which dispatch
 * to the ACTIVE backend. A backend is a const struct nand_backend:
 *   "sw"   nand-swnand.c   CPU-sequenced FMC register driver (ported)
 *   "fmss" nand-fmss.c     FMSS microcode driver (sequencer programs, nand/fmss/*.s)
 * To add a backend: implement the struct, add it to nand_backends[] in
 * nand-backend.c, list the file in SOURCES. All batch calls take arrays so
 * a microcode backend can queue a whole batch before waiting.
 */
#ifndef __NAND_BACKEND_H__
#define __NAND_BACKEND_H__

#include "nand-target.h"

struct nand_backend
{
    const char *name;
    /* Bring up clocks/pins, reset and identify all banks, match the chip,
     * fill the geometry. 0 or NAND_ERR_*. */
    int (*init)(void);
    int (*reset)(uint32_t bank);
    /* reset-free READ ID of a bank (the sw backend resets it first) */
    int (*identify)(uint32_t bank, uint32_t *id, uint32_t *ext);
    /* as nand_read_pages(), nand_write_pages() of nand-target.h */
    int (*read_pages)(struct nand_read *r, unsigned int n);
    int (*write_pages)(const struct nand_write *w, unsigned int n,
                       bool two_plane, uint32_t *failbank);
    /* erase n blocks; on failure *failidx = index of the failing entry */
    int (*erase_blocks)(const struct nand_erase *e, unsigned int n,
                        unsigned int *failidx);
    const struct nand_geometry *(*get_geometry)(void);
    uint32_t (*get_id)(void);
};

#define NAND_BACKEND_SW   0
#define NAND_BACKEND_FMSS 1
#define NAND_NBACKENDS    2

extern const struct nand_backend nand_backend_sw;
extern const struct nand_backend nand_backend_fmss;

/* Select the active backend (also resets "initialised"). Returns 0, or
 * NAND_ERR_NOINIT for a bad index. */
int nand_backend_select(unsigned int idx);
const struct nand_backend *nand_backend_get(unsigned int idx);
const struct nand_backend *nand_backend_active(void);
unsigned int nand_backend_active_index(void);
/* init() of the active backend; remembers success */
int nand_backend_init(void);
bool nand_backend_ready(void);

/* Dispatch helpers for batches (nand-target.h's single page calls use them) */
int nand_erase_blocks(const struct nand_erase *e, unsigned int n,
                      unsigned int *failidx);

#endif
