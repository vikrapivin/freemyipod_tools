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

/* The "fmss" backend's job planning, free of hardware access: it turns a
 * request (pages, banks, buffers) into the lists in memory and the parameter
 * RAM words a sequencer program takes. nand-fmss.c includes this header and
 * writes the result to the controller. It has no target dependencies, so host
 * code can include it unchanged. */
#ifndef __NAND_FMSS_PLAN_H__
#define __NAND_FMSS_PLAN_H__

/* emCORE has its own stdint (via nand-target.h); the host tool the compiler's */
#ifndef __NAND_TARGET_H__
#include <stdint.h>
#endif

/* Parameter RAM word offsets from the FMC base (0x38a00000) */
#define FMSS_P_BANKS    0xd04   /* banks (informational for most programs) */
#define FMSS_P_IDBUF    0xd08
#define FMSS_P_ROWS     0xd0c   /* READID: number of CEs */
#define FMSS_P_CELIST   0xd10
#define FMSS_P_CEMAP    0xd14   /* READID: reset delay loop count */
#define FMSS_P_COUNT    0xd18
#define FMSS_P_META     0xd1c
#define FMSS_P_DMALIST  0xd20
#define FMSS_P_RESULTS  0xd24
#define FMSS_P_CHUNKS   0xd28
#define FMSS_P_FLAGS    0xd44   /* program: bit 0 two-plane */
#define FMSS_P_WTIMING  0xd48
#define FMSS_P_TIMING   0xd4c

#define FMSS_MAXPAGES   32      /* pages per sequencer run */
#define FMSS_MAXUNITS   2       /* 2 KiB units per page (4 KiB pages) */
#define FMSS_NCE        8       /* chip enables the controller has */
#define FMSS_UNIT_BYTES 2048
#define FMSS_READID_DELAY 0x8000 /* reset settle loop count (2 insns each) */

/* Memory the sequencer reads and writes. Every array is a multiple of the
 * 32-byte cache line so a range can be cleaned or invalidated on its own. */
struct fmss_job
{
    uint32_t cemap[FMSS_NCE];                       /* index -> CE number */
    uint32_t idbuf[FMSS_NCE];                       /* READID answers */
    uint32_t rows[FMSS_MAXPAGES];
    uint32_t cel[FMSS_MAXPAGES];                    /* CE index per page */
    uint32_t dma[FMSS_MAXPAGES * FMSS_MAXUNITS];    /* bus address per unit */
    uint32_t res[FMSS_MAXPAGES];                    /* per-page read result */
    uint32_t meta[FMSS_MAXPAGES * 3];               /* 12 bytes per page */
} __attribute__((aligned(32)));

struct fmss_param
{
    uint32_t off;
    uint32_t val;
};

#define FMSS_MAXPARAMS 16
struct fmss_plan
{
    struct fmss_param p[FMSS_MAXPARAMS];
    unsigned int np;
};

static inline void fmss_plan_set(struct fmss_plan *pl, uint32_t off,
                                 uint32_t val)
{
    pl->p[pl->np].off = off;
    pl->p[pl->np].val = val;
    pl->np++;
}

/* Bus address of a member of a job placed at bus address base */
#define FMSS_JOB_ADDR(base, member) \
    ((uint32_t)(base) + (uint32_t)__builtin_offsetof(struct fmss_job, member))

/* Bus timing word ORed into FMC_CTRL0 (setup << 16 | hold << 12 | edo << 28):
 * three figures in ns (cycle, low, high) rounded up to bus clock periods of
 * 1000 / bus_mhz ns. */
static inline uint32_t fmss_timing_word(uint32_t cycle_ns, uint32_t low_ns,
                                        uint32_t high_ns, uint32_t bus_mhz)
{
    uint32_t t = bus_mhz && bus_mhz <= 1000 ? 1000 / bus_mhz : 1;
    uint32_t c1 = (cycle_ns + t - 1) / t;
    uint32_t c2 = (low_ns + t - 1) / t;
    uint32_t c3 = (high_ns + t - 1) / t;
    uint32_t h, s, e;

    if (c3 + c2 < c1)
        c3 = c1 - c2;
    h = c3 > 1 ? c3 - 1 : 1;
    s = c2 > 1 ? c2 - 1 : 1;
    e = h + 1 < 3 ? h + 1 : 3;
    return (s & 7) << 16 | (h & 7) << 12 | (e & 3) << 28;
}

static inline void fmss_plan_cemap(struct fmss_job *job)
{
    unsigned int i;

    for (i = 0; i < FMSS_NCE; i++)
        job->cemap[i] = i;
}

/* READID of CEs 0..n-1 */
static inline void fmss_plan_readid(struct fmss_plan *pl, struct fmss_job *job,
                                    uint32_t base, unsigned int n,
                                    uint32_t delay)
{
    unsigned int i;

    pl->np = 0;
    fmss_plan_cemap(job);
    for (i = 0; i < n; i++)
    {
        job->cel[i] = i;
        job->idbuf[i] = 0;
    }
    fmss_plan_set(pl, FMSS_P_IDBUF, FMSS_JOB_ADDR(base, idbuf));
    fmss_plan_set(pl, FMSS_P_ROWS, n);
    fmss_plan_set(pl, FMSS_P_CELIST, FMSS_JOB_ADDR(base, cel));
    fmss_plan_set(pl, FMSS_P_CEMAP, delay);
}

/* READ n pages (n <= FMSS_MAXPAGES); buf[i] is the bus address of page i's
 * data buffer, units the 2 KiB units per page */
static inline void fmss_plan_read(struct fmss_plan *pl, struct fmss_job *job,
                                  uint32_t base, unsigned int banks,
                                  unsigned int units, unsigned int n,
                                  const uint32_t *bank, const uint32_t *page,
                                  const uint32_t *buf, uint32_t timing)
{
    unsigned int i, u;

    pl->np = 0;
    fmss_plan_cemap(job);
    for (i = 0; i < n; i++)
    {
        job->rows[i] = page[i];
        job->cel[i] = bank[i];
        for (u = 0; u < units; u++)
            job->dma[i * units + u] = buf[i] + u * FMSS_UNIT_BYTES;
        job->res[i] = 0;
    }
    fmss_plan_set(pl, FMSS_P_BANKS, banks);
    fmss_plan_set(pl, FMSS_P_ROWS, FMSS_JOB_ADDR(base, rows));
    fmss_plan_set(pl, FMSS_P_CELIST, FMSS_JOB_ADDR(base, cel));
    fmss_plan_set(pl, FMSS_P_CEMAP, FMSS_JOB_ADDR(base, cemap));
    fmss_plan_set(pl, FMSS_P_COUNT, n);
    fmss_plan_set(pl, FMSS_P_META, FMSS_JOB_ADDR(base, meta));
    fmss_plan_set(pl, FMSS_P_DMALIST, FMSS_JOB_ADDR(base, dma));
    fmss_plan_set(pl, FMSS_P_RESULTS, FMSS_JOB_ADDR(base, res));
    fmss_plan_set(pl, FMSS_P_CHUNKS, units);
    fmss_plan_set(pl, FMSS_P_TIMING, timing);
}

/* PROGRAM n pages; meta[i] points at page i's 3 spare words. two_plane: the
 * pages come in pairs on one bank (plane 0, then plane 1) */
static inline void fmss_plan_write(struct fmss_plan *pl, struct fmss_job *job,
                                   uint32_t base, unsigned int banks,
                                   unsigned int units, unsigned int n,
                                   const uint32_t *bank, const uint32_t *page,
                                   const uint32_t *buf,
                                   const uint32_t *const *meta,
                                   int two_plane, uint32_t wtiming)
{
    unsigned int i, u;

    pl->np = 0;
    fmss_plan_cemap(job);
    for (i = 0; i < n; i++)
    {
        job->rows[i] = page[i];
        job->cel[i] = bank[i];
        for (u = 0; u < units; u++)
            job->dma[i * units + u] = buf[i] + u * FMSS_UNIT_BYTES;
        job->meta[i * 3 + 0] = meta[i][0];
        job->meta[i * 3 + 1] = meta[i][1];
        job->meta[i * 3 + 2] = meta[i][2];
    }
    fmss_plan_set(pl, FMSS_P_BANKS, banks);
    fmss_plan_set(pl, FMSS_P_ROWS, FMSS_JOB_ADDR(base, rows));
    fmss_plan_set(pl, FMSS_P_CELIST, FMSS_JOB_ADDR(base, cel));
    fmss_plan_set(pl, FMSS_P_CEMAP, FMSS_JOB_ADDR(base, cemap));
    fmss_plan_set(pl, FMSS_P_COUNT, n);
    fmss_plan_set(pl, FMSS_P_META, FMSS_JOB_ADDR(base, meta));
    fmss_plan_set(pl, FMSS_P_DMALIST, FMSS_JOB_ADDR(base, dma));
    fmss_plan_set(pl, FMSS_P_CHUNKS, units);
    fmss_plan_set(pl, FMSS_P_FLAGS, two_plane ? 1 : 0);
    fmss_plan_set(pl, FMSS_P_WTIMING, wtiming);
}

/* ERASE n blocks of pagesperblock pages (every bank at most once) */
static inline void fmss_plan_erase(struct fmss_plan *pl, struct fmss_job *job,
                                   uint32_t base, unsigned int banks,
                                   unsigned int pagesperblock, unsigned int n,
                                   const uint32_t *bank,
                                   const uint32_t *block, uint32_t timing)
{
    unsigned int i;

    pl->np = 0;
    fmss_plan_cemap(job);
    for (i = 0; i < n; i++)
    {
        job->rows[i] = block[i] * pagesperblock;
        job->cel[i] = bank[i];
    }
    fmss_plan_set(pl, FMSS_P_BANKS, banks);
    fmss_plan_set(pl, FMSS_P_ROWS, FMSS_JOB_ADDR(base, rows));
    fmss_plan_set(pl, FMSS_P_CELIST, FMSS_JOB_ADDR(base, cel));
    fmss_plan_set(pl, FMSS_P_CEMAP, FMSS_JOB_ADDR(base, cemap));
    fmss_plan_set(pl, FMSS_P_COUNT, n);
    fmss_plan_set(pl, FMSS_P_TIMING, timing);
}

#endif
