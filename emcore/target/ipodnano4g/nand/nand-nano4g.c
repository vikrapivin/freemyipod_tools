/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2009 by Michael Sparmann
 * Copyright (C) 2026 by Andrew Rice
 * Copyright (C) 2026 by Robin Tschirschnitz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

/*
 * NAND driver for the iPod Nano 4G (S5L8720 FMI).
 *
 * The register block is the same FMI aperture as the Nano 3G (0x38a00000).
 * Identify and the raw read follow that driver's NANDReadPage
 * sequence: command, five address cycles, then 512-byte chunks. A 4 KiB
 * page is eight chunks. The pair index that the 2 KiB driver stored in one
 * bit (FMADDR6 0x00/0x10, ECC kick bit 16) is the chunk number shifted
 * right by one, so chunks 0..3 issue the same writes as on the Nano 3G and
 * chunks 4..7 continue the same step (0x20/0x30, bits 17..18).
 *
 * wInd3x's Nano 4G NAND hooks are unimplemented, so this is that sequence
 * on the S5L8720 clock gates (NAND = 5, NANDECC = 9) rather than a second
 * disassembly. Pinmux is left to gpio_preinit(); the Nano 3G PCON values
 * are a different board.
 *
 * Page reads and programs run the FMSS sequencer programs.
 */

#include "rockbox-compat.h"
#include "nand-target.h"
#include "nand-profile.h"

#define NAND_CMD_READ           0x00
#define NAND_CMD_READ2          0x30
#define NAND_CMD_READSTATUS     0x70
#define NAND_CMD_READID         0x90
#define NAND_CMD_RESET          0xFF

#define NAND_STATUS_READY       (1 << 6)

#define NAND_CHUNK_SIZE         512
#define NAND_CHUNKS_PER_PAGE    (NAND_PAGE_SIZE / NAND_CHUNK_SIZE)
#define NAND_MAX_BANKS          4

/* Same timings the Nano 3G BootROM writes into FMCTRL0. */
#define NAND_TUNK1              4
#define NAND_TWP                3
#define NAND_POLL_SPINS         1000000

struct nand_chip_info
{
    uint32_t id;
    unsigned int blocks;
    unsigned int pagesperblock;
    unsigned int planes;
    unsigned int userblocks;
    unsigned int vflspares;
    bool twoplane;
};

static struct nand_chip_info nand_chip_decoded;
static const struct nand_chip_info *nand_chip;
static unsigned int nand_banks;
static uint32_t nand_id;
static bool nand_ready = false;
static long nand_last_activity_value = -1;
static struct nand_geometry nand_geo;

const struct nand_geometry *nand_get_geometry(void)
{
    return nand_ready ? &nand_geo : NULL;
}

static void nand_fill_geo(void)
{
    nand_geo.banks = nand_banks;
    nand_geo.blocks = nand_chip->blocks;
    nand_geo.pagesperblock = nand_chip->pagesperblock;
    nand_geo.planes = nand_chip->planes;
    nand_geo.userblocks = nand_chip->userblocks;
    nand_geo.vflspares = nand_chip->vflspares;
    nand_geo.twoplane = nand_chip->twoplane;
}

/* Addresses outside the detected chip. Block 0 is a normal block. */
static bool nand_page_ok(uint32_t bank, uint32_t page)
{
    return nand_ready && nand_chip && bank < nand_banks
        && page < (uint32_t)nand_chip->blocks * nand_chip->pagesperblock;
}

static void nand_touch(void)
{
    nand_last_activity_value = current_tick;
}

static void nand_set_fmctrl0(uint32_t bank)
{
    FMCTRL0 = (NAND_TUNK1 << 16) | (NAND_TWP << 12) | FMCTRL0_UNK1
            | 1 | (1 << (bank + 1));
}

static int nand_wait_reg(volatile uint32_t *reg, uint32_t bit)
{
    unsigned long spins = NAND_POLL_SPINS;
    while (spins--)
    {
        if (*reg & bit)
        {
            *reg = bit;
            return 0;
        }
    }
    return -1;
}

static int nand_wait_stat(uint32_t bit)
{
    return nand_wait_reg(&FMCSTAT, bit);
}

static int nand_wait_stat_noclear(uint32_t bit)
{
    unsigned long spins = NAND_POLL_SPINS;
    while (spins--)
        if (FMCSTAT & bit)
            return 0;
    return -1;
}

static void nand_clear_status(void)
{
    FMCSTAT = 0xffffffff;
}

static int nand_send_cmd(uint32_t cmd)
{
    FMCMD = cmd;
    return nand_wait_stat(FMCSTAT_CMDDONE);
}

static int nand_send_addr_byte(uint32_t addr)
{
    FMANUM = 0;
    FMADDR0 = addr;
    FMCTRL1 = FMCTRL1_DOTRANSADDR;
    return nand_wait_stat(FMCSTAT_ADDRDONE);
}

static int nand_read_status(void)
{
    unsigned long timeout = USEC_TIMER + 200000;
    unsigned long spins;
    uint32_t status = 0;

    FMADDR2 = 1;
    if (nand_send_cmd(NAND_CMD_READSTATUS))
        return -1;
    do
    {
        FMCTRL1 = 0xca;
        for (spins = 10000; spins; spins--)
        {
            status = FMSYND0;
            if (status & NAND_STATUS_READY)
                break;
        }
    } while (!(status & NAND_STATUS_READY)
             && TIME_BEFORE(USEC_TIMER, timeout));
    if (!(status & NAND_STATUS_READY))
        return -1;
    FMCSTAT = FMCSTAT_STATUSREADY;
    FMCTRL1 = 0x20;
    return status & 0xff;
}

static int nand_wait_ready(void)
{
    int status = nand_read_status();

    FMADDR2 = 0;
    FMCTRL1 = 0xe0;
    return status < 0 ? NAND_ERR_READY_CMD : 0;
}

static int nand_reset(uint32_t bank)
{
    clockgate_enable(CLOCKGATE_NAND, true);
    clockgate_enable(CLOCKGATE_NANDECC, true);

    nand_set_fmctrl0(bank);
    if (nand_send_cmd(NAND_CMD_RESET))
        return NAND_ERR_RESET_CMD;

    udelay(5000);
    return nand_wait_ready();
}

static int nand_identify(uint32_t *id)
{
    if (nand_send_cmd(NAND_CMD_READID))
        return -1;
    if (nand_send_addr_byte(0))
        return -1;

    FMDNUM = 7;
    FMADDR2 = 1;
    FMCSTAT = FMCSTAT_TRANSDONE;
    FMCTRL1 = FMCTRL1_DOREADDATA | FMCTRL1_CLEARWFIFO | FMCTRL1_CLEARRFIFO
            | (1 << 8);
    if (nand_wait_stat_noclear(FMCSTAT_TRANSDONE))
        return -1;
    FMADDR2 = 0x100;
    FMCTRL1 = FMCTRL1_CLEARWFIFO | (1 << 8) | (1 << 9);

    *id = FMFIFO;
    nand_clear_status();
    return 0;
}

/* JEDEC extended ID: page = 1024 << (id[3] & 3), erase = 64 KiB << ((id[3] >> 4) & 3).
 * id is the first four ID bytes, little endian, as READ ID returns them. */
static int nand_decode(uint32_t id, struct nand_chip_info *out)
{
    uint8_t dev = (id >> 8) & 0xff;
    uint8_t ext = (id >> 24) & 0xff;
    unsigned int pagesize = 1024u << (ext & 3);
    unsigned int erasesize = 65536u << ((ext >> 4) & 3);
    unsigned int mbytes;
    unsigned int blocks;
    unsigned int ppb;

    if (pagesize != NAND_PAGE_SIZE || erasesize < pagesize)
        return -1;
    switch (dev)
    {
        case 0xD3: mbytes = 1024; break;    /* 8 Gbit */
        case 0xD5: mbytes = 2048; break;    /* 16 Gbit */
        case 0xD7: mbytes = 4096; break;    /* 32 Gbit */
        case 0xDE: mbytes = 8192; break;    /* 64 Gbit, TH58NVG6 class */
        case 0x3A: mbytes = 16384; break;   /* 128 Gbit */
        case 0xDC: mbytes = 512; break;     /* 4 Gbit */
        default: return -1;
    }
    ppb = erasesize / pagesize;
    blocks = (mbytes * (1024u * 1024u / pagesize)) / ppb;
    if (!blocks || !ppb)
        return -1;

    out->id = id;
    out->blocks = blocks;
    out->pagesperblock = ppb;
    out->planes = 2;
    /* 23 spare superblocks, same allowance the Nano 3G VFL uses, plus a
     * small remap pool the context can replace once it is loaded. */
    out->vflspares = 16;
    if (blocks <= out->planes * (out->vflspares + 23))
        return -1;
    out->userblocks = blocks - out->planes * (out->vflspares + 23);
    out->twoplane = true;
    return 0;
}

/* S5L8720 ECC engine, from the BootROM page read at 0x200092b0. The
 * Nano 3G offsets (0x80c kick, 0x840 status) do not apply here. */
#define FMECC_KICK      (*(REG32_PTR_T)(FMC_BASE + 0x0804))
#define FMECC_RESULT    (*(REG32_PTR_T)(FMC_BASE + 0x080C))
#define FMECC_STATUS    (*(REG32_PTR_T)(FMC_BASE + 0x0810))

/* The BootROM's own ready wait: READ STATUS, then FMCTRL1 0x2a has the
 * controller poll until the part is ready and set FMCSTAT bit 23. */
static int nand_wait_ready_rom(void)
{
    int rc = 0;

    FMCTRL1 = 0x1000e0;
    FMCMD = NAND_CMD_READSTATUS;
    if (nand_wait_stat(FMCSTAT_CMDDONE))
        rc = -1;
    else
    {
        FMCTRL1 = 0x2a;
        if (nand_wait_stat(FMCSTAT_STATUSREADY))
            rc = -1;
    }
    FMADDR2 = 0;
    FMCTRL1 = 0xe0;
    return rc;
}

/* READ, row address, READ2, wait, READ: the page is then streamed out
 * from column 0 */
static int nand_start_read(uint32_t bank, uint32_t page)
{
    FMCTRL1 = 0x1f0e0;
    nand_set_fmctrl0(bank);
    FMCMD = NAND_CMD_READ;
    if (nand_wait_stat(FMCSTAT_CMDDONE))
        return -1;
    FMANUM = 4;
    FMADDR0 = page << 16;
    FMADDR1 = (page >> 16) & 0xff;
    FMCTRL1 = 0xe1;
    if (nand_wait_stat(FMCSTAT_ADDRDONE))
        return -2;
    FMCMD = NAND_CMD_READ2;
    if (nand_wait_stat(FMCSTAT_CMDDONE))
        return -3;
    if (nand_wait_ready_rom())
        return -4;
    FMCMD = NAND_CMD_READ;
    if (nand_wait_stat(FMCSTAT_CMDDONE))
        return -5;
    return 0;
}

static void nand_end_read(void)
{
    FMCTRL0 = 0x43802;
    FMCTRL0 &= ~0x1ff;
    FMCSTAT = 0xffffffff;
    FMECC_STATUS = 0x1ff;
}

/* No ECC: the next len bytes of the page stream through the FIFO, 32 at a
 * time, the way READ ID is read. The last word may be partly filled. */
static int nand_fifo_read(void *buf, unsigned int len)
{
    uint32_t *w = buf;
    unsigned int done, n, i;

    for (done = 0; done < len; done += n)
    {
        n = MIN(len - done, 32u);
        FMDNUM = n - 1;
        FMADDR2 = 1;
        FMCSTAT = FMCSTAT_TRANSDONE;
        FMCTRL1 = FMCTRL1_DOREADDATA | FMCTRL1_CLEARWFIFO
                | FMCTRL1_CLEARRFIFO | (1 << 8);
        if (nand_wait_stat_noclear(FMCSTAT_TRANSDONE))
            return -6;
        FMADDR2 = 0x100;
        FMCTRL1 = FMCTRL1_CLEARWFIFO | (1 << 8) | (1 << 9);
        for (i = 0; i < (n + 3) / 4; i++)
            *w++ = FMFIFO;
    }
    return 0;
}

/* The Whimory page format, as Apple's EFI NAND driver reads it: chunk 0 is
 * 10 bytes of spare metadata, the parity and 512 data bytes, with one BCH
 * codeword over metadata and data; chunks 1 to 7 are parity + 512 data
 * bytes. meta[2] reads 0xffff above the byte-8 flag and the spare type.
 *
 * Most parts take 8-bit BCH, 13 parity bytes (decode modes 0xa01 and 1).
 * The Toshiba 98 D5 94 BA in an 8 GB player takes 16-bit, 26 parity bytes
 * (0xa05 and 5), the strength of the boot block: the 8-bit decode fails
 * every programmed page there and passes erased ones (measured). */
#define NAND_PAGE_ECCLEN    13
#define NAND_KICK_META      0xa01
#define NAND_KICK_DATA      0x001
#define NAND_PAGE_ECCLEN16  26
#define NAND_KICK_META16    0xa05
#define NAND_KICK_DATA16    0x005

static unsigned int nand_ecclen = 13;
#define NAND_KICK_BCH16 (nand_ecclen == 26 ? 4 : 0)

/* The first len bytes (a multiple of 4) of a page, no ECC. Diagnostic. */
int nand_read_raw(uint32_t bank, uint32_t page, void *buf, unsigned int len)
{
    int rc;

    if (!nand_page_ok(bank, page) || !buf || !len)
        return -1;
    if (lcd_wait_write_dma())
        return -3;
    rc = nand_start_read(bank, page);

    if (!rc)
        rc = nand_fifo_read(buf, len);
    nand_end_read();
    return rc;
}

#include "fmss-nano4g.h"

#define FMISS_REG(off) (*(volatile uint32_t *)(FMC_BASE + (off)))
#define FMISS_RAM_ALIAS(p) ((uint32_t)(uintptr_t)(p)) /* emCORE identity DMA map */

#define FMISS_POLL_SPINS    100000000
#define FMISS_WRITE_BYTES   0x600

/* FMISS register r6 after erase.s / program.s: bit 0 the chip reported a
 * failed operation, bit 27 a sequencer fault (aborted) */
#define FMISS_R6_FAILED     0x00000001
#define FMISS_R6_ABORTED    0x08000000

/* Parameter words of the fmss/ programs */
#define FMISS_P_ROWS        0x0d0c
#define FMISS_P_CEMASKS     0x0d10
#define FMISS_P_ECCBUF      0x0d14
#define FMISS_P_COUNT       0x0d18
#define FMISS_P_META        0x0d1c
#define FMISS_P_DMALIST     0x0d20
#define FMISS_P_STATS       0x0d24
#define FMISS_P_TIMING      0x0d4c

static uint8_t nand_fmiss_page[NAND_PAGE_SIZE] CACHEALIGN_ATTR;
static unsigned int nand_backend;

void nand_set_cpu_optimized(bool enabled) { (void)enabled; }

int nand_set_backend(unsigned int backend)
{
    if (backend != NAND_BACKEND_M2) return -1;
    nand_backend = backend;
    return 0;
}
unsigned int nand_get_backend(void) { return nand_backend; }
int nand_set_ecc_bits(unsigned int bits)
{
    if (bits != 8 && bits != 16) return -1;
    nand_ecclen = bits == 16 ? 26 : 13;
    return 0;
}


static int nand_fmiss_run(const void *program, size_t len)
{
    return n4g_fmss_run(program, len);
}

static void nand_fmiss_cleanup(void)
{
    unsigned int previous = nand_profile_enter(NAND_PROFILE_CLEANUP);
    FMCTRL1 = 0x20;
    FMCTRL0 = 0;
    FMCSTAT = 0xffffffff;
    nand_profile_leave(previous);
}

static bool nand_fmiss_target_ok(uint32_t bank, uint32_t block)
{
    return nand_ready && nand_chip && bank < nand_banks
        && block < nand_chip->blocks;
}

/* The FMISS calls below return 0, the nand_fmiss_run() error, or -10 when
 * the program ran and the chip or the result check refused the operation */

#define NAND_WRITE_TIMING   0x00077000

/* fmss/erase.s: P_ROWS the first page of the block, P_CEMASKS 1 << bank */
static int nand_fmiss_erase(uint32_t bank, uint32_t block)
{
    uint32_t r6;
    int rc;

    if (lcd_wait_write_dma())
        return -3;
    /* Prepare the engine before parameter writes; run must preserve them. */
    if (n4g_fmss_reset())
        return -2;
    FMISS_REG(FMISS_P_ROWS) = block * nand_chip->pagesperblock;
    FMISS_REG(FMISS_P_CEMASKS) = 1u << bank;
    FMISS_REG(FMISS_P_TIMING) = NAND_WRITE_TIMING;
    rc = nand_fmiss_run(fmss_prog_erase, sizeof(fmss_prog_erase));
    r6 = FMISS_REG(0x0c30);
    if (!rc && (r6 & FMISS_R6_ABORTED))
        rc = -4;
    else if (!rc && (r6 & FMISS_R6_FAILED))
        rc = -10;
    if (rc >= -1 || rc == -4 || rc == -10)
        nand_fmiss_cleanup();
    return rc;
}

/* fmss/program.s: one 4 KiB page from P_DMALIST with the 10 metadata bytes
 * at P_META, eight codewords. 0x0d2c/0x0d30 are the encode modes of
 * chunks 1..7 and of chunk 0 (with the metadata), 0x0d34/0x0d38 the bytes
 * before chunk 0's and chunks 1..7's data, less one. */
static uint32_t nand_fmiss_meta[4] CACHEALIGN_ATTR;

static int nand_fmiss_program_page(uint32_t bank, uint32_t page, const void *data,
                              const uint32_t *meta)
{
    uint32_t r6;
    int rc;

    memcpy(nand_fmiss_page, data, NAND_PAGE_SIZE);
    if (meta)
        memcpy(nand_fmiss_meta, meta, NAND_META_WORDS * 4);
    else
        memset(nand_fmiss_meta, 0xff, NAND_META_WORDS * 4);
    commit_dcache_range(nand_fmiss_page, sizeof(nand_fmiss_page));
    commit_dcache_range(nand_fmiss_meta, sizeof(nand_fmiss_meta));

    if (lcd_wait_write_dma())
        return -3;
    /* Prepare the engine before parameter writes; run must preserve them. */
    if (n4g_fmss_reset())
        return -2;
    FMISS_REG(FMISS_P_ROWS) = page;
    FMISS_REG(FMISS_P_CEMASKS) = 1u << bank;
    FMISS_REG(FMISS_P_META) = FMISS_RAM_ALIAS(nand_fmiss_meta);
    FMISS_REG(FMISS_P_DMALIST) = FMISS_RAM_ALIAS(nand_fmiss_page);
    FMISS_REG(0x0d2c) = 0x002 | NAND_KICK_BCH16;
    FMISS_REG(0x0d30) = 0xa02 | NAND_KICK_BCH16;
    FMISS_REG(0x0d34) = 10 + nand_ecclen - 1;
    FMISS_REG(0x0d38) = nand_ecclen - 1;
    FMISS_REG(FMISS_P_TIMING) = NAND_WRITE_TIMING;
    rc = nand_fmiss_run(fmss_prog_program, sizeof(fmss_prog_program));
    r6 = FMISS_REG(0x0c30);
    if (!rc && (r6 & FMISS_R6_ABORTED))
        rc = -4;
    else if (!rc && (r6 & FMISS_R6_FAILED))
        rc = -10;
    if (rc >= -1 || rc == -4 || rc == -10)
        nand_fmiss_cleanup();
    return rc;
}

/* Keep the batched write API while the source program handles one page
 * per sequencer run. Report the exact bank of the first failed page. */
#define NAND_FMISS_PROGRAM_BATCH NAND_MAX_BANKS
static int nand_fmiss_program(const struct nand_write *w, unsigned int n,
                              uint32_t *failbank)
{
    for (unsigned int i = 0; i < n; i++)
    {
        int rc = nand_fmiss_program_page(w[i].bank, w[i].page,
                                        w[i].buf, w[i].meta);
        if (rc)
        {
            if (failbank)
                *failbank = w[i].bank;
            return rc;
        }
    }
    return 0;
}

/* fmss/read.s reads pages from the lists at P_ROWS and P_CEMASKS (P_COUNT
 * pages) to the 2 KiB DMA destinations listed at P_DMALIST, 12 metadata
 * bytes per page to P_META and one status byte per chunk to P_ECCBUF:
 * 0xff uncorrectable, 0xfe erased, else the corrected bits. r6 is the
 * summary: bit 30 a chunk was uncorrectable, bit n (0..16) a chunk had n
 * bits corrected, bit 27 a sequencer fault. The lists are read and
 * advanced in place by the program. Plain bus addresses, as the reads were
 * proven with. */
#define FMISS_R6_UECC       0x40000000
#define FMISS_R6_CORRECTED  0x0001fffe
#define NAND_READ_TIMING    ((NAND_TUNK1 << 16) | (NAND_TWP << 12))

static struct
{
    uint32_t row, cemask, dma[2], meta[NAND_META_WORDS];
    uint8_t stat[NAND_CHUNKS_PER_PAGE];
} nand_rd CACHEALIGN_ATTR;

/* Apple's READ program (fmss/read.s) consumes a list. One run reads every
 * page: P_COUNT, a row and a CE mask each, two 2 KiB DMA addresses, 12
 * metadata bytes and eight sector-status bytes. */
#define FMSS_READ_SPAN 32u
static struct
{
    uint32_t row[FMSS_READ_SPAN];
    uint32_t cemask[FMSS_READ_SPAN];
    uint32_t dma[FMSS_READ_SPAN * 2];
    uint32_t meta[FMSS_READ_SPAN * NAND_META_WORDS];
    uint8_t stat[FMSS_READ_SPAN * NAND_CHUNKS_PER_PAGE];
} nand_fmiss_span CACHEALIGN_ATTR;

static int nand_stat_result(uint8_t *buf, uint32_t *meta, const uint8_t *stat)
{
    unsigned int i, blank = 0, bad = 0, corrected = 0;

    for (i = 0; i < NAND_CHUNKS_PER_PAGE; i++)
    {
        if (stat[i] == 0xfe)
            blank++;
        else if (stat[i] == 0xff)
            bad++;
        else if (stat[i])
            corrected = 1;
    }
    if (blank == NAND_CHUNKS_PER_PAGE
        || (bad < 3 && blank + bad == NAND_CHUNKS_PER_PAGE))
    {
        memset(buf, 0xff, NAND_PAGE_SIZE);
        if (meta)
            memset(meta, 0xff, NAND_META_WORDS * 4);
        return NAND_ECC_CLEAN;
    }
    if (bad)
        return NAND_ECC_FAILED;
    return corrected ? NAND_ECC_CORRECTED : NAND_ECC_CLEAN;
}

/* One READ-program run. Caller has checked geometry and alignment. */
static int nand_fmiss_read_span(struct nand_read *r, unsigned int n)
{
    unsigned int i;
    uint32_t r6;
    int rc;

    for (i = 0; i < n; i++)
    {
        nand_fmiss_span.row[i] = r[i].page;
        nand_fmiss_span.cemask[i] = 1u << r[i].bank;
        nand_fmiss_span.dma[2 * i] = (uint32_t)(intptr_t)r[i].buf;
        nand_fmiss_span.dma[2 * i + 1] = (uint32_t)(intptr_t)r[i].buf + NAND_PAGE_SIZE / 2;
        discard_dcache_range(r[i].buf, NAND_PAGE_SIZE);
    }
    commit_discard_dcache_range(&nand_fmiss_span, sizeof(nand_fmiss_span));
    if (lcd_wait_write_dma())
        return -3;
    if (n4g_fmss_reset())
        return -2;
    FMISS_REG(FMISS_P_ROWS) = (uint32_t)(intptr_t)nand_fmiss_span.row;
    FMISS_REG(FMISS_P_CEMASKS) = (uint32_t)(intptr_t)nand_fmiss_span.cemask;
    FMISS_REG(FMISS_P_COUNT) = n;
    FMISS_REG(FMISS_P_META) = (uint32_t)(intptr_t)nand_fmiss_span.meta;
    FMISS_REG(FMISS_P_DMALIST) = (uint32_t)(intptr_t)nand_fmiss_span.dma;
    FMISS_REG(FMISS_P_ECCBUF) = (uint32_t)(intptr_t)nand_fmiss_span.stat;
    FMISS_REG(FMISS_P_STATS) = 0;
    FMISS_REG(0x0d2c) = NAND_KICK_DATA | NAND_KICK_BCH16;
    FMISS_REG(0x0d30) = NAND_KICK_META | NAND_KICK_BCH16;
    FMISS_REG(0x0d34) = 10 + nand_ecclen - 1;
    FMISS_REG(0x0d38) = nand_ecclen - 1;
    FMISS_REG(FMISS_P_TIMING) = NAND_READ_TIMING;
    rc = nand_fmiss_run(fmss_prog_read, sizeof(fmss_prog_read));
    r6 = FMISS_REG(0x0c30);
    if (!rc && (r6 & FMISS_R6_ABORTED))
        rc = -4;
    if (rc >= -1 || rc == -4)
        nand_fmiss_cleanup();
    nand_touch();
    if (rc)
        return rc;
    discard_dcache_range(&nand_fmiss_span, sizeof(nand_fmiss_span));
    for (i = 0; i < n; i++)
    {
        uint32_t *meta = r[i].meta;
        discard_dcache_range(r[i].buf, NAND_PAGE_SIZE);
        if (meta)
            memcpy(meta, &nand_fmiss_span.meta[i * NAND_META_WORDS], NAND_META_WORDS * 4);
        r[i].ecc = nand_stat_result(r[i].buf, meta,
                                    &nand_fmiss_span.stat[i * NAND_CHUNKS_PER_PAGE]);
    }
    return 0;
}

static int nand_physical_read(uint32_t bank, uint32_t page, uint8_t *buf, uint32_t *summary)
{
    uint32_t r6;
    int rc;
    nand_rd.row = page;
    nand_rd.cemask = 1u << bank;
    nand_rd.dma[0] = (uint32_t)(intptr_t)buf;
    nand_rd.dma[1] = (uint32_t)(intptr_t)buf + NAND_PAGE_SIZE / 2;
    commit_discard_dcache_range(&nand_rd, sizeof(nand_rd));
    discard_dcache_range(buf, NAND_PAGE_SIZE);

    if (lcd_wait_write_dma())
        return -3;
    /* Prepare the engine before parameter writes; run must preserve them. */
    if (n4g_fmss_reset())
        return -2;
    FMISS_REG(FMISS_P_ROWS) = (uint32_t)(intptr_t)&nand_rd.row;
    FMISS_REG(FMISS_P_CEMASKS) = (uint32_t)(intptr_t)&nand_rd.cemask;
    FMISS_REG(FMISS_P_COUNT) = 1;
    FMISS_REG(FMISS_P_META) = (uint32_t)(intptr_t)nand_rd.meta;
    FMISS_REG(FMISS_P_DMALIST) = (uint32_t)(intptr_t)nand_rd.dma;
    FMISS_REG(FMISS_P_ECCBUF) = (uint32_t)(intptr_t)nand_rd.stat;
    FMISS_REG(FMISS_P_STATS) = 0;
    FMISS_REG(0x0d2c) = NAND_KICK_DATA | NAND_KICK_BCH16;
    FMISS_REG(0x0d30) = NAND_KICK_META | NAND_KICK_BCH16;
    FMISS_REG(0x0d34) = 10 + nand_ecclen - 1;
    FMISS_REG(0x0d38) = nand_ecclen - 1;
    FMISS_REG(FMISS_P_TIMING) = NAND_READ_TIMING;
    rc = nand_fmiss_run(fmss_prog_read, sizeof(fmss_prog_read));
    r6 = FMISS_REG(0x0c30);
    if (!rc && (r6 & FMISS_R6_ABORTED))
        rc = -4;
    if (rc >= -1 || rc == -4)
        nand_fmiss_cleanup();
    nand_touch();
    if (rc)
        return rc;
    discard_dcache_range(&nand_rd, sizeof(nand_rd));
    discard_dcache_range(buf, NAND_PAGE_SIZE);

    *summary = r6;
    return 0;
}

static int nand_read_page_fmt(uint32_t bank, uint32_t page, void *databuf,
                   uint32_t *meta)
{
    uint8_t *buf = databuf;
    unsigned int i, blank = 0, bad = 0;
    uint32_t r6;
    int rc;

    if ((uintptr_t)buf & (CACHEALIGN_SIZE - 1))
        buf = nand_fmiss_page;
    rc = nand_physical_read(bank, page, buf, &r6);
    nand_touch();
    if (rc) return rc;

    /* An erased page reads as erased chunks, a few of which may decode as
     * uncorrectable from bits flipped in the blank area */
    for (i = 0; i < NAND_CHUNKS_PER_PAGE; i++)
    {
        if (nand_rd.stat[i] == 0xfe)
            blank++;
        else if (nand_rd.stat[i] == 0xff)
            bad++;
    }
    if (blank == NAND_CHUNKS_PER_PAGE
        || (bad < 3 && blank + bad == NAND_CHUNKS_PER_PAGE))
    {
        memset(buf, 0xff, NAND_PAGE_SIZE);
        memset(nand_rd.meta, 0xff, sizeof(nand_rd.meta));
        r6 = 0;
    }
    if (buf != databuf)
        memcpy(databuf, buf, NAND_PAGE_SIZE);
    if (meta)
        memcpy(meta, nand_rd.meta, sizeof(nand_rd.meta));
    if (r6 & FMISS_R6_UECC)
        return NAND_ECC_FAILED;
    return (r6 & FMISS_R6_CORRECTED) ? NAND_ECC_CORRECTED : NAND_ECC_CLEAN;
}

/* Erased pages cannot select the ECC strength. Retry an uncorrectable
 * programmed page with BCH16 and retain that format only after success. */
int nand_read_page(uint32_t bank, uint32_t page, void *databuf,
                   uint32_t *meta)
{
    int rc;

    if (!nand_page_ok(bank, page) || !databuf)
        return -1;
    rc = nand_read_page_fmt(bank, page, databuf, meta);

    if (nand_ecclen == 26 || rc != NAND_ECC_FAILED)
        return rc;
    nand_ecclen = 26;
    rc = nand_read_page_fmt(bank, page, databuf, meta);
    if (rc != NAND_ECC_CLEAN && rc != NAND_ECC_CORRECTED)
        nand_ecclen = 13;
    return rc;
}

/* fmss/boot_read.s: the BootROM boot-block format, 0x600 bytes per page in
 * three BCH16 chunks with 26 parity bytes each. 0x0d00..0x0d08 return each
 * chunk's uncorrectable (bit 0) and erased (bit 6) flags. Bank 0 page 0
 * holds the IMG1 header. */
int nand_read_boot_page(uint32_t bank, uint32_t page, void *databuf)
{
    uint8_t *buf = databuf;
    int rc, i, c;

    if (!nand_page_ok(bank, page) || !databuf)
        return -1;
    if ((uintptr_t)buf & (CACHEALIGN_SIZE - 1))
        buf = nand_fmiss_page;
    uint32_t status[3];
    {
        discard_dcache_range(buf, FMISS_WRITE_BYTES);
        if (lcd_wait_write_dma())
            return -3;
        /* Prepare the engine before parameter writes; run must preserve them. */
        if (n4g_fmss_reset())
            return -2;
        FMISS_REG(FMISS_P_ROWS) = page;
        FMISS_REG(FMISS_P_CEMASKS) = 1u << bank;
        FMISS_REG(FMISS_P_DMALIST) = (uint32_t)(intptr_t)buf;
        rc = nand_fmiss_run(fmss_prog_boot_read, sizeof(fmss_prog_boot_read));
        if (!rc && (FMISS_REG(0x0c30) & FMISS_R6_ABORTED))
            rc = -4;
        if (rc >= -1 || rc == -4)
            nand_fmiss_cleanup();
        if (rc)
            return rc;
        discard_dcache_range(buf, FMISS_WRITE_BYTES);
        for (c = 0; c < 3; c++) status[c] = FMISS_REG(0x0d00 + 4 * c);
    }
    if (buf != databuf)
        memcpy(databuf, buf, FMISS_WRITE_BYTES);
    for (c = 0; c < 3; c++)
    {
        uint32_t st = status[c];

        if (!(st & 0x40))
        {
            if (st & 1)
                return NAND_ECC_FAILED;
            continue;
        }
        /* an erased chunk must read all 0xff */
        for (i = 0; i < 512; i++)
            if (buf[c * 512 + i] != 0xff)
                return NAND_ECC_FAILED;
    }
    return NAND_ECC_CLEAN;
}

void nand_set_writable(bool on)
{
    (void)on; /* Compatibility API: emCORE has no write gate. */
}

/* emCORE rejects only addresses outside the detected chip geometry. */
static bool nand_write_allowed(uint32_t bank, uint32_t block)
{
    return nand_fmiss_target_ok(bank, block);
}



int nand_read_pages(struct nand_read *r, unsigned int n)
{
    unsigned int i;

    if (!nand_ready || !n)
        return -1;
    for (i = 0; i < n; i++)
        if (!r[i].buf || !nand_page_ok(r[i].bank, r[i].page))
            return -1;
        for (i = 0; i < n; )
        {
            unsigned int cnt = 0, j;
            int rc;

            while (i + cnt < n && cnt < FMSS_READ_SPAN
                   && !((uintptr_t)r[i + cnt].buf & (CACHEALIGN_SIZE - 1)))
                cnt++;
            if (!cnt)
            {
                int ecc = nand_read_page(r[i].bank, r[i].page, r[i].buf, r[i].meta);
                if (ecc < 0)
                    return -1;
                r[i].ecc = ecc;
                i++;
                continue;
            }
            rc = nand_fmiss_read_span(r + i, cnt);
            if (rc)
                return -1;
            for (j = 0; j < cnt; j++)
                if (r[i + j].ecc == NAND_ECC_FAILED && nand_ecclen != 26)
                {
                    int ecc = nand_read_page(r[i + j].bank, r[i + j].page,
                                             r[i + j].buf, r[i + j].meta);
                    if (ecc < 0)
                        return -1;
                    r[i + j].ecc = ecc;
                }
            i += cnt;
        }
        return 0;
}

int nand_erase_block(uint32_t bank, uint32_t block)
{
    int rc;

    if (!nand_write_allowed(bank, block))
        return -1;
    rc = nand_fmiss_erase(bank, block);
    nand_touch();
    if (rc)
        logf("nand: erase bank %lu block %lu: %d", (unsigned long)bank,
             (unsigned long)block, rc);
    return rc == -10 ? NAND_OP_FAILED : rc ? -1 : 0;
}

/* two_plane is accepted and ignored. Program at most one page per bank in
 * each FMISS run, preserving the order of pages sent to each bank. */
int nand_write_pages(const struct nand_write *w, unsigned int n,
                     bool two_plane, uint32_t *failbank)
{
    struct nand_write batch[NAND_FMISS_PROGRAM_BATCH];
    unsigned int i, bank, done = 0;
    unsigned int next[NAND_MAX_BANKS];
    int rc;

    (void)two_plane;
    if (!n)
        return -1;
    for (i = 0; i < n; i++)
    {
        if (!w[i].buf || !nand_chip
            || !nand_write_allowed(w[i].bank,
                                   w[i].page / nand_chip->pagesperblock))
        {
            if (failbank)
                *failbank = w[i].bank;
            return -1;
        }
    }
    for (bank = 0; bank < NAND_MAX_BANKS; bank++)
    {
        next[bank] = n;
        for (i = 0; i < n; i++)
            if (w[i].bank == bank)
            {
                next[bank] = i;
                break;
            }
    }
    while (done < n)
    {
        unsigned int count = 0;

        for (bank = 0; bank < nand_banks; bank++)
            if (next[bank] < n)
            {
                i = next[bank];
                batch[count++] = w[i];
                next[bank] = n;
                for (i++; i < n; i++)
                    if (w[i].bank == bank)
                    {
                        next[bank] = i;
                        break;
                    }
            }
        if (!count)
            return -1;
        rc = nand_fmiss_program(batch, count, failbank);
        if (rc)
        {
            uint32_t failed_bank = failbank ? *failbank : batch[0].bank;

            for (i = 0; i < count; i++)
                if (batch[i].bank == failed_bank)
                    break;
            if (i == count)
                i = 0;
            logf("nand: program bank %lu page %lu: %d",
                 (unsigned long)failed_bank,
                 (unsigned long)batch[i].page, rc);
            return rc == -10 ? NAND_OP_FAILED : -1;
        }
        done += count;
    }
    nand_touch();
    return 0;
}

int nand_write_page(uint32_t bank, uint32_t page, const void *databuf,
                    const uint32_t *meta)
{
    struct nand_write w = { bank, page, databuf, meta };

    return nand_write_pages(&w, 1, false, NULL);
}

unsigned int nand_get_bank_count(void)
{
    return nand_banks;
}

uint32_t nand_chip_id(void)
{
    return nand_id;
}

static const uintptr_t nand_public_api[] = {
    (uintptr_t)nand_read_pages,
    (uintptr_t)nand_read_raw,
    (uintptr_t)nand_get_bank_count,
    (uintptr_t)nand_set_writable,
};

static void nand_keep_public_api(void)
{
    asm volatile("" : : "r"(nand_public_api) : "memory");
}

int nand_device_init(void)
{
    uint32_t bank, id;
    int rc;

    nand_keep_public_api();
    nand_ready = false;
    nand_banks = 0;
    nand_chip = NULL;
    nand_id = 0;
    nand_ecclen = 13;

    /* NAND pinmux, as the BootROM's NAND boot path (0x20001b70) sets it */
    PCON6 = (PCON6 & 0x0000000f) | 0x22222220;
    PCON7 = 0x22222222;
    PCON(8) = (PCON(8) & 0xff0000f0) | 0x00333302;

    rc = nand_reset(0);
    if (rc)
    {
        logf("nand: reset failed (%d)", rc);
        return rc;
    }
    if (nand_identify(&id))
    {
        logf("nand: identify failed");
        return NAND_ERR_IDENTIFY;
    }
    /* Keep the ID even for an unknown chip, so the storage error on screen
     * can show which chip it is. */
    nand_id = id;
    if (!nand_decode(id, &nand_chip_decoded))
        nand_chip = &nand_chip_decoded;

    for (bank = 1; bank < NAND_MAX_BANKS; bank++)
        if (nand_reset(bank) || nand_identify(&id) || id != nand_id)
            break;
    nand_banks = bank;
    if (!nand_chip)
    {
        logf("nand: unknown chip %08lx x %u", (unsigned long)nand_id,
             nand_banks);
        return NAND_ERR_UNKNOWN;
    }
    /* Apple's populations for this model: two or four chips of one part on
     * the first chip enables, the others empty */
    for (; bank < NAND_MAX_BANKS; bank++)
        if (!nand_reset(bank) && !nand_identify(&id)
            && id != 0 && id != 0xffffffff)
            return NAND_ERR_UNKNOWN;
    if (nand_banks != 2 && nand_banks != NAND_MAX_BANKS)
        return NAND_ERR_UNKNOWN;

    nand_ready = true;
    nand_fill_geo();
    return 0;
}

/* emCORE diagnostic accessors; do not infer physical ECC strength from an erased page. */
unsigned int nand_get_ecc_bits(void) { return nand_ecclen == 26 ? 16 : 8; }
void nand_copy_sector_status(void *out) { memcpy(out, nand_rd.stat, sizeof(nand_rd.stat)); }
/* emCORE CPU-direct NAND experiments (cpu_nand.c) use the same target check. */
