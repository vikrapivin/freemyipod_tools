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

/* Backend registry and the nand-target.h dispatch functions. */
#include "global.h"
#include "nand-backend.h"

static const struct nand_backend *const nand_backends[NAND_NBACKENDS] =
{
    &nand_backend_sw,
};

static unsigned int nand_active = NAND_BACKEND_SW;
static bool nand_active_ready;

const struct nand_backend *nand_backend_get(unsigned int idx)
{
    return idx < NAND_NBACKENDS ? nand_backends[idx] : NULL;
}

const struct nand_backend *nand_backend_active(void)
{
    return nand_backends[nand_active];
}

unsigned int nand_backend_active_index(void)
{
    return nand_active;
}

int nand_backend_select(unsigned int idx)
{
    if (idx >= NAND_NBACKENDS)
        return NAND_ERR_NOINIT;
    nand_active = idx;
    nand_active_ready = false;
    return 0;
}

int nand_backend_init(void)
{
    const struct nand_backend *b = nand_backend_active();
    int rc = b->init ? b->init() : NAND_ERR_NOSYS;
    nand_active_ready = rc == 0;
    return rc;
}

bool nand_backend_ready(void)
{
    return nand_active_ready;
}

int nand_reset(uint32_t bank)
{
    const struct nand_backend *b = nand_backend_active();
    return b->reset ? b->reset(bank) : NAND_ERR_NOSYS;
}

int nand_read_pages(struct nand_read *r, unsigned int n)
{
    const struct nand_backend *b = nand_backend_active();
    if (!nand_active_ready || !b->read_pages)
        return NAND_ERR_NOSYS;
    return b->read_pages(r, n);
}

int nand_read_page(uint32_t bank, uint32_t page, void *databuf, uint32_t *meta)
{
    struct nand_read r = { bank, page, databuf, meta, 0, 0 };
    if (nand_read_pages(&r, 1))
        return -1;
    return r.ecc;
}

int nand_write_pages(const struct nand_write *w, unsigned int n,
                     bool two_plane, uint32_t *failbank)
{
    const struct nand_backend *b = nand_backend_active();
    if (!nand_active_ready || !b->write_pages)
        return NAND_ERR_NOSYS;
    return b->write_pages(w, n, two_plane, failbank);
}

int nand_write_page(uint32_t bank, uint32_t page, const void *databuf,
                    const uint32_t *meta)
{
    struct nand_write w = { bank, page, databuf, meta };
    return nand_write_pages(&w, 1, false, NULL);
}

int nand_erase_blocks(const struct nand_erase *e, unsigned int n,
                      unsigned int *failidx)
{
    const struct nand_backend *b = nand_backend_active();
    if (!nand_active_ready || !b->erase_blocks)
        return NAND_ERR_NOSYS;
    return b->erase_blocks(e, n, failidx);
}

int nand_erase_block(uint32_t bank, uint32_t block)
{
    struct nand_erase e = { bank, block };
    return nand_erase_blocks(&e, 1, NULL);
}

const struct nand_geometry *nand_get_geometry(void)
{
    const struct nand_backend *b = nand_backend_active();
    if (!nand_active_ready || !b->get_geometry)
        return NULL;
    return b->get_geometry();
}

uint32_t nand_get_id(void)
{
    const struct nand_backend *b = nand_backend_active();
    return nand_active_ready && b->get_id ? b->get_id() : 0;
}

unsigned int nand_get_bank_count(void)
{
    const struct nand_geometry *g = nand_get_geometry();
    return g ? g->banks : 0;
}
