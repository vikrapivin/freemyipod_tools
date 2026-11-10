/* Nano 4G physical NAND under the nano 5G VFL/FTL reader.
 * Pages are 4096 bytes. Writes stay off: mount loads the context and does
 * not program or erase. */
#include "global.h"
#include "target/ipodnano5g/nand/nand-target.h"

void n4g_ftl_chip(unsigned int *banks, unsigned int *blocks, unsigned int *ppb,
                  uint32_t *chipid);

static struct nand_geometry ftl_geom;
static int ftl_geom_ready;

static int mapped(int rc, const uint8_t *data, const uint8_t *meta)
{
    unsigned int i;
    if (rc == 0 || rc == 1)
    {
        if (meta)
        {
            for (i = 0; i < NAND_META_SIZE; i++)
                if (meta[i] != 0xff)
                    return NAND_OK;
            return NAND_ERASED;
        }
        if (data)
        {
            for (i = 0; i < 4096; i++)
                if (data[i] != 0xff)
                    return NAND_OK;
            return NAND_ERASED;
        }
        return NAND_OK;
    }
    if (rc == 3 || rc == -10)
        return NAND_ERR_ECC;
    if (rc == -2 || rc == -3 || rc == -4)
        return NAND_ERR_TIMEOUT;
    return rc < 0 ? NAND_ERR_ECC : rc;
}

const struct nand_geometry *ftl_phys_get_geometry(void)
{
    unsigned int banks, blocks, ppb, i;
    uint32_t id;

    if (ftl_geom_ready)
        return &ftl_geom;
    n4g_ftl_chip(&banks, &blocks, &ppb, &id);
    if (!banks)
        return NULL;
    memset(&ftl_geom, 0, sizeof(ftl_geom));
    ftl_geom.id = id;
    ftl_geom.banks = banks;
    for (i = 0; i < banks && i < NAND_MAX_BANKS; i++)
        ftl_geom.ce[i] = i;
    ftl_geom.blocks = blocks;
    ftl_geom.pages_per_block = ppb;
    ftl_geom.page_size = 4096;
    ftl_geom.spare_size = 128;
    ftl_geom.ecc_bits = 8;
    ftl_geom.vendor = (id & 0xff) == 0xad ? NAND_VENDOR_HYNIX : 0;
    /* On-flash context wins. This is the nano 4G one-way fallback. */
    ftl_geom.layout = NAND_LAYOUT_1WAY;
    ftl_geom.dies = 1;
    ftl_geom.blocks_per_die = blocks;
    ftl_geom.die_stride = blocks;
    ftl_geom.block_span = blocks;
    ftl_geom.blocks_aligned = blocks;
    ftl_geom.page_stride = ppb;
    ftl_geom.refresh_threshold = 6;
    ftl_geom_ready = 1;
    return &ftl_geom;
}

uint32_t ftl_phys_block_row(unsigned int block)
{
    const struct nand_geometry *g = ftl_phys_get_geometry();
    if (!g)
        return 0;
    return g->page_stride * block;
}

_Bool ftl_phys_write_supported(void)
{
    return false;
}

int ftl_phys_read_page(unsigned int bank, uint32_t page, void *data, void *meta)
{
    uint32_t words[3];
    int rc;

    if (!data)
        return NAND_ERR_ECC;
    rc = nand_read_page(bank, page, data, words);
    if (meta && (rc == 0 || rc == 1))
        memcpy(meta, words, NAND_META_SIZE);
    return mapped(rc, data, (rc == 0 || rc == 1) ? (const uint8_t *)words : NULL);
}

int ftl_phys_read_pages(unsigned int n, const uint8_t *bank, const uint32_t *page,
                        void *data, void *meta)
{
    unsigned int i;
    if (!n || n > NAND_MAX_PAGES_PER_READ || !bank || !page || !data)
        return NAND_ERR_ECC;
    for (i = 0; i < n; i++)
    {
        int rc = ftl_phys_read_page(bank[i], page[i],
                                    (uint8_t *)data + i * NAND_PAGE_SIZE,
                                    meta ? (uint8_t *)meta + i * NAND_META_SIZE : NULL);
        if (rc != NAND_OK && rc != NAND_ERASED)
            return rc < 0 ? rc : NAND_ERR_ECC;
    }
    return NAND_OK;
}
