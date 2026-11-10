/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef NAND_HAL_EMCORE_H
#define NAND_HAL_EMCORE_H
#include "global.h"
struct n4g_probe_hal {
    void (*cache_wb)(void *addr, uint32_t len);
    void (*cache_wb_inv)(void *addr, uint32_t len);
};
/* Set by the common crt0: 0 after the DFU stub, 1 after EXECFIRMWARE. */
extern uint32_t n4g_reload_boot;
extern const struct n4g_probe_hal g_hal;
void n4g_emcore_cache_inv(void *addr, uint32_t len);
#endif
