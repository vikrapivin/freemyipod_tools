/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef N4G_FMSS_CONTROLLER_H
#define N4G_FMSS_CONTROLLER_H
#include "global.h"
/* Reset, then populate parameter RAM before calling run. */
int n4g_fmss_run(const void *program, uint32_t len);
struct n4g_fmss_reset_state {
    uint32_t phase, control, status, bufrst, bufok, bufcfg, fmc_control, fmc_status;
};
int n4g_fmss_reset(void);
int n4g_fmss_reset_traced(struct n4g_fmss_reset_state *state);
int n4g_fmss_start(uint32_t control);
#endif
