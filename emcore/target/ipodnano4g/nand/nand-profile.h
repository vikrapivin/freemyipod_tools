/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef N4G_NAND_PROFILE_H
#define N4G_NAND_PROFILE_H
#include "global.h"
/* Exclusive device time: nested reset/interlock scopes are charged separately.
 * OTHER is the unclassified API remainder, including shared logical policy.
 * EXEC includes NAND, DMA and BCH together, not separately measured engines. */
enum nand_profile_phase {
    NAND_PROFILE_OTHER, NAND_PROFILE_RESET, NAND_PROFILE_INTERLOCK,
    NAND_PROFILE_SETUP, NAND_PROFILE_EXEC, NAND_PROFILE_CLEANUP,
    NAND_PROFILE_PHASES
};
void nand_profile_begin(bool enabled);
unsigned int nand_profile_enter(unsigned int phase);
void nand_profile_leave(unsigned int previous);
void nand_profile_end(uint32_t *times);
#endif
