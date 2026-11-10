/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef N4G_NAND_COMPARE_H
#define N4G_NAND_COMPARE_H
#include "global.h"
/* Command 143: CE, first of two blocks, pages | repetitions<<16 | BCH<<24 | profile<<31.
 * Result: 16-word header followed by 24-word records, ABI 1, or ABI 2 with exclusive phase times and CPU baseline controls. */
int nand_compare(uint8_t *out, uint32_t capacity, const uint32_t *args, uint32_t *length);
#endif
