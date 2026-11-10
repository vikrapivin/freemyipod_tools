/* SPDX-License-Identifier: GPL-2.0-or-later
 * Software BCH for the S5L8720 NAND formats, no ECC engine involved.
 *
 * Both codes are narrow-sense binary BCH over GF(2^13), x^13+x^4+x^3+x+1:
 *   t = 8:  generator degree 104 (roots alpha^1..alpha^16), 13 parity bytes
 *   t = 16: generator degree 208 (roots alpha^1..alpha^32), 26 parity bytes
 * Message = the bytes in order (512 data bytes, then the covered metadata),
 * most significant bit first; parity = message * x^deg mod g, most
 * significant byte first. This is the stream the ECC engine writes.
 *
 * Decode corrects up to t bit errors anywhere in message and parity (in
 * place) and reports the corrected locations in the engine's numbering
 * (0x820..): codeword byte b (message then parity), bit i (0 = LSB) is
 * b * 8 + i, ascending. More than t errors: nothing is changed.
 */
#ifndef N4G_BCH_SW_H
#define N4G_BCH_SW_H
#include "global.h"

#define BCH_SW_MAX_T        16u
#define BCH_SW_MAX_MSG      544u        /* message bytes per code word */
#define BCH_SW_UNCORRECTABLE (-1)
#define BCH_SW_EARG         (-2)        /* t not 8/16, length out of range, tables not allocated */

/* Allocates and fills the field and encoder tables once (about 58 KiB). 0 or BCH_SW_EARG. */
int bch_sw_init(void);
unsigned bch_sw_parity_bytes(unsigned t);
/* par = parity of msg[0..len) (13 or 26 bytes). 0 or BCH_SW_EARG. */
int bch_sw_encode(unsigned t, const uint8_t *msg, unsigned len, uint8_t *par);
/* Returns the number of corrected bits (0..t), BCH_SW_UNCORRECTABLE or BCH_SW_EARG.
 * loc (may be NULL) receives up to t locations, ascending. */
int bch_sw_decode(unsigned t, uint8_t *msg, unsigned len, uint8_t *par, uint16_t *loc);
#endif
