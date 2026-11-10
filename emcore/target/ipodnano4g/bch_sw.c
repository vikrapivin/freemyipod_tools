/* SPDX-License-Identifier: GPL-2.0-or-later
 * Software BCH encoder/decoder for the S5L8720 NAND formats (see bch_sw.h).
 *
 * Encode: byte-wise LFSR with a 256-entry remainder table per code.
 * Decode: remainder of the received word (re-encode XOR received parity);
 * zero = clean. Otherwise syndromes S_1..S_2t of that remainder,
 * Berlekamp-Massey for the error locator, and a Chien search over the
 * shortened code's bit positions. The word is changed only when the number
 * of roots found equals the locator degree (<= t).
 */
#include "global.h"
#include "malloc.h"
#include "bch_sw.h"

#define GF_POLY 0x201bu
#define GF_N    8191u

/* Generators without their leading x^deg term, most significant byte first. */
static const uint8_t g8[13] = {
    0x15, 0xf9, 0x14, 0xe0, 0x7b, 0x0c, 0x13, 0x87, 0x41, 0xc5, 0xc4, 0xfb, 0x23
};
static const uint8_t g16[26] = {
    0xcb, 0xbe, 0x3f, 0x0d, 0xbe, 0xc5, 0x63, 0xb5, 0xfb, 0x20, 0xff, 0x07, 0xf7,
    0xaa, 0x45, 0xff, 0x02, 0x6f, 0xb3, 0x78, 0xa6, 0x01, 0xcd, 0xd0, 0xfd, 0xd1
};

static uint16_t *gf_exp;    /* 2 * GF_N entries */
static uint16_t *gf_log;    /* 8192 entries */
static uint8_t *enc8;       /* 256 x 13 */
static uint8_t *enc16;      /* 256 x 26 */

static uint16_t gmul(uint16_t a, uint16_t b)
{
    return a && b ? gf_exp[gf_log[a] + gf_log[b]] : 0;
}

static void table(uint8_t *t, const uint8_t *g, unsigned n)
{
    unsigned i, bit, k;
    for (i = 0; i < 256; i++) {
        uint8_t *r = t + i * n;
        memset(r, 0, n);
        for (bit = 8; bit--;) {
            unsigned fb = ((i >> bit) & 1) ^ (r[0] >> 7);
            for (k = 0; k < n; k++) r[k] = (uint8_t)(r[k] << 1 | (k + 1 < n ? r[k + 1] >> 7 : 0));
            if (fb) for (k = 0; k < n; k++) r[k] ^= g[k];
        }
    }
}

int bch_sw_init(void)
{
    uint32_t i, x = 1;
    uint8_t *mem;
    if (gf_exp) return 0;
    mem = memalign(4, 2 * GF_N * 2 + 8192 * 2 + 256 * (13 + 26));
    if (!mem) return BCH_SW_EARG;
    gf_exp = (uint16_t *)mem;
    gf_log = gf_exp + 2 * GF_N;
    enc8 = (uint8_t *)(gf_log + 8192);
    enc16 = enc8 + 256 * 13;
    gf_log[0] = 0;
    for (i = 0; i < GF_N; i++) {
        gf_exp[i] = gf_exp[i + GF_N] = (uint16_t)x;
        gf_log[x] = (uint16_t)i;
        x <<= 1;
        if (x & 0x2000) x ^= GF_POLY;
    }
    table(enc8, g8, 13);
    table(enc16, g16, 26);
    return 0;
}

unsigned bch_sw_parity_bytes(unsigned t) { return t == 8 ? 13 : t == 16 ? 26 : 0; }

int bch_sw_encode(unsigned t, const uint8_t *msg, unsigned len, uint8_t *par)
{
    unsigned n = bch_sw_parity_bytes(t), i, k;
    const uint8_t *tab = t == 8 ? enc8 : enc16;
    if (!n || !gf_exp || len > BCH_SW_MAX_MSG) return BCH_SW_EARG;
    memset(par, 0, n);
    for (i = 0; i < len; i++) {
        const uint8_t *row = tab + (unsigned)(par[0] ^ msg[i]) * n;
        for (k = 0; k + 1 < n; k++) par[k] = par[k + 1] ^ row[k];
        par[n - 1] = row[n - 1];
    }
    return 0;
}

int bch_sw_decode(unsigned t, uint8_t *msg, unsigned len, uint8_t *par, uint16_t *loc)
{
    unsigned n = bch_sw_parity_bytes(t), i, j, k, nbits, total, L = 0, m = 1, nroots = 0;
    uint8_t rem[26];
    uint16_t S[2 * BCH_SW_MAX_T], C[2 * BCH_SW_MAX_T + 1], B[2 * BCH_SW_MAX_T + 1], T[2 * BCH_SW_MAX_T + 1];
    uint16_t b = 1, e[BCH_SW_MAX_T + 1];
    uint32_t root[BCH_SW_MAX_T];
    int any = 0;

    if (bch_sw_encode(t, msg, len, rem)) return BCH_SW_EARG;
    for (i = 0; i < n; i++) any |= rem[i] ^= par[i];
    if (!any) return 0;

    /* Syndromes of the remainder r(x): S_j = r(alpha^j); bit k of r is byte n-1-k/8, bit k%8. */
    memset(S, 0, sizeof(S));
    for (k = 0; k < 8 * n; k++)
        if (rem[n - 1 - k / 8] >> (k % 8) & 1)
            for (j = 1; j <= 2 * t; j++) S[j - 1] ^= gf_exp[(j * k) % GF_N];

    /* Berlekamp-Massey */
    memset(C, 0, sizeof(C));
    memset(B, 0, sizeof(B));
    C[0] = B[0] = 1;
    for (i = 0; i < 2 * t; i++) {
        uint16_t d = S[i], coef;
        for (j = 1; j <= L; j++) d ^= gmul(C[j], S[i - j]);
        if (!d) { m++; continue; }
        coef = gf_exp[gf_log[d] + GF_N - gf_log[b]];
        memcpy(T, C, sizeof(C));
        for (j = 0; j + m <= 2 * t; j++) C[j + m] ^= gmul(coef, B[j]);
        if (2 * L <= i) {
            L = i + 1 - L;
            memcpy(B, T, sizeof(B));
            b = d;
            m = 1;
        } else m++;
    }
    if (L > t) return BCH_SW_UNCORRECTABLE;
    for (j = L + 1; j <= 2 * t; j++) if (C[j]) return BCH_SW_UNCORRECTABLE;

    /* Chien search: error at degree k iff Lambda(alpha^-k) = 0, k over the shortened word. */
    total = len + n;
    nbits = 8 * total;
    for (j = 1; j <= L; j++) e[j] = C[j] ? gf_log[C[j]] : 0;
    for (k = 0; k < nbits && nroots <= L; k++) {
        uint16_t v = 1;
        for (j = 1; j <= L; j++) {
            if (!C[j]) continue;
            v ^= gf_exp[e[j]];
            e[j] = e[j] >= j ? e[j] - j : e[j] + GF_N - j;    /* next k: times alpha^-j */
        }
        if (!v) {
            if (nroots == L) { nroots++; break; }
            root[nroots++] = k;
        }
    }
    if (nroots != L) return BCH_SW_UNCORRECTABLE;

    for (i = 0; i < L; i++) {
        uint32_t byte = total - 1 - root[i] / 8, l = byte * 8 + root[i] % 8;
        if (byte < len) msg[byte] ^= 1u << (root[i] % 8);
        else par[byte - len] ^= 1u << (root[i] % 8);
        if (loc) {
            for (j = i; j > 0 && loc[j - 1] > l; j--) loc[j] = loc[j - 1];
            loc[j] = (uint16_t)l;
        }
    }
    return (int)L;
}
