#include "global.h"
#include "boot_beacon.h"
#include "console.h"
#ifdef HAVE_LCD
#include "lcdconsole.h"
#endif
#ifndef BOOT_HALT_AT
#define BOOT_HALT_AT 0
#endif
static int lcd_ready;   /* set once the LCD console is initialised (milestone BCN_LCDCONSOLE) */

void boot_beacon(uint32_t id, uint32_t aux)
{
    volatile uint32_t *b = (volatile uint32_t *)BOOT_BEACON_BASE;
    if (b[0] != BOOT_BEACON_MAGIC) { b[0] = BOOT_BEACON_MAGIC; b[1] = 0; b[2] = 0; b[3] = 0; }
    b[1] = id; b[3] = aux;
    b[4 + (b[2] % 12u)] = id;
    b[2]++;
    if (id == BCN_LCDCONSOLE) lcd_ready = 1;
#ifdef HAVE_LCD
    if (lcd_ready) {
        char txt[24]; int n = 0; uint32_t v = id, d = 100;
        const char *p = "[boot "; while (*p) txt[n++] = *p++;
        while (d && !(v / d)) d /= 10;            /* no leading zeros */
        if (!d) txt[n++] = '0';
        for (; d; d /= 10) txt[n++] = '0' + (char)((v / d) % 10);
        txt[n++] = ']'; txt[n++] = (BOOT_HALT_AT && id == BOOT_HALT_AT) ? ' ' : '\n'; txt[n] = 0;
        lcdconsole_puts(txt, -1, -1);
        if (BOOT_HALT_AT && id == BOOT_HALT_AT) lcdconsole_puts("HALT\n", -1, -1);
    }
#endif
    if (BOOT_HALT_AT && id == BOOT_HALT_AT) for (;;) ;
}
