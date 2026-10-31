//
//
//    Copyright 2010 TheSeven
//
//
//    This file is part of emCORE.
//
//    emCORE is free software: you can redistribute it and/or
//    modify it under the terms of the GNU General Public License as
//    published by the Free Software Foundation, either version 2 of the
//    License, or (at your option) any later version.
//
//    emCORE is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with emCORE.  If not, see <http://www.gnu.org/licenses/>.
//
//


#include "global.h"
#include "backlight.h"
#include "pmu.h"


/* nano 3G: the backlight is the D1671 PMU's LED control register (I2C bus 0, device 0xe6), as in Rockbox's
   backlight-nano3g.c: bit 7 enable, bit 6 set by the original firmware (meaning unknown), bit 5 cleared when
   turning on, bits 4..0 the level. The classic's registers 0x28/0x29/0x2b do not exist on this PMU. */
#define LEDCTL          0x20
#define LEDCTL_ENABLE   0x80
#define LEDCTL_UNKNOWN  0x40
#define LEDCTL_LEVEL    0x1f
#define LEVEL_DEFAULT   0x10    /* Rockbox's default setting 0x20 >> 1 */

static void backlight_set_level(unsigned int level)
{
    if (level > LEDCTL_LEVEL) level = LEDCTL_LEVEL;
    pmu_write(LEDCTL, (pmu_read(LEDCTL) & ~LEDCTL_LEVEL) | level);
}

void backlight_init()
{
    backlight_set_level(LEVEL_DEFAULT);
    backlight_on(true);
}

void backlight_on(bool on)
{
    unsigned char v = pmu_read(LEDCTL);
    if (on) pmu_write(LEDCTL, (v & ~0x20) | LEDCTL_ENABLE | LEDCTL_UNKNOWN);
    else pmu_write(LEDCTL, v & ~LEDCTL_ENABLE);
}

void backlight_set_fade(uint8_t fade)
{
    (void)fade;     /* no fade control known on the D1671 */
}

void backlight_set_brightness(uint8_t brightness)
{
    /* emCORE scale 0..255 -> PMU level 0..31; never fully dark while on */
    unsigned int level = (brightness * LEDCTL_LEVEL + 127) / 255;
    backlight_set_level(level ? level : 1);
}
