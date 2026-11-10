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


#ifndef __TARGET_H__
#define __TARGET_H__


#define PLATFORM_ID 0x47344e49


#define ARM_ARCH 6
#define LITTLE_ENDIAN
#define CACHEALIGN_BITS 5
#define CPU_FREQ 532000000


#define CONSOLE_BOOT 3
#define CONSOLE_PANIC 3
#define CONSOLE_PANICDUMP 0


#include "../ipodnano4g/s5l8720.h"
#define HAVE_USB
/* Experimental monitor: NAND is accessed only through explicit worker jobs. */
#define NAND_POC_NOSTORAGE
/* NAND harness: drop FAT/file/dir, app loader, boot-from-file and libc extras */
#ifdef N4G_SLIM   /* opt-in since round 9 (make EMCORE_SLIM=1): every SLIM image rebooted on the device, every pre-SLIM image booted */
#define EMCORE_SLIM
#endif
/* defensive device init for the DFU/bootstub path (target/ipodnano4g/targetinit.c, A) */
#ifdef N4G_TARGETINIT /* opt-in (make N4G_TARGETINIT=1): defensive init of; OFF by default since round 8 */
#define HAVE_TARGETINIT_VERYEARLY
#define HAVE_TARGETINIT_EARLY
#define HAVE_TARGETINIT_LATE
#endif
#define USB_DRIVER_HEADER "usb/synopsysotg.h"
#define USB_DRIVER synopsysotg_driver
#define USB_DRIVER_CONFIG_TYPE const struct synopsysotg_config
#define USB_DRIVER_CONFIG \
{ \
    .core = (struct synopsysotg_core_regs*)OTGBASE, \
    .phy_16bit = true, \
    .phy_ulpi = false, \
    .use_dma = true, \
    .shared_txfifo = false, \
    .disable_double_buffering = false, \
    .fifosize = 1024, \
    .txfifosize = { 0x40, 0x100, 0, 0x100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, \
}
#define USB_DRIVER_STATE_TYPE struct synopsysotg_state
#define USB_DRIVER_STATE \
{ \
    .endpoints = { {}, {}, {}, {}, {} }, \
}
#define USB_ENDPOINTS 0b0000000000010110000000000010101
#define USB_MAXCURRENT 500
//#define USB_HAVE_TARGET_SPECIFIC_REQUESTS


#define HAVE_LCD
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_FORMAT 0x004154b4  // rgb565
#define LCD_BYTESPERPIXEL 2
#define LCDCONSOLE_FGCOLOR 0
#define LCDCONSOLE_BGCOLOR -1

#define HAVE_BACKLIGHT

#define HAVE_I2C

#define HAVE_HWKEYAES

/* Physical experiment monitor: no mounted storage/FTL. */

#define HAVE_ACCELEROMETER



#endif
