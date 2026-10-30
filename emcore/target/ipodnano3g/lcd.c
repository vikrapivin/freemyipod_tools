
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
#include "thread.h"
#include "s5l8702.h"
#include "util.h"
#include "clockgates-target.h"
#include "clockgates.h"
#include "lcd.h"
#include "mmu.h"
#include "lcdconsole.h"
#include "timer.h"


static struct dma_lli lcd_lli[(LCD_WIDTH * LCD_HEIGHT - 1) / 0xfff]
    IDATA_ATTR CACHEALIGN_ATTR;

static uint16_t lcd_color IDATA_ATTR;

static struct mutex lcd_mutex IDATA_ATTR;
static struct wakeup lcd_wakeup IDATA_ATTR;

static bool lcd_dma_busy IDATA_ATTR;


int lcd_get_width()
{
    return LCD_WIDTH;
}

int lcd_get_height()
{
    return LCD_HEIGHT;
}

int lcd_get_bytes_per_pixel()
{
    return LCD_BYTESPERPIXEL;
}

int lcd_get_format()
{
    return LCD_FORMAT;
}

/* Panel support follows Apple's iPod OS (osos 1.1.3): the panel types and their ID bytes (osos mem:0x08098764,
 * 6 tries 10 ms apart: mem:0x0809e700), the LCD controller setup (mem:0x080c95b8: frame mode 0x81100db9, +0x7c =
 * 0x402, +0x88 = 0x01000000, PHTIME 0x33), command mode as Apple derives it from the frame word (0x81000c21), the
 * per-window commands (mem:0x080d6510: 0x36 memory access control, 0x2a, 0x2b, 0x2c) and the per-panel register
 * values, delays, display-on order and GRAM clears of the init and sleep sequences in its panel tables (osos file
 * 0x9ef85c..0x9efb2d). Apple's OS only re-runs an init after a power-down; at boot its bootloader has done it, which
 * from DFU nobody has, so emCORE runs it here. Rockbox's lcd-nano3g.c (GPLv2+) was the first version of this driver
 * and agrees except: 38d5 e0[3], 58xx eb[3], 58xx sleep (0x28), 38c4 sleep delay, display-on order, GRAM clears. */
#define LCD_MODE_CMD   0x81000c21   /* commands and parameters, 8-bit */
#define LCD_MODE_FRAME 0x81100db9   /* RGB565 frame data on the 9-bit bus */
#define LCDREG(off)    (*((uint32_t volatile*)(0x38300000 + (off))))

enum { SEQ_CMD = 0, SEQ_SLEEP, SEQ_CLEAR, SEQ_END = 0xff };   /* SEQ_SLEEP unit: 1 ms; SEQ_CLEAR: GRAM = 0xffff */

enum
{
    LCD_TYPE_UNKNOWN = -1,
    LCD_TYPE_38B3 = 0,
    LCD_TYPE_38C4,
    LCD_TYPE_38D5,
    LCD_TYPE_38E6,
    LCD_TYPE_58XX,
};

/* Per-panel register values, from the iPod OS panel tables. 38e6 is 38b3 with c0 = 0x0a and c5[0] = 0x38, as the
 * OS patches it after reading that ID. */
/* 38b3 */
static const uint8_t lcd_init_seq_0[] INITCONST_ATTR =
{
    SEQ_CMD, 0xef, 1, 0x80,
    SEQ_CMD, 0xc0, 1, 0x06,
    SEQ_CMD, 0xc1, 1, 0x03,
    SEQ_CMD, 0xc2, 2, 0x12, 0x00,
    SEQ_CMD, 0xc3, 2, 0x12, 0x00,
    SEQ_CMD, 0xc4, 2, 0x12, 0x00,
    SEQ_CMD, 0xc5, 2, 0x40, 0x38,
    SEQ_CMD, 0xb1, 2, 0x5f, 0x3f,
    SEQ_CMD, 0xb2, 2, 0x5f, 0x3f,
    SEQ_CMD, 0xb3, 2, 0x5f, 0x3f,
    SEQ_CMD, 0xb4, 1, 0x02,
    SEQ_CMD, 0xb6, 2, 0x12, 0x02,
    SEQ_CMD, 0x35, 1, 0x00,
    SEQ_CMD, 0x26, 1, 0x10,
    SEQ_CMD, 0xfe, 1, 0x00,
    SEQ_CMD, 0xe0, 11, 0x0f, 0x70, 0x47, 0x03, 0x02, 0x02, 0xa0, 0x94, 0x05, 0x00, 0x0e,
    SEQ_CMD, 0xe1, 11, 0x02, 0x43, 0x77, 0x00, 0x0f, 0x05, 0x49, 0x0a, 0x02, 0x0e, 0x00,
    SEQ_CMD, 0xe2, 11, 0x2f, 0x63, 0x20, 0x50, 0x00, 0x07, 0xd1, 0x13, 0x00, 0x00, 0x0e,
    SEQ_CMD, 0xe3, 11, 0x50, 0x20, 0x60, 0x23, 0x0f, 0x00, 0x31, 0x1d, 0x07, 0x0e, 0x00,
    SEQ_CMD, 0xe4, 11, 0x5e, 0x50, 0x65, 0x27, 0x00, 0x0b, 0xdf, 0xf1, 0x01, 0x00, 0x0e,
    SEQ_CMD, 0xe5, 11, 0x20, 0x67, 0x55, 0x50, 0x0e, 0x01, 0x1f, 0xfd, 0x0b, 0x0e, 0x00,
    SEQ_CMD, 0x3a, 1, 0x06,
    SEQ_CMD, 0x13, 0,
    SEQ_CLEAR,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_CMD, 0x11, 0,
    SEQ_SLEEP, 120,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_END
};

/* 38c4 */
static const uint8_t lcd_init_seq_1[] INITCONST_ATTR =
{
    SEQ_CMD, 0x01, 0,
    SEQ_SLEEP, 5,
    SEQ_CMD, 0xc0, 1, 0x01,
    SEQ_CMD, 0xc1, 1, 0x03,
    SEQ_CMD, 0xc2, 2, 0x74, 0x00,
    SEQ_CMD, 0xc3, 2, 0x72, 0x03,
    SEQ_CMD, 0xc4, 2, 0x73, 0x03,
    SEQ_CMD, 0xc5, 2, 0x3c, 0x3c,
    SEQ_CMD, 0xb1, 2, 0x6a, 0x15,
    SEQ_CMD, 0xb2, 2, 0x6a, 0x15,
    SEQ_CMD, 0xb3, 2, 0x6a, 0x15,
    SEQ_CMD, 0xb4, 1, 0x02,
    SEQ_CMD, 0xb6, 2, 0x12, 0x02,
    SEQ_CMD, 0x35, 1, 0x00,
    SEQ_CMD, 0x26, 1, 0x10,
    SEQ_CMD, 0xe0, 11, 0x1e, 0x22, 0x44, 0x00, 0x09, 0x01, 0x47, 0xc1, 0x05, 0x02, 0x09,
    SEQ_CMD, 0xe1, 11, 0x0f, 0x32, 0x35, 0x00, 0x03, 0x05, 0x5e, 0x78, 0x03, 0x00, 0x03,
    SEQ_CMD, 0xe2, 11, 0x0d, 0x74, 0x47, 0x41, 0x07, 0x01, 0x74, 0x41, 0x09, 0x03, 0x07,
    SEQ_CMD, 0xe3, 11, 0x5f, 0x41, 0x27, 0x02, 0x00, 0x03, 0x43, 0x55, 0x02, 0x00, 0x03,
    SEQ_CMD, 0xe4, 11, 0x1b, 0x53, 0x44, 0x51, 0x0b, 0x01, 0x64, 0x20, 0x05, 0x02, 0x09,
    SEQ_CMD, 0xe5, 11, 0x7f, 0x41, 0x26, 0x02, 0x04, 0x00, 0x33, 0x35, 0x01, 0x00, 0x02,
    SEQ_CMD, 0x3a, 1, 0x66,
    SEQ_CLEAR,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_CMD, 0x11, 0,
    SEQ_SLEEP, 120,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_END
};

/* 38d5 */
static const uint8_t lcd_init_seq_2[] INITCONST_ATTR =
{
    SEQ_CMD, 0xfe, 1, 0x00,
    SEQ_CMD, 0xc0, 1, 0x00,
    SEQ_CMD, 0xc1, 1, 0x03,
    SEQ_CMD, 0xc2, 2, 0x73, 0x03,
    SEQ_CMD, 0xc3, 2, 0x73, 0x03,
    SEQ_CMD, 0xc4, 2, 0x73, 0x03,
    SEQ_CMD, 0xc5, 2, 0x64, 0x37,
    SEQ_CMD, 0xb1, 2, 0x69, 0x13,
    SEQ_CMD, 0xb2, 2, 0x69, 0x13,
    SEQ_CMD, 0xb3, 2, 0x69, 0x13,
    SEQ_CMD, 0xb4, 1, 0x02,
    SEQ_CMD, 0xb6, 2, 0x03, 0x12,
    SEQ_CMD, 0x35, 1, 0x00,
    SEQ_CMD, 0x26, 1, 0x10,
    SEQ_CMD, 0xe0, 11, 0x08, 0x00, 0x10, 0x20, 0x03, 0x0e, 0xc8, 0x65, 0x05, 0x00, 0x00,
    SEQ_CMD, 0xe1, 11, 0x06, 0x20, 0x00, 0x00, 0x00, 0x07, 0x4d, 0x0b, 0x08, 0x00, 0x00,
    SEQ_CMD, 0xe2, 11, 0x08, 0x77, 0x27, 0x63, 0x0f, 0x16, 0xcf, 0x25, 0x03, 0x00, 0x00,
    SEQ_CMD, 0xe3, 11, 0x5f, 0x53, 0x77, 0x06, 0x00, 0x02, 0x4b, 0x7b, 0x0f, 0x00, 0x00,
    SEQ_CMD, 0xe4, 11, 0x08, 0x46, 0x57, 0x52, 0x0f, 0x16, 0xcf, 0x25, 0x04, 0x00, 0x00,
    SEQ_CMD, 0xe5, 11, 0x6f, 0x42, 0x57, 0x06, 0x00, 0x04, 0x43, 0x7b, 0x0f, 0x00, 0x00,
    SEQ_CMD, 0x3a, 1, 0x66,
    SEQ_CLEAR,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_CMD, 0x11, 0,
    SEQ_SLEEP, 120,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_END
};

/* 38e6 */
static const uint8_t lcd_init_seq_3[] INITCONST_ATTR =
{
    SEQ_CMD, 0xef, 1, 0x80,
    SEQ_CMD, 0xc0, 1, 0x0a,
    SEQ_CMD, 0xc1, 1, 0x03,
    SEQ_CMD, 0xc2, 2, 0x12, 0x00,
    SEQ_CMD, 0xc3, 2, 0x12, 0x00,
    SEQ_CMD, 0xc4, 2, 0x12, 0x00,
    SEQ_CMD, 0xc5, 2, 0x38, 0x38,
    SEQ_CMD, 0xb1, 2, 0x5f, 0x3f,
    SEQ_CMD, 0xb2, 2, 0x5f, 0x3f,
    SEQ_CMD, 0xb3, 2, 0x5f, 0x3f,
    SEQ_CMD, 0xb4, 1, 0x02,
    SEQ_CMD, 0xb6, 2, 0x12, 0x02,
    SEQ_CMD, 0x35, 1, 0x00,
    SEQ_CMD, 0x26, 1, 0x10,
    SEQ_CMD, 0xfe, 1, 0x00,
    SEQ_CMD, 0xe0, 11, 0x0f, 0x70, 0x47, 0x03, 0x02, 0x02, 0xa0, 0x94, 0x05, 0x00, 0x0e,
    SEQ_CMD, 0xe1, 11, 0x02, 0x43, 0x77, 0x00, 0x0f, 0x05, 0x49, 0x0a, 0x02, 0x0e, 0x00,
    SEQ_CMD, 0xe2, 11, 0x2f, 0x63, 0x20, 0x50, 0x00, 0x07, 0xd1, 0x13, 0x00, 0x00, 0x0e,
    SEQ_CMD, 0xe3, 11, 0x50, 0x20, 0x60, 0x23, 0x0f, 0x00, 0x31, 0x1d, 0x07, 0x0e, 0x00,
    SEQ_CMD, 0xe4, 11, 0x5e, 0x50, 0x65, 0x27, 0x00, 0x0b, 0xdf, 0xf1, 0x01, 0x00, 0x0e,
    SEQ_CMD, 0xe5, 11, 0x20, 0x67, 0x55, 0x50, 0x0e, 0x01, 0x1f, 0xfd, 0x0b, 0x0e, 0x00,
    SEQ_CMD, 0x3a, 1, 0x06,
    SEQ_CMD, 0x13, 0,
    SEQ_CLEAR,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_CMD, 0x11, 0,
    SEQ_SLEEP, 120,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_END
};

/* 58xx */
static const uint8_t lcd_init_seq_4[] INITCONST_ATTR =
{
    SEQ_CMD, 0xe1, 3, 0x0f, 0x31, 0x04,
    SEQ_CMD, 0xe2, 5, 0x02, 0xa2, 0x08, 0x11, 0x01,
    SEQ_CMD, 0xe3, 2, 0x10, 0x88,
    SEQ_CMD, 0xe4, 2, 0x10, 0x88,
    SEQ_CMD, 0xe5, 2, 0x00, 0x88,
    SEQ_CMD, 0xe7, 13, 0x33, 0x0d, 0x57, 0x0e, 0x57, 0x05, 0x57, 0x02, 0x04, 0x10, 0x0b, 0x02, 0x02,
    SEQ_CMD, 0xe8, 5, 0x30, 0x0d, 0x84, 0x8c, 0x21,
    SEQ_CMD, 0xe9, 5, 0x4b, 0x1a, 0xba, 0x60, 0x11,
    SEQ_CMD, 0xea, 5, 0x4b, 0xba, 0xba, 0x10, 0x11,
    SEQ_CMD, 0xeb, 5, 0x0b, 0x3a, 0xd9, 0x10, 0x11,
    SEQ_CMD, 0xef, 3, 0x00, 0x00, 0x00,
    SEQ_CMD, 0xf0, 18, 0x12, 0x01, 0x21, 0xa5, 0x6c, 0x23, 0x02, 0x01, 0x08, 0x0d, 0x1e, 0xde, 0x5a, 0x93, 0xdc, 0x0d, 0x1e, 0x17,
    SEQ_CMD, 0xf1, 18, 0x12, 0x05, 0x21, 0xb5, 0x8d, 0x24, 0x02, 0x01, 0x08, 0x0d, 0x1a, 0xde, 0x4a, 0x72, 0xdb, 0x0d, 0x1e, 0x17,
    SEQ_CMD, 0xf2, 18, 0x0e, 0x00, 0x30, 0xd6, 0x8f, 0x34, 0x03, 0x07, 0x08, 0x11, 0x1f, 0xcf, 0x29, 0x70, 0xcb, 0x0c, 0x18, 0x17,
    SEQ_CMD, 0xfa, 3, 0x02, 0x00, 0x02,
    SEQ_CMD, 0xf7, 2, 0x00, 0xc0,
    SEQ_CMD, 0x35, 1, 0x00,
    SEQ_CLEAR,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_CMD, 0x11, 0,
    SEQ_SLEEP, 120,
    SEQ_CMD, 0x29, 0,
    SEQ_SLEEP, 1,
    SEQ_END
};

static const uint8_t* const lcd_init_seqs[] INITCONST_ATTR =
{
    lcd_init_seq_0, lcd_init_seq_1, lcd_init_seq_2, lcd_init_seq_3, lcd_init_seq_4
};

/* display off + sleep in, per type (iPod OS panel tables) */
static const uint8_t lcd_sleep_seq_03[] =
{
    SEQ_CLEAR,
    SEQ_CMD, 0x28, 0,
    SEQ_CMD, 0x10, 0,
    SEQ_SLEEP, 50,
    SEQ_END
};

static const uint8_t lcd_sleep_seq_1[] =
{
    SEQ_CLEAR,
    SEQ_CMD, 0x28, 0,
    SEQ_CMD, 0x10, 0,
    SEQ_SLEEP, 50,
    SEQ_END
};

static const uint8_t lcd_sleep_seq_2[] =
{
    SEQ_CLEAR,
    SEQ_CMD, 0x28, 0,
    SEQ_CMD, 0x10, 0,
    SEQ_END
};

static const uint8_t lcd_sleep_seq_4[] =
{
    SEQ_CLEAR,
    SEQ_CMD, 0x28, 0,
    SEQ_CMD, 0x10, 0,
    SEQ_END
};

static const uint8_t* const lcd_sleep_seqs[] =
{
    lcd_sleep_seq_03, lcd_sleep_seq_1, lcd_sleep_seq_2, lcd_sleep_seq_03, lcd_sleep_seq_4
};

static int lcd_type IDATA_ATTR = LCD_TYPE_UNKNOWN;
static uint8_t lcd_madctl IDATA_ATTR = 0x60;  /* 0x36 parameter per window: 0x20 on 58xx, 0x60 on the others */
uint8_t lcd_id[4];

static void lcd_set_mode(uint32_t mode) ICODE_ATTR __attribute__((noinline));
static void lcd_set_mode(uint32_t mode)
{
    while (!(LCDSTATUS & 0x2));
    udelay(1);
    LCDCON = mode;
}

static void lcd_send_cmd(uint16_t cmd) ICODE_ATTR __attribute__((noinline));
static void lcd_send_cmd(uint16_t cmd)
{
    while (LCDSTATUS & 0x10);
    LCDWCMD = cmd;
}

static void lcd_send_data(uint16_t data) ICODE_ATTR __attribute__((noinline));
static void lcd_send_data(uint16_t data)
{
    while (LCDSTATUS & 0x10);
    LCDWDATA = data;
}

/* the whole GRAM to 0xffff, polled (the panel tables clear it before display on and before display off) */
static void lcd_clear_gram(void)
{
    int n;
    lcd_set_mode(LCD_MODE_CMD);
    lcd_send_cmd(0x2a);
    lcd_send_data(0);
    lcd_send_data(0);
    lcd_send_data((LCD_WIDTH - 1) >> 8);
    lcd_send_data((LCD_WIDTH - 1) & 0xff);
    lcd_send_cmd(0x2b);
    lcd_send_data(0);
    lcd_send_data(0);
    lcd_send_data((LCD_HEIGHT - 1) >> 8);
    lcd_send_data((LCD_HEIGHT - 1) & 0xff);
    lcd_send_cmd(0x2c);
    lcd_set_mode(LCD_MODE_FRAME);
    for (n = 0; n < LCD_WIDTH * LCD_HEIGHT; n++)
    {
        while (LCDSTATUS & 0x10);
        LCDWDATA = 0xffff;
    }
    lcd_set_mode(LCD_MODE_CMD);
}

static void lcd_run_seq(const uint8_t* seq)
{
    while (1)
        switch (*seq++)
        {
        case SEQ_CMD:
        {
            int len;
            lcd_send_cmd(*seq++);
            len = *seq++;
            while (len--) lcd_send_data(*seq++);
            break;
        }
        case SEQ_SLEEP:
        {
            /* udelay() evaluates its argument on every poll: read the step first */
            long us = *seq++ * 1000;
            udelay(us);             /* also runs before the scheduler, so no sleep() */
            break;
        }
        case SEQ_CLEAR:
            lcd_clear_gram();
            break;
        default:
            return;
        }
}

static void lcd_read_id(uint8_t* id)
{
    int i;
    lcd_set_mode(LCD_MODE_CMD);
    lcd_send_cmd(0x04);
    for (i = 0; i < 4; i++)
    {
        long timestamp;
        while (!(LCDSTATUS & 0x2));
        LCDRDATA = 0;
        timestamp = USEC_TIMER;
        while (!(LCDSTATUS & 0x1))
            if (TIMEOUT_EXPIRED(timestamp, 10000)) break;
        id[i] = LCDDBUFF >> 1;
    }
}

static int lcd_detect()
{
    int retry;
    for (retry = 0; retry < 6; retry++)
    {
        if (retry) udelay(10000);
        lcd_read_id(lcd_id);
        if (lcd_id[1] == 0x58) return LCD_TYPE_58XX;
        if (lcd_id[1] == 0x38)
            switch (lcd_id[2])
            {
            case 0xb3: return LCD_TYPE_38B3;
            case 0xc4: return LCD_TYPE_38C4;
            case 0xd5: return LCD_TYPE_38D5;
            case 0xe6: return LCD_TYPE_38E6;
            }
    }
    return LCD_TYPE_UNKNOWN;
}

void lcd_init()
{
    int i;
    mutex_init(&lcd_mutex);
    wakeup_init(&lcd_wakeup);
    lcd_dma_busy = true;
    clockgate_dma(0, 4, true);
    if (!(DMAC0C4CONFIG & 1))
    {
        lcd_dma_busy = false;
        clockgate_dma(0, 4, false);
    }
    clockgate_enable(CLOCKGATE_LCD, true);         /* the only LCD gate on the S5L8702 (gate 1) */
    DMAC0CONFIG = 1;                                /* PL080 master enable (DMAC0 is otherwise only as a loader left it) */
    /* panel reset as Apple's DFU loader does before the init table (wtf mem:0x220049a8): GPIO 0x36 (port 6 bit 6,
       PCON nibble 0xf = output high from stage 1) low 10 ms, high 20 ms */
    PDAT(6) &= ~0x40;
    udelay(10000);
    PDAT(6) |= 0x40;
    udelay(20000);
    lcd_set_mode(LCD_MODE_FRAME);   /* controller setup as the iPod OS does it */
    LCDREG(0x88) = 0x01000000;
    LCDPHTIME = 0x33;
    LCDREG(0x7c) = 0x402;
    lcd_set_mode(LCD_MODE_CMD);
    for (i = 0; i < 4; i++) lcd_send_data(0);     /* the loader's four dummy writes after the reset */
    lcd_type = lcd_detect();
    lcd_madctl = lcd_type == LCD_TYPE_58XX ? 0x20 : 0x60;
    lcd_set_mode(LCD_MODE_CMD);
    /* unknown panel: nothing is sent, the frame writes still go out */
    if (lcd_type != LCD_TYPE_UNKNOWN) lcd_run_seq(lcd_init_seqs[lcd_type]);
}

bool displaylcd_busy() ICODE_ATTR;
bool displaylcd_busy()
{
    return lcd_dma_busy;
}

void displaylcd_sync() ICODE_ATTR;
void displaylcd_sync()
{
    mutex_lock(&lcd_mutex, TIMEOUT_BLOCK);
    while (displaylcd_busy()) wakeup_wait(&lcd_wakeup, TIMEOUT_BLOCK);
    mutex_unlock(&lcd_mutex);
}

void displaylcd_setup(unsigned int startx, unsigned int endx,
                      unsigned int starty, unsigned int endy, bool safe) ICODE_ATTR;
void displaylcd_setup(unsigned int startx, unsigned int endx,
                      unsigned int starty, unsigned int endy, bool safe)
{
    if (!safe)
    {
        mutex_lock(&lcd_mutex, TIMEOUT_BLOCK);
        displaylcd_sync();
    }
    else while (DMAC0C4CONFIG & 1);
    lcd_set_mode(LCD_MODE_CMD);
    lcd_send_cmd(0x36);
    lcd_send_data(lcd_madctl);
    lcd_send_cmd(0x2a);
    lcd_send_data(startx >> 8);
    lcd_send_data(startx & 0xff);
    lcd_send_data(endx >> 8);
    lcd_send_data(endx & 0xff);
    lcd_send_cmd(0x2b);
    lcd_send_data(starty >> 8);
    lcd_send_data(starty & 0xff);
    lcd_send_data(endy >> 8);
    lcd_send_data(endy & 0xff);
    lcd_send_cmd(0x2c);
    lcd_set_mode(LCD_MODE_FRAME);
}

static void displaylcd_dma(void* data, int pixels, bool solid) ICODE_ATTR;
static void displaylcd_dma(void* data, int pixels, bool solid)
{
    int i;
    lcd_dma_busy = true;
    clockgate_dma(0, 4, true);
    for (i = -1; i < (int)ARRAYLEN(lcd_lli) && pixels > 0; i++, pixels -= 0xfff)
    {
        bool last = i + 1 >= ARRAYLEN(lcd_lli) || pixels <= 0xfff;
        struct dma_lli* lli = i < 0 ? (struct dma_lli*)((int)&DMAC0C4LLI) : &lcd_lli[i];
        lli->srcaddr = data;
        lli->dstaddr = (void*)((int)&LCDWDATA);
        lli->nextlli = last ? NULL : &lcd_lli[i + 1];
        lli->control = 0x70240000 | (last ? pixels : 0xfff)
                     | (last ? 0x80000000 : 0) | (solid ? 0 : 0x4000000);
        if (!solid) data += 0x1ffe;
    }
    clean_dcache();
    DMAC0C4CONFIG = 0x88c1;
}

void displaylcd_native(unsigned int startx, unsigned int endx,
                       unsigned int starty, unsigned int endy, void* data)
{
    int pixels = (endx - startx + 1) * (endy - starty + 1);
    if (pixels <= 0) return;
    displaylcd_setup(startx, endx, starty, endy, false);
    displaylcd_dma(data, pixels, false);
    mutex_unlock(&lcd_mutex);
}

void displaylcd_safe_native(unsigned int startx, unsigned int endx,
                            unsigned int starty, unsigned int endy, void* data)
{
    int pixels = (endx - startx + 1) * (endy - starty + 1);
    if (pixels <= 0) return;
    displaylcd_setup(startx, endx, starty, endy, true);
    displaylcd_dma(data, pixels, false);
}

void filllcd_native(unsigned int startx, unsigned int endx,
                    unsigned int starty, unsigned int endy, int color)
{
    int pixels = (endx - startx + 1) * (endy - starty + 1);
    if (pixels <= 0) return;
    displaylcd_setup(startx, endx, starty, endy, false);
    lcd_color = color;
    displaylcd_dma(&lcd_color, pixels, true);
    mutex_unlock(&lcd_mutex);
}

void displaylcd_dither(unsigned int x, unsigned int y, unsigned int width,
                       unsigned int height, void* data, unsigned int datax,
                       unsigned int datay, unsigned int stride, bool solid)
     ICODE_ATTR __attribute__((naked,noinline));
void displaylcd_dither(unsigned int x, unsigned int y, unsigned int width,
                       unsigned int height, void* data, unsigned int datax,
                       unsigned int datay, unsigned int stride, bool solid)
{
    __asm__ volatile("    muls r12, r2, r3             \n");
    __asm__ volatile("    bxeq lr                      \n");
    __asm__ volatile("    stmfd sp!, {r1-r11,lr}       \n");
    __asm__ volatile("    mov r12, #0                  \n");
    __asm__ volatile("    str r12, [sp]                \n");
    __asm__ volatile("    mov r12, r2                  \n");
    __asm__ volatile("    add r8, r2, r2,lsl#1         \n");
    __asm__ volatile("    add r8, r8, #3               \n");
    __asm__ volatile("    add r3, r1, r3               \n");
    __asm__ volatile("    sub r3, r3, #1               \n");
    __asm__ volatile("    mov r2, r1                   \n");
    __asm__ volatile("    add r1, r0, r12              \n");
    __asm__ volatile("    sub r1, r1, #1               \n");
    __asm__ volatile("    bl displaylcd_setup          \n");
    __asm__ volatile("    add sp, sp, #4               \n");
    __asm__ volatile("    mov r0, r8                   \n");
    __asm__ volatile("    bl malloc                    \n");
    __asm__ volatile("    cmp r0, #0                   \n");
    __asm__ volatile("    beq displaylcd_dither_unlock \n");
    __asm__ volatile("    mov r2, r8                   \n");
    __asm__ volatile("    mov r1, #0                   \n");
    __asm__ volatile("    mov r8, r0                   \n");
    __asm__ volatile("    bl memset                    \n");
    __asm__ volatile("    ldr r0, [sp,#0x30]           \n");
    __asm__ volatile("    ldr r1, [sp,#0x34]           \n");
    __asm__ volatile("    ldr r11, [sp,#0x38]          \n");
    __asm__ volatile("    ldr r3, [sp,#0x2c]           \n");
    __asm__ volatile("    mla r0, r1, r11, r0          \n");
    __asm__ volatile("    ldr r12, [sp,#0x04]          \n");
    __asm__ volatile("    ldr r2, [sp,#0x3c]           \n");
    __asm__ volatile("    add r3, r3, r0,lsl#1         \n");
    __asm__ volatile("    cmp r2, #0                   \n");
    __asm__ volatile("    ldreq r1, [sp]               \n");
    __asm__ volatile("    add r3, r3, r0               \n");
    __asm__ volatile("    subeq r11, r11, r1           \n");
    __asm__ volatile("    add r11, r11, r11,lsl#1      \n");
    __asm__ volatile("    movne r10, #3                \n");
    __asm__ volatile("    moveq r10, #0                \n");
    __asm__ volatile("    ldr r9, =0x38300040          \n");
    __asm__ volatile("displaylcd_dither_wait :         \n");
    __asm__ volatile("    ldr r4, [r9,#-0x24]          \n");
    __asm__ volatile("    tst r4, #2                   \n");
    __asm__ volatile("    beq displaylcd_dither_wait   \n");
    __asm__ volatile("    ldr r4, =0x81104eb0          \n");
    __asm__ volatile("    str r4, [r9,#-0x40]          \n");
    __asm__ volatile("displaylcd_dither_y:             \n");
    __asm__ volatile("    ldr lr, [sp]                 \n");
    __asm__ volatile("    mov r4, #0                   \n");
    __asm__ volatile("    mov r5, #0                   \n");
    __asm__ volatile("    mov r6, #0                   \n");
    __asm__ volatile("    mov r7, r8                   \n");
    __asm__ volatile("displaylcd_dither_x:             \n");
    __asm__ volatile("    ldrb r1, [r3], #1            \n");
    __asm__ volatile("    ldrsb r0, [r7,#3]            \n");
    __asm__ volatile("    add r1, r1, r4               \n");
    __asm__ volatile("    add r1, r1, r0               \n");
    __asm__ volatile("    cmp r1, #0xff                \n");
    __asm__ volatile("    mvnhi r1, r1,asr#31          \n");
    __asm__ volatile("    andhi r1, r1, #0xff          \n");
    __asm__ volatile("    mov r0, r1,lsr#2             \n");
    __asm__ volatile("    mov r2, r0,lsl#18            \n");
    __asm__ volatile("    sub r1, r1, r0,lsl#2         \n");
    __asm__ volatile("    sub r1, r1, r0,lsr#4         \n");
    __asm__ volatile("    mov r4, r4,lsr#1             \n");
    __asm__ volatile("    add r4, r4, r1,lsr#2         \n");
    __asm__ volatile("    strb r4, [r7], #1            \n");
    __asm__ volatile("    mov r4, r1,asr#1             \n");
    __asm__ volatile("    ldrb r1, [r3], #1            \n");
    __asm__ volatile("    ldrsb r0, [r7,#3]            \n");
    __asm__ volatile("    add r1, r1, r5               \n");
    __asm__ volatile("    add r1, r1, r0               \n");
    __asm__ volatile("    cmp r1, #0xff                \n");
    __asm__ volatile("    mvnhi r1, r1,asr#31          \n");
    __asm__ volatile("    andhi r1, r1, #0xff          \n");
    __asm__ volatile("    mov r0, r1,lsr#2             \n");
    __asm__ volatile("    orr r2, r2, r0,lsl#10        \n");
    __asm__ volatile("    sub r1, r1, r0,lsl#2         \n");
    __asm__ volatile("    sub r1, r1, r0,lsr#4         \n");
    __asm__ volatile("    mov r5, r5,lsr#1             \n");
    __asm__ volatile("    add r5, r5, r1,lsr#2         \n");
    __asm__ volatile("    strb r5, [r7], #1            \n");
    __asm__ volatile("    mov r5, r1,asr#1             \n");
    __asm__ volatile("    ldrb r1, [r3], #1            \n");
    __asm__ volatile("    ldrsb r0, [r7,#3]            \n");
    __asm__ volatile("    add r1, r1, r6               \n");
    __asm__ volatile("    add r1, r1, r0               \n");
    __asm__ volatile("    cmp r1, #0xff                \n");
    __asm__ volatile("    mvnhi r1, r1,asr#31          \n");
    __asm__ volatile("    andhi r1, r1, #0xff          \n");
    __asm__ volatile("    mov r0, r1,lsr#2             \n");
    __asm__ volatile("    orr r2, r2, r0,lsl#2         \n");
    __asm__ volatile("    sub r1, r1, r0,lsl#2         \n");
    __asm__ volatile("    sub r1, r1, r0,lsr#4         \n");
    __asm__ volatile("    mov r6, r6,lsr#1             \n");
    __asm__ volatile("    add r6, r6, r1,lsr#2         \n");
    __asm__ volatile("    strb r6, [r7], #1            \n");
    __asm__ volatile("displaylcd_dither_wait2:         \n");
    __asm__ volatile("    ldr r0, [r9,#-0x24]          \n");
    __asm__ volatile("    mov r6, r1,asr#1             \n");
    __asm__ volatile("    tst r0, #0x10                \n");
    __asm__ volatile("    bne displaylcd_dither_wait2  \n");
    __asm__ volatile("    str r2, [r9]                 \n");
    __asm__ volatile("    sub r3, r3, r10              \n");
    __asm__ volatile("    subs lr, lr, #1              \n");
    __asm__ volatile("    bne displaylcd_dither_x      \n");
    __asm__ volatile("    add r3, r3, r11              \n");
    __asm__ volatile("    subs r12, r12, #1            \n");
    __asm__ volatile("    bne displaylcd_dither_y      \n");
    __asm__ volatile("displaylcd_dither_free:          \n");
    __asm__ volatile("    mov r0, r8                   \n");
    __asm__ volatile("    bl free                      \n");
    __asm__ volatile("displaylcd_dither_unlock:        \n");
    __asm__ volatile("    ldr r0, =lcd_mutex           \n");
    __asm__ volatile("    bl mutex_unlock              \n");
    __asm__ volatile("    ldmfd sp!, {r2-r11,pc}       \n");
}

void displaylcd(unsigned int x, unsigned int y, unsigned int width, unsigned int height,
                void* data, unsigned int datax, unsigned int datay, unsigned int stride)
{
    displaylcd_dither(x, y, width, height, data, datax, datay, stride, false);
}

void filllcd(unsigned int x, unsigned int y, unsigned int width, unsigned int height, int color)
{
    if (width * height <= 0) return;
    mutex_lock(&lcd_mutex, TIMEOUT_BLOCK);
    displaylcd_sync();
    lcd_color = color;
    displaylcd_dither(x, y, width, height, &lcd_color, 0, 0, 0, true);
    mutex_unlock(&lcd_mutex);
}

void lcd_shutdown()
{
    mutex_lock(&lcd_mutex, TIMEOUT_BLOCK);
    displaylcd_sync();
    lcd_set_mode(LCD_MODE_CMD);
    if (lcd_type != LCD_TYPE_UNKNOWN) lcd_run_seq(lcd_sleep_seqs[lcd_type]);
}

void INT_DMAC0C4()
{
    DMAC0INTTCCLR = 0x10;
    lcd_dma_busy = false;
    clockgate_dma(0, 4, false);
    lcdconsole_callback();
    wakeup_signal(&lcd_wakeup);
}

int lcd_translate_color(uint8_t alpha, uint8_t red, uint8_t green, uint8_t blue)
    ICODE_ATTR __attribute__((naked, noinline));
int lcd_translate_color(uint8_t alpha, uint8_t red, uint8_t green, uint8_t blue)
{
    asm volatile(
        "cmp r0, #0xff             \n\t"
        "moveq r0, #-1             \n\t"
        "moveq pc, lr              \n\t"
        "cmp r0, #0                \n\t"
        "movne r0, #0xff000000     \n\t"
        "orrne r0, r0, #0xff0000   \n\t"
        "mov r2, r2,lsr#2          \n\t"
        "orr r0, r0, r3,lsr#3      \n\t"
        "mov r1, r1,lsr#3          \n\t"
        "orr r0, r0, r2,lsl#5      \n\t"
        "orr r0, r0, r1,lsl#11     \n\t"
        "mov pc, lr                \n\t"
    );
}
