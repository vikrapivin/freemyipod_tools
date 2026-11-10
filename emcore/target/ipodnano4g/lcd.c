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
#include "mmu.h"
#include "lcdconsole.h"
#include "thread.h"
#include "s5l8720.h"
#include "util.h"
#include "clockgates-target.h"
#include "lcd.h"
#include "timer.h"
#include "contextswitch.h"


static struct dma_lli lcd_lli[(LCD_WIDTH * LCD_HEIGHT - 1) / 0xfff]
    IDATA_ATTR CACHEALIGN_ATTR;

static uint16_t lcd_color IDATA_ATTR;

static struct mutex lcd_mutex IDATA_ATTR;
static struct wakeup lcd_wakeup IDATA_ATTR;

static bool lcd_dma_busy IDATA_ATTR;

/* F01 guard : a frame normally takes ~15 ms. If channel 4 is still "busy" this long after it was
   started (interrupt_init zeroed its CONTROL mid-frame, or its TC interrupt was cleared), it is stopped and the busy
   flag dropped, so neither the console nor the panic path can wait for it forever. */
#define LCD_DMA_TIMEOUT_US 100000
static uint32_t lcd_dma_start IDATA_ATTR;
uint32_t lcd_dma_aborts;              /* number of F01 recoveries (host-readable, n4g_lcdid.py) */

/* Panel identification, filled by lcd_init. Host-readable, n4g_lcdid.py finds it in the.map:
   magic 'LCDP', id[0..3] = reads after DCS command 0x04 (id[0] dummy, id[1] = 0x38, id[2] = panel type),
   type = index into lcd_panels[] (0 b3, 1 c4, 2 d5, 3 e6) or -1, tries = detection rounds used, ran = 1 when
   the panel init sequence was sent. */
struct lcd_panel_info
{
    uint32_t magic;
    uint8_t id[4];
    int8_t type;
    uint8_t tries;
    uint8_t ran;
    uint8_t pad;
};
struct lcd_panel_info lcd_panel_info;

static void lcd_panel_init(void) INITCODE_ATTR;


void lcd_init()
{
    mutex_init(&lcd_mutex);
    wakeup_init(&lcd_wakeup);
    lcd_dma_busy = true;
    clockgate_dma(0, 4, true);
    if (!(DMAC0C4CONFIG & 1))
    {
        lcd_dma_busy = false;
        clockgate_dma(0, 4, false);
    }
    lcd_dma_start = USEC_TIMER;
    lcd_panel_init();
}

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
    LCDWDATA = (data & 0xff) | ((data & 0x7f00) << 1);
}

/* ---------------------------------------------------------------------------------------------------------------
   Panel detection and init.
   The nano 4G shipped with four panels, all answering DCS "read display ID" (0x04) with 0x38 and a type byte
   (bl_DisplayPlatform:0x220..0x28e: Lcd->Read(0x04, 3 bytes), buf[0] == 0x38, buf[1] in {b3, c4, d5, e6}, up to 5
   tries at bl_DisplayPlatform:0x40e..0x41c; Rockbox lcd-nano4g.c lcd_target_get_info). The bootstub detects the
   panel too but indexes its tables with (type & 3) and sends only a 3..4 command wake-up for d5 and e6
   (bootstub:0x220004e8..0x22000540, offset table at 0x2200060c -> 0x22000734 / 0x220007ca); its full d5/e6 tables
   at 0x2200073c / 0x220007d2 are never used. lcd_init therefore always re-sends Apple's complete sequence.
   Tables: byte copies of bl_DisplayPlatform .data, in Apple's format, interpreted like bl_Lcd:0x394 (switch at
   bl_Lcd:0x3a8): 0 = CMD cmd len data[len], 2 = DELAY ms, 3 = MARK (Apple fills the panel RAM, bl_Lcd:0x422;
   Rockbox and this driver skip it), 4 = END. Sent in command mode 0x41000c20 (bl_DisplayPlatform:0x508/0x530),
   8-bit data per write (bl_Lcd:0x5d4), then the controller is put back in 0x41100db8 (bl_DisplayPlatform:0x3f4,
   the state the bootstub leaves at 0x220009d0).
   --------------------------------------------------------------------------------------------------------------- */
#define LCDRDATA (*((uint32_t volatile*)(0x38300010)))   /* write: start one read cycle (bl_Lcd:0x306) */
#define LCDDBUFF (*((uint32_t volatile*)(0x38300014)))   /* read data in bits 8:1 (bl_Lcd:0x338, bootstub:0x220009bc) */
#define LCD_MODE_CMD8  0x41000c20
#define LCD_MODE_FRAME 0x41100db8
#define LCD_PANEL_MAGIC 0x5044434c                       /* 'LCDP' */

enum { OP_CMD = 0, OP_DELAY = 2, OP_MARK = 3, OP_END = 4 };

/* panel 0x38 0xb3: bl_DisplayPlatform:0x62c..0x6d6 (file .data), selected at bl_DisplayPlatform:0x24c */
static const uint8_t INITCONST_ATTR lcd_seq_b3[] =
{
    OP_CMD, 0x11,  0,
    OP_DELAY, 120,  /* ms */
    OP_CMD, 0xfe,  1, 0x00,
    OP_CMD, 0xef,  1, 0x80,
    OP_CMD, 0xc0,  1, 0x0c,
    OP_CMD, 0xc1,  1, 0x03,
    OP_CMD, 0xc2,  2, 0x12, 0x00,
    OP_CMD, 0xc3,  2, 0x12, 0x00,
    OP_CMD, 0xc4,  2, 0x12, 0x00,
    OP_CMD, 0xc5,  2, 0x3a, 0x3e,
    OP_CMD, 0xb1,  2, 0x6a, 0x15,
    OP_CMD, 0xb2,  2, 0x5f, 0x3f,
    OP_CMD, 0xb3,  2, 0x5f, 0x3f,
    OP_CMD, 0xb4,  1, 0x02,
    OP_CMD, 0xb6,  2, 0x12, 0x02,
    OP_CMD, 0x35,  1, 0x00,
    OP_CMD, 0x26,  1, 0x10,
    OP_CMD, 0xe0, 11, 0x0f, 0x42, 0x24, 0x01, 0x00, 0x02, 0xa6, 0x98, 0x05, 0x04, 0x15,
    OP_CMD, 0xe1, 11, 0x00, 0x21, 0x44, 0x02, 0x0f, 0x05, 0x89, 0x6a, 0x02, 0x15, 0x04,
    OP_CMD, 0xe2, 11, 0x7e, 0x04, 0x43, 0x40, 0x00, 0x02, 0x13, 0x00, 0x00, 0x01, 0x0b,
    OP_CMD, 0xe3, 11, 0x40, 0x40, 0x03, 0x74, 0x0e, 0x00, 0x00, 0x31, 0x02, 0x0b, 0x01,
    OP_CMD, 0xe4, 11, 0x5a, 0x43, 0x67, 0x56, 0x00, 0x02, 0x67, 0x72, 0x00, 0x05, 0x12,
    OP_CMD, 0xe5, 11, 0x50, 0x66, 0x47, 0x53, 0x0a, 0x00, 0x27, 0x76, 0x02, 0x12, 0x05,
    OP_CMD, 0x3a,  1, 0x06,
    OP_CMD, 0x13,  0,
    OP_MARK,
    OP_CMD, 0x29,  0,
    OP_DELAY, 1,  /* ms */
    OP_END
};

/* panel 0x38 0xc4: bl_DisplayPlatform:0x570..0x621 (file .data), selected at bl_DisplayPlatform:0x24c */
static const uint8_t INITCONST_ATTR lcd_seq_c4[] =
{
    OP_CMD, 0x01,  0,
    OP_DELAY, 5,  /* ms */
    OP_CMD, 0xc0,  1, 0x00,
    OP_CMD, 0xc1,  1, 0x03,
    OP_CMD, 0xc2,  2, 0x34, 0x00,
    OP_CMD, 0xc3,  2, 0x72, 0x03,
    OP_CMD, 0xc4,  2, 0x73, 0x03,
    OP_CMD, 0xc5,  2, 0x3c, 0x3c,
    OP_CMD, 0xfe,  1, 0x00,
    OP_CMD, 0xb1,  2, 0x6a, 0x15,
    OP_CMD, 0xb2,  2, 0x6a, 0x15,
    OP_CMD, 0xb3,  2, 0x6a, 0x15,
    OP_CMD, 0xb4,  1, 0x02,
    OP_CMD, 0xb6,  2, 0x12, 0x02,
    OP_CMD, 0x35,  1, 0x00,
    OP_CMD, 0x26,  1, 0x10,
    OP_CMD, 0xe0, 11, 0x77, 0x52, 0x76, 0x53, 0x03, 0x03, 0x57, 0x42, 0x10, 0x18, 0x09,
    OP_CMD, 0xe1, 11, 0x0d, 0x00, 0x23, 0x66, 0x0f, 0x15, 0x4d, 0x85, 0x08, 0x02, 0x10,
    OP_CMD, 0xe2, 11, 0x39, 0x60, 0x77, 0x05, 0x03, 0x07, 0x96, 0x64, 0x0d, 0x1a, 0x0a,
    OP_CMD, 0xe3, 11, 0x3f, 0x10, 0x16, 0x44, 0x0e, 0x04, 0x6c, 0x44, 0x04, 0x03, 0x0b,
    OP_CMD, 0xe4, 11, 0x00, 0x61, 0x77, 0x04, 0x02, 0x04, 0x72, 0x32, 0x09, 0x19, 0x06,
    OP_CMD, 0xe5, 11, 0x4f, 0x42, 0x27, 0x67, 0x0f, 0x02, 0x26, 0x33, 0x01, 0x03, 0x09,
    OP_CMD, 0x3a,  1, 0x66,
    OP_CMD, 0x36,  1, 0x00,
    OP_MARK,
    OP_CMD, 0x29,  0,
    OP_DELAY, 1,  /* ms */
    OP_CMD, 0x11,  0,
    OP_DELAY, 120,  /* ms */
    OP_CMD, 0x29,  0,
    OP_DELAY, 1,  /* ms */
    OP_END
};

/* panel 0x38 0xd5: bl_DisplayPlatform:0x796..0x841 (file .data), selected at bl_DisplayPlatform:0x24c */
static const uint8_t INITCONST_ATTR lcd_seq_d5[] =
{
    OP_CMD, 0xfe,  1, 0x00,
    OP_CMD, 0xc0,  1, 0x01,
    OP_CMD, 0xc1,  1, 0x01,
    OP_CMD, 0xc2,  2, 0x03, 0x00,
    OP_CMD, 0xc3,  2, 0x01, 0x00,
    OP_CMD, 0xc4,  2, 0x03, 0x00,
    OP_CMD, 0xc5,  2, 0x34, 0x34,
    OP_CMD, 0xc7,  1, 0x00,
    OP_CMD, 0xb1,  2, 0x6d, 0x15,
    OP_CMD, 0xb2,  2, 0x6d, 0x15,
    OP_CMD, 0xb3,  2, 0x6d, 0x15,
    OP_CMD, 0xb4,  1, 0x03,
    OP_CMD, 0xb6,  2, 0x11, 0x02,
    OP_CMD, 0x35,  1, 0x00,
    OP_CMD, 0x26,  1, 0x10,
    OP_CMD, 0xe0, 11, 0x23, 0x42, 0x20, 0x42, 0x0e, 0x01, 0xf5, 0xeb, 0x1e, 0x05, 0x18,
    OP_CMD, 0xe1, 11, 0x5f, 0x22, 0x36, 0x21, 0x03, 0x1e, 0xfe, 0x7b, 0x02, 0x07, 0x18,
    OP_CMD, 0xe2, 11, 0x5f, 0x34, 0x53, 0x77, 0x0a, 0x00, 0x70, 0xf4, 0x14, 0x06, 0x0f,
    OP_CMD, 0xe3, 11, 0x0f, 0x23, 0x31, 0x54, 0x0f, 0x0b, 0x8e, 0x08, 0x00, 0x05, 0x15,
    OP_CMD, 0xe4, 11, 0x5f, 0x33, 0x42, 0x14, 0x0e, 0x04, 0xa6, 0xf7, 0x0e, 0x00, 0x14,
    OP_CMD, 0xe5, 11, 0x0c, 0x43, 0x44, 0x44, 0x0d, 0x0d, 0x7f, 0x39, 0x03, 0x02, 0x10,
    OP_CMD, 0x3a,  1, 0x66,
    OP_CMD, 0x36,  1, 0x00,
    OP_MARK,
    OP_CMD, 0x29,  0,
    OP_DELAY, 1,  /* ms */
    OP_CMD, 0x11,  0,
    OP_DELAY, 120,  /* ms */
    OP_END
};

/* panel 0x38 0xe6: bl_DisplayPlatform:0x6e1..0x78b (file .data), selected at bl_DisplayPlatform:0x24c */
static const uint8_t INITCONST_ATTR lcd_seq_e6[] =
{
    OP_CMD, 0x11,  0,
    OP_DELAY, 120,  /* ms */
    OP_CMD, 0xfe,  1, 0x00,
    OP_CMD, 0xef,  1, 0x80,
    OP_CMD, 0xc0,  1, 0x13,
    OP_CMD, 0xc1,  1, 0x03,
    OP_CMD, 0xc2,  2, 0x12, 0x00,
    OP_CMD, 0xc3,  2, 0x12, 0x00,
    OP_CMD, 0xc4,  2, 0x12, 0x00,
    OP_CMD, 0xc5,  2, 0x2a, 0x3c,
    OP_CMD, 0xb1,  2, 0x6a, 0x15,
    OP_CMD, 0xb2,  2, 0x5f, 0x3f,
    OP_CMD, 0xb3,  2, 0x5f, 0x3f,
    OP_CMD, 0xb4,  1, 0x02,
    OP_CMD, 0xb6,  2, 0x12, 0x02,
    OP_CMD, 0x35,  1, 0x00,
    OP_CMD, 0x26,  1, 0x10,
    OP_CMD, 0xe0, 11, 0x0f, 0x53, 0x45, 0x07, 0x00, 0x00, 0xb9, 0xf6, 0x08, 0x04, 0x18,
    OP_CMD, 0xe1, 11, 0x00, 0x47, 0x55, 0x03, 0x0f, 0x08, 0x6f, 0x9b, 0x00, 0x18, 0x04,
    OP_CMD, 0xe2, 11, 0x7e, 0x03, 0x54, 0x75, 0x00, 0x00, 0x3a, 0x52, 0x03, 0x02, 0x10,
    OP_CMD, 0xe3, 11, 0x70, 0x55, 0x04, 0x73, 0x0e, 0x03, 0x25, 0xa3, 0x00, 0x10, 0x02,
    OP_CMD, 0xe4, 11, 0x1a, 0x72, 0x33, 0x76, 0x00, 0x00, 0xeb, 0x97, 0x03, 0x05, 0x17,
    OP_CMD, 0xe5, 11, 0x70, 0x36, 0x73, 0x12, 0x0a, 0x03, 0x79, 0xbe, 0x00, 0x17, 0x05,
    OP_CMD, 0x3a,  1, 0x06,
    OP_CMD, 0x13,  0,
    OP_MARK,
    OP_CMD, 0x29,  0,
    OP_DELAY, 1,  /* ms */
    OP_END
};

static const uint8_t lcd_panel_ids[4] INITCONST_ATTR = { 0xb3, 0xc4, 0xd5, 0xe6 };
static const uint8_t* const lcd_panel_seqs[4] INITCONST_ATTR = { lcd_seq_b3, lcd_seq_c4, lcd_seq_d5, lcd_seq_e6 };

/* bl_DisplayPlatform:0x458 / 0x508: wait until the interface is idle (STATUS bit 1), Stall(1), write LCDCON. */
static void lcd_set_mode(uint32_t mode) INITCODE_ATTR;
static void lcd_set_mode(uint32_t mode)
{
    while (!(LCDSTATUS & 0x2));
    udelay(1);
    LCDCON = mode;
}

static void lcd_send_data8(uint8_t data) INITCODE_ATTR;
static void lcd_send_data8(uint8_t data)
{
    while (LCDSTATUS & 0x10);
    LCDWDATA = data;
}

/* bl_Lcd:0x2f8: start a read cycle, wait for STATUS bit 0 (Apple bounds this wait too), data = DBUFF >> 1. */
static bool lcd_read8(uint8_t* data) INITCODE_ATTR;
static bool lcd_read8(uint8_t* data)
{
    long start;
    while (!(LCDSTATUS & 0x2));
    LCDRDATA = 0;
    start = USEC_TIMER;
    while (!(LCDSTATUS & 0x1)) if (TIMEOUT_EXPIRED(start, 1000)) return false;
    *data = LCDDBUFF >> 1;
    return true;
}

/* Returns the index into lcd_panel_ids or -1. 4 reads per try: dummy, 0x38, type, last ID byte
   (Apple reads len+1 = 4 and drops the first, bl_Lcd:0x2ce..0x2e0; the stub uses its 3rd read). */
static int lcd_detect_panel(void) INITCODE_ATTR;
static int lcd_detect_panel(void)
{
    int tries, i, type;
    for (tries = 1; tries <= 5; tries++)
    {
        lcd_panel_info.tries = tries;
        lcd_set_mode(LCD_MODE_CMD8);
        lcd_send_cmd(0x04);
        for (i = 0; i < 4; i++)
            if (!lcd_read8(&lcd_panel_info.id[i])) break;
        if (i == 4 && lcd_panel_info.id[1] == 0x38)
            for (type = 0; type < 4; type++)
                if (lcd_panel_info.id[2] == lcd_panel_ids[type]) return type;
    }
    return -1;
}

static void lcd_run_seq(const uint8_t* seq) INITCODE_ATTR;
static void lcd_run_seq(const uint8_t* seq)
{
    while (1)
        switch (*seq++)
        {
        case OP_CMD:
        {
            int len;
            lcd_send_cmd(*seq++);
            len = *seq++;
            while (len--) lcd_send_data8(*seq++);
            break;
        }
        case OP_DELAY:
        {
            long us = *seq++ * 1000;   /* udelay() evaluates its argument on every poll */
            udelay(us);
            break;
        }
        case OP_MARK:
            break;
        default:   /* OP_END */
            return;
        }
}

static void lcd_panel_init(void)
{
    int type;
    lcd_panel_info.magic = LCD_PANEL_MAGIC;
    type = lcd_detect_panel();
    lcd_panel_info.type = type;
    if (type >= 0)
    {
        lcd_run_seq(lcd_panel_seqs[type]);
        lcd_panel_info.ran = 1;
    }
    /* unknown panel: leave whatever the bootstub sent */
    lcd_set_mode(LCD_MODE_FRAME);
}

/* F01 recovery: stop channel 4 and drop the busy flag. Callable with IRQs on or off (panic path). */
static void lcd_dma_abort(void) ICODE_ATTR;
static void lcd_dma_abort(void)
{
    uint32_t mode = enter_critical_section();
    DMAC0C4CONFIG = 0;            /* PL080: clearing the enable bit stops the channel (queued data is lost) */
    DMAC0INTTCCLR = 0x10;
    DMAC0INTERRCLR = 0x10;
    lcd_dma_busy = false;
    lcd_dma_aborts++;
    clockgate_dma(0, 4, false);
    leave_critical_section(mode);
}

void lcd_shutdown()
{
    mutex_lock(&lcd_mutex, TIMEOUT_BLOCK);
    displaylcd_sync();
    while (!(LCDSTATUS & 0x2));
    LCDCON = 0x81100db8;
}

bool displaylcd_busy() ICODE_ATTR;
bool displaylcd_busy()
{
    if (lcd_dma_busy && TIMEOUT_EXPIRED(lcd_dma_start, LCD_DMA_TIMEOUT_US)) lcd_dma_abort();
    return lcd_dma_busy;
}

void displaylcd_sync() ICODE_ATTR;
void displaylcd_sync()
{
    mutex_lock(&lcd_mutex, TIMEOUT_BLOCK);
    while (displaylcd_busy()) wakeup_wait(&lcd_wakeup, LCD_DMA_TIMEOUT_US / 4);
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
    else
    {
        uint32_t start = USEC_TIMER;
        while (DMAC0C4CONFIG & 1)
            if (TIMEOUT_EXPIRED(start, LCD_DMA_TIMEOUT_US))
            {
                lcd_dma_abort();
                break;
            }
    }
    while (!(LCDSTATUS & 0x2));
    LCDCON = 0x81100db8;
    lcd_send_cmd(0x2a);
    lcd_send_data(startx);
    lcd_send_data(endx);
    lcd_send_cmd(0x2b);
    lcd_send_data(starty);
    lcd_send_data(endy);
    lcd_send_cmd(0x2c);
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
    lcd_dma_start = USEC_TIMER;
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
    __asm__ volatile("    mov r8, r2,lsl#2             \n");
    __asm__ volatile("    add r3, r1, r3               \n");
    __asm__ volatile("    add r8, r8, #4               \n");
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
    __asm__ volatile("    movne r10, #0                \n");
    __asm__ volatile("    moveq r10, #3                \n");
    __asm__ volatile("    ldr r9, =0x38300040          \n");
    __asm__ volatile("displaylcd_dither_wait :         \n");
    __asm__ volatile("    ldr r4, [r9,#-0x24]          \n");
    __asm__ volatile("    tst r4, #2                   \n");
    __asm__ volatile("    beq displaylcd_dither_wait   \n");
    __asm__ volatile("    ldr r4, =0x81104eb8          \n");
    __asm__ volatile("    str r4, [r9,#-0x40]          \n");
    __asm__ volatile("    ldr r6, =0x30303             \n");
    __asm__ volatile("    ldr r4, =0x808080            \n");
    __asm__ volatile("displaylcd_dither_y:             \n");
    __asm__ volatile("    ldr lr, [sp]                 \n");
    __asm__ volatile("    mov r5, #0                   \n");
    __asm__ volatile("    mov r7, r8                   \n");
    __asm__ volatile("displaylcd_dither_x:             \n"); // the lcd can accept one pixel every 25 clocks
    __asm__ volatile("    ldr r0, [r3]                 \n"); // 1 cycle, 2 mem, r0 latency 4, r3 early
    __asm__ volatile("    add r3, r3, r10              \n"); // 1 cycle
    __asm__ volatile("    ldr r1, [r7,#4]              \n"); // 1 cycle, 1 mem, r1 latency 3, r7 early
    __asm__ volatile("    subs lr, lr, #1              \n"); // 1 cycle
    __asm__ volatile("    ssub8 r0, r0, r4             \n"); // 1 cycle
    __asm__ volatile("    sadd8 r1, r1, r5             \n"); // 1 cycle
    __asm__ volatile("    qadd8 r0, r0, r1             \n"); // 1 cycle, r0 latency 2
                                                             // bubble (due to r0 latency)
    __asm__ volatile("    sadd8 r0, r0, r4             \n"); // 1 cycle
    __asm__ volatile("    bic r2, r0, r6               \n"); // 1 cycle
    __asm__ volatile("    and r1, r6, r0,lsr#6         \n"); // 1 cycle, r0 early
    __asm__ volatile("    orr r2, r2, r1               \n"); // 1 cycle
    __asm__ volatile("    mov r1, r5                   \n"); // 1 cycle
    __asm__ volatile("    shsub8 r5, r0, r2            \n"); // 1 cycle
    __asm__ volatile("    rev r0, r0                   \n"); // 1 cycle, r1 early
    __asm__ volatile("    shadd8 r1, r1, r5            \n"); // 1 cycle
    __asm__ volatile("    mov r0, r0, lsr#8            \n"); // 1 cycle, r1 early
    __asm__ volatile("    str r0, [r9]                 \n"); // 1 cycle, 1 mem, r9 early
    __asm__ volatile("    str r1, [r7], #4             \n"); // 1 cycle, 1 mem, r7 early
    __asm__ volatile("    nop                          \n"); // 2 cycles
    __asm__ volatile("    nop                          \n"); // 2 cycles
    __asm__ volatile("    nop                          \n"); // 2 cycles
    __asm__ volatile("    nop                          \n"); // 2 cycles
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

void INT_DMAC0C4()
{
    DMAC0INTTCCLR = 0x10;
    DMAC0INTERRCLR = 0x10; /* : an error left pending would re-enter this IRQ for ever */
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
