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
#include "timer.h"
#include "thread.h"
#include "s5l8702.h"


void timer_init()
{
    TACMD = (1 << 1);   /* TA_CLR */
    TBCMD = (1 << 1);   /* TB_CLR */
    TCCMD = (1 << 1);   /* TC_CLR */
    TDCMD = (1 << 1);   /* TD_CLR */
    TGCMD = (1 << 1);   /* TG_CLR */
    THCMD = (1 << 1);   /* TH_CLR */
}

void timer_schedule_wakeup(uint32_t usecs)
{
    /* Timer B on ECLK (12 MHz) / 16 = 750 kHz, as the iPod OS clocks its timers (osos mem:0x0836d36c: TxCON 0x240,
       bit 6 = ECLK); the classic's PCLK/64 constants assumed a 76 MHz PCLK, the nano 3G's is 54 MHz */
    if (usecs > 44739242)               /* 65535 ticks at 750000/512 Hz */
    {
        TBPRE = 511;
        TBDATA1 = 65535;
    }
    else if (usecs > 87380)             /* 65535 ticks at 750 kHz */
    {
        TBPRE = 511;
        TBDATA1 = (usecs * 3) >> 11;    /* 750000 / 512 Hz = 3 / 2048 per us */
    }
    else
    {
        TBPRE = 0;
        TBDATA1 = (usecs * 3) >> 2;     /* 0.75 per us */
    }
    if (!TBDATA1) TBDATA1 = 1;
    TBCON = (1 << 13) | /* TB_INT1_EN */
            (0 << 12) | /* TB_INT0_EN */
            (0 << 11) | /* TB_START */
            (2 << 8) |  /* TB_CS = ECLK / 16 */
            (1 << 6) |  /* clock source ECLK */
            (2 << 4);   /* TB_MODE_SEL = one-shot mode */
    TBCMD = (1 << 1);   /* TB_CLR */
    TBCMD = (1 << 0);   /* TB_EN */
}

void timer_kill_wakeup()
{
    TBCMD = (1 << 1);   /* TB_CLR */
    TBCON = TBCON;
}

void INT_TIMERB(void)
{
    TBCON = TBCON;
    scheduler_switch(NULL, NULL);
}
