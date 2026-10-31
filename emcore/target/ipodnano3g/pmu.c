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
#include "i2c.h"
#include "pmu.h"
#include "thread.h"
#include "timer.h"


static struct mutex pmumutex;

void pmu_read_multiple(int address, int count, unsigned char* buffer)
{
    i2c_recv(0, 0xe6, address, buffer, count);
}

void pmu_write_multiple(int address, int count, unsigned char* buffer)
{
    i2c_send(0, 0xe6, address, buffer, count);
}

unsigned char pmu_read(int address)
{
    return i2c_recvbyte(0, 0xe6, address);
}

void pmu_write(int address, unsigned char val)
{
    i2c_sendbyte(0, 0xe6, address, val);
}

/* The nano 3G's PMU is a D1671 (I2C bus 0, 0xe6), not the classic's PCF50635: registers below follow Apple's iPod OS
 * (addresses cited per function); Rockbox's pmu-nano3g.c agrees where it covers them. */
extern uint32_t i2c_errors;

void pmu_init()
{
    mutex_init(&pmumutex);
    /* Apple's PMU init (osos mem:0x082dca64): IRQ masks 0x06/0x07, clear events 0x02/0x03. Apple also writes 0x1d = 0,
       which the DFU loader sets to 0x0a (a rail): left alone until a device test says what it is. The charger
       (0x21/0x22) is left as the ROM/DFU set it. */
    pmu_write_multiple(0x06, 2, (unsigned char*)"\x02\xbe");
    pmu_write_multiple(0x02, 2, (unsigned char*)"\xff\xff");
}

/* One conversion of ADC channel adc (0..7), 10 bits, or -1 (osos mem:0x0836ba08: reg 0x30 = 0x20 | channel | 8 starts,
   bit 3 clears when done, result (0x32 << 2) | (0x31 & 3), then 0x30 = 0x20 idle). Apple's no-completion path writes
   0x80/0x87 and halts; emCORE gives up instead. */
int pmu_read_adc(unsigned int adc)
{
    int value = -1, tries;
    uint32_t err0;
    mutex_lock(&pmumutex, TIMEOUT_BLOCK);
    err0 = i2c_errors;
    pmu_write(0x30, 0x20 | (adc & 7) | 0x08);
    for (tries = 0; tries < 20; tries++)
    {
        udelay(50);
        if (!(pmu_read(0x30) & 0x08))
        {
            value = (pmu_read(0x32) << 2) | (pmu_read(0x31) & 3);
            break;
        }
    }
    pmu_write(0x30, 0x20);
    if (i2c_errors != err0) value = -1;
    mutex_unlock(&pmumutex);
    return value;
}

/* millivolts, -1 if unreadable: channel 4 is the battery (osos mem:0x082dcbb4, mem:0x0836b8e0); Apple converts with a
   firmware table, the scale here is Rockbox's measured 2500 + raw * 2000 / 1023 (adc-nano3g.c), 4 samples */
int pmu_read_battery_voltage(void)
{
    int i, sum = 0, v;
    for (i = 0; i < 4; i++)
    {
        v = pmu_read_adc(4);
        if (v < 0) return -1;
        sum += v;
    }
    return 2500 + (sum / 4) * 2000 / 1023;
}

/* milliamps: no current channel identified on the D1671 */
int pmu_read_battery_current(void)
{
    return -1;
}

/* The classic's LDO registers (0x2d.., 0x3b/0x3c) do not exist on the D1671 and the rail assignment of Apple's
   regulator registers (0x13/0x15/0x16/0x18/0x1a/0x1b, osos mem:0x082dc814..0x082dce18) is not established: no-ops. */
void pmu_ldo_on_in_standby(unsigned int ldo, int onoff) { (void)ldo; (void)onoff; }
void pmu_ldo_set_voltage(unsigned int ldo, unsigned char voltage) { (void)ldo; (void)voltage; }
void pmu_ldo_power_on(unsigned int ldo) { (void)ldo; }
void pmu_ldo_power_off(unsigned int ldo) { (void)ldo; }

/* wake sources on the D1671 are the IRQ masks (0x06/0x07, set by pmu_init); a per-call condition is not established */
void pmu_set_wake_condition(unsigned char condition)
{
    (void)condition;
}

/* standby (power off): osos mem:0x082dced0 writes reg 0x08 = 1 */
void pmu_enter_standby(void)
{
    pmu_write(0x08, 0x01);
}

/* RTC: 6 bytes at 0x40 (sec, min, hour, day, month, year), bit 6 of the seconds byte is a flag (osos mem:0x0836bc80,
   write mem:0x0836beb8: seconds | 0x40 first, then one write per register) */
void pmu_read_rtc(unsigned char* buffer)
{
    pmu_read_multiple(0x40, 6, buffer);
    buffer[0] &= ~0x40;
}

void pmu_write_rtc(unsigned char* buffer)
{
    int i;
    for (i = 0; i < 6; i++) pmu_write(0x40 + i, buffer[i] | (i == 0 ? 0x40 : 0));
}
