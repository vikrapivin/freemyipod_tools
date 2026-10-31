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
#include "s5l8702.h"
#include "power.h"
#include "pmu.h"
#include "thread.h"


void reset();


void power_off(void)
{
    pmu_write_multiple(0x02, 2, (unsigned char*)"\xff\xff");   /* clear events */
    pmu_enter_standby();                                        /* D1671 reg 0x08 = 1 (osos mem:0x082dced0) */
    sleep(1000000);
    reset();            /* only reached if the PMU refused standby (Apple does not try with VBUS/FireWire present) */
}

void power_init(void)
{
    pmu_init();
}                          

/* The nano 3G reports external power and charging through the PMU (STATUSA 0x04, osos mem:0x082dcf88 / 0x082dc784 /
   0x0836b920), not through the classic's GPIO port 11. A failed read (0xff from a dead bus) reports nothing present. */
extern uint32_t i2c_errors;

bool vbus_state(void)
{
    uint32_t err0 = i2c_errors;
    unsigned char st = pmu_read(0x04), sys = pmu_read(0x08);
    if (i2c_errors != err0) return false;
    if (sys & 0x04) return (st & 0x40) != 0;    /* charger suspended: VBUS shows in bit 6 */
    return (st & 0x08) != 0;
}

static bool firewire_state(void)
{
    uint32_t err0 = i2c_errors;
    unsigned char st = pmu_read(0x04);
    return i2c_errors == err0 && (st & 0x10) != 0;
}

bool external_power_state(void)
{
    return vbus_state() || firewire_state();
}

bool charging_state(void)
{
    uint32_t err0;
    unsigned char chg, sys, stb;
    if (!external_power_state()) return false;
    err0 = i2c_errors;
    chg = pmu_read(0x21);
    sys = pmu_read(0x08);
    stb = pmu_read(0x05);
    if (i2c_errors != err0) return false;
    if (!(chg & 0x7e)) return false;            /* charger not enabled */
    if (sys & 0x04) return false;               /* suspended */
    return !(stb & 0x06);
}

int read_battery_voltage(int battery)
{
    if (battery == 0) return pmu_read_battery_voltage();
    return -1;
}

int read_battery_current(int battery)
{
    if (battery == 0) return pmu_read_battery_current();
    return -1;
}

int read_battery_mwh_design(int battery)
{
    if (battery == 0) return 2035;
    return -1;
}

int read_battery_mwh_full(int battery)
{
    if (battery == 0) return 2035;
    return -1;
}

int read_battery_mwh_current(int battery)
{
    /* capacity of the nano 3G cell unknown: 0 at or below the 3300 mV shutoff (Rockbox powermgmt-nano3g.c), linear above */
    if (battery == 0)
    {
        int v = read_battery_voltage(0);
        return v < 0 ? 0 : MAX(0, (v - 3300) * 3);
    }
    return -1;
}

int read_battery_mw(int battery)
{
    return -1;
}

enum battery_state read_battery_state(int battery)
{
    if (battery != 0) return BATTERY_STATE_INVALID;
    {
        int v = read_battery_voltage(0);
        if (v < 0) return BATTERY_STATE_INVALID;    /* unreadable is not "no battery" */
        if (v < 100) return BATTERY_STATE_NONPRESENT;
    }
    if (charging_state()) return BATTERY_STATE_CHARGING;
    if (external_power_state()) return BATTERY_STATE_IDLE;
    return BATTERY_STATE_DISCHARGING;
}

int read_input_voltage(int input)
{
    return -1;
}

int read_input_current(int input)
{
    return -1;
}

int read_input_mw(int input)
{
    return -1;
}

enum input_state read_input_state(int input)
{
    if (input != 0) return INPUT_STATE_INVALID;
    if (external_power_state()) return INPUT_STATE_ACTIVE;
    return INPUT_STATE_NONPRESENT;
}

