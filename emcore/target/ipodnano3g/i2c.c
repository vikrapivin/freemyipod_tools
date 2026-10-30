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
#include "thread.h"
#include "timer.h"
#include "s5l8702.h"
#include "clockgates.h"


/* S5L8702 I2C master, following Apple's driver in the iPod OS (osos mem:0x0836cbf0 setup, 0x0836cc48 write,
 * 0x0836ca30 read, 0x0836c894 byte state machine, 0x0836c85c IICSTA2 clear + IICCON rewrite); Rockbox's
 * i2c-s5l8702.c (GPLv2+) has the same register protocol and was the starting point. The classic S3C sequence this
 * file had before waited for IICCON bit 4 as an "interrupt pending" flag and set STOP right after; on this
 * controller writing IICCON moves the next byte, and the byte-done / STOP status is IICSTA2 (+0x20) bits 8 / 13
 * (write 1 to clear). Device result with the old code: reads worked, writes never reached the PMU.
 * Apple's setup: own address 0x40, +0x14 = 0 (clock source; Rockbox uses 1 = ECLK), +0x18 = 0, IICCON = 0x184
 * (bit 8 interrupt enable, ACK, divider 4); emCORE polls IICSTA2 instead of taking the interrupt: IICCON = 0x84.
 * Every wait is bounded (I2C_TIMEOUT_US): a stuck bus fails the transfer instead of hanging the caller. */
#define IICUNK14(bus) (*((uint32_t volatile*)(0x3C600014 + 0x300000 * (bus))))
#define IICUNK18(bus) (*((uint32_t volatile*)(0x3C600018 + 0x300000 * (bus))))
#define IICSTA2(bus)  (*((uint32_t volatile*)(0x3C600020 + 0x300000 * (bus))))
#define I2C_TIMEOUT_US 20000

static struct mutex i2cmutex;
static bool i2c_ready[2];
uint32_t i2c_errors;            /* transfers that failed after all retries (nano 3G; i2c.h's API returns nothing) */
#define I2C_TRIES 3             /* Apple re-runs the controller setup and retries a failed transfer (osos mem:0x082c0a10) */

static int wait_rdy(int bus)
{
    long start = USEC_TIMER;
    while (IIC10(bus))
        if (TIMEOUT_EXPIRED(start, I2C_TIMEOUT_US)) return -1;
    return 0;
}

static void i2c_on(int bus)
{
    clockgate_enable(CLOCKGATE_I2C(bus), true);
}

static void i2c_off(int bus)
{
    wait_rdy(bus);
    IICSTAT(bus) = 0;           /* serial output off */
    wait_rdy(bus);
    clockgate_enable(CLOCKGATE_I2C(bus), false);
}

/* one-time controller setup, Apple's order (osos mem:0x0836cbf0); the pins are set by stage 1's PCON table */
#define I2C_CON 0x84            /* ACK generation, CLKSEL 0, divider 4; Apple also sets bit 8 (interrupt) */
static void i2c_preinit(int bus)
{
    i2c_on(bus);
    wait_rdy(bus);
    IICADD(bus) = 0x40;         /* own slave address */
    wait_rdy(bus);
    IICUNK14(bus) = 0;          /* clock source as Apple sets it */
    wait_rdy(bus);
    IICUNK18(bus) = 0;
    wait_rdy(bus);
    IICSTAT(bus) = 0x80;
    wait_rdy(bus);
    IICCON(bus) = 0;
    wait_rdy(bus);
    IICSTAT(bus) = 0;
    wait_rdy(bus);
    IICCON(bus) = I2C_CON;
    wait_rdy(bus);
    IICSTAT(bus) = 0x10;        /* serial output on */
    wait_rdy(bus);
    IICSTA2(bus) = 0x3f00;
    i2c_off(bus);
    i2c_ready[bus] = true;
}

/* wait until the byte (or START/STOP) has gone out: IICSTAT bit 5 drops or IICSTA2 reports byte done / STOP */
static int i2c_wait_io(int bus)
{
    long start = USEC_TIMER;
    while ((IICSTAT(bus) & (1 << 5)) && !(IICSTA2(bus) & ((1 << 8) | (1 << 13))))
    {
        wait_rdy(bus);
        if (TIMEOUT_EXPIRED(start, I2C_TIMEOUT_US)) return -1;
    }
    IICSTA2(bus) |= (1 << 8) | (1 << 13);
    return 0;
}

static int i2c_start(int bus, uint32_t slave, bool rd)
{
    int mode = rd ? 0x80 : 0xC0;
    wait_rdy(bus);
    IICCON(bus) = I2C_CON;      /* Apple: IICCON |= 0x80 on the value set up above */
    wait_rdy(bus);
    IICSTAT(bus) = mode;
    wait_rdy(bus);
    IICDS(bus) = (slave & ~1) | (rd ? 1 : 0);
    wait_rdy(bus);
    IICSTAT(bus) = mode | 0x30; /* START, serial output on */
    wait_rdy(bus);
    if (i2c_wait_io(bus)) return -1;
    return (IICSTAT(bus) & 1) ? -1 : 0;     /* no ACK */
}

static void i2c_stop(int bus)
{
    wait_rdy(bus);
    IICSTAT(bus) &= ~0x20;      /* Apple: 0xd0 after a write, 0x90 after a read */
    wait_rdy(bus);
    IICCON(bus) = IICCON(bus);  /* Apple rewrites IICCON to let the STOP out */
    i2c_wait_io(bus);
}

static int i2c_wr_internal(int bus, uint32_t slave, int address, const uint8_t *data, uint32_t len)
{
    int rc = 0;
    uint8_t addr = address;
    const uint8_t *ptr = data;
    if (i2c_start(bus, slave, false) == 0)
    {
        if (address >= 0) { ptr = &addr; len++; }
        while (len--)
        {
            wait_rdy(bus);
            IICDS(bus) = *ptr;
            udelay(5);
            wait_rdy(bus);
            IICCON(bus) = IICCON(bus);      /* send it */
            if (i2c_wait_io(bus) || (IICSTAT(bus) & 1)) { rc = -2; break; }
            if (ptr == &addr) ptr = data;
            else ptr++;
        }
    }
    else rc = -1;
    i2c_stop(bus);
    return rc;
}

static int i2c_rd_internal(int bus, uint32_t slave, uint8_t *data, uint32_t len)
{
    int rc = 0;
    if (i2c_start(bus, slave, true) == 0)
    {
        while (len--)
        {
            wait_rdy(bus);
            IICCON(bus) &= ~(len ? 0 : 0x80);   /* ACK, NAK on the last byte */
            if (i2c_wait_io(bus)) { rc = -2; break; }
            *data++ = IICDS(bus);
        }
    }
    else rc = -3;
    i2c_stop(bus);
    return rc;
}

void i2c_init()
{
    mutex_init(&i2cmutex);
}

void i2c_send(uint32_t bus, uint32_t device, uint32_t address, const uint8_t* data, uint32_t length)
{
    int rc, tries = 0;
    if (!length) return;
    mutex_lock(&i2cmutex, TIMEOUT_BLOCK);
    do
    {
        if (!i2c_ready[bus & 1]) i2c_preinit(bus);
        i2c_on(bus);
        rc = i2c_wr_internal(bus, device, (int)address, data, length);
        i2c_off(bus);
        if (rc) i2c_ready[bus & 1] = false;
    } while (rc && ++tries < I2C_TRIES);
    if (rc) i2c_errors++;
    mutex_unlock(&i2cmutex);
}

void i2c_recv(uint32_t bus, uint32_t device, uint32_t address, uint8_t* data, uint32_t length)
{
    int rc, tries = 0;
    if (!length) return;
    mutex_lock(&i2cmutex, TIMEOUT_BLOCK);
    do
    {
        if (!i2c_ready[bus & 1]) i2c_preinit(bus);
        i2c_on(bus);
        rc = i2c_wr_internal(bus, device, (int)address, NULL, 0);
        if (rc == 0) rc = i2c_rd_internal(bus, device, data, length);
        i2c_off(bus);
        if (rc) i2c_ready[bus & 1] = false;
    } while (rc && ++tries < I2C_TRIES);
    if (rc)
    {
        memset(data, 0xff, length);     /* never hand back stale bytes */
        i2c_errors++;
    }
    mutex_unlock(&i2cmutex);
}

void i2c_sendbyte(uint32_t bus, uint32_t device, uint32_t address, uint32_t data)
{
    uint8_t buf[1];
    buf[0] = data;
    i2c_send(bus, device, address, buf, 1);
}

uint8_t i2c_recvbyte(uint32_t bus, uint32_t device, uint32_t address)
{
    uint8_t buf[1];
    i2c_recv(bus, device, address, buf, 1);
    return buf[0];
}
