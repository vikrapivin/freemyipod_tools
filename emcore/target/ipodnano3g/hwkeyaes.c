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
#include "hwkeyaes.h"
#include "s5l8702.h"
#include "thread.h"
#include "timer.h"
#include "clockgates.h"
#include "mmu.h"


struct mutex hwkeyaes_mutex;


/* S5L8702 AES engine, register sequence as in the restore OS aes routine (aupd mem:0x0800371c) and the mask ROM
   (bootrom mem:0x20001b8c / 0x20001c54): +0x14 = 0xe (decrypt) / 0xf (encrypt) for AES-128, key selector at +0x6c,
   CBC with the IV at +0x74 (always 0 here), status bits cleared by writing them back.
   Gate 10 of the first clock gate register is the AES unit (aupd mem:0x08002b68 called with mask 0x400). */
void hwkeyaes(enum hwkeyaes_direction direction, uint32_t keyidx, void* data, uint32_t size)
{
    int i;
    mutex_lock(&hwkeyaes_mutex, TIMEOUT_BLOCK);
    clockgate_enable(10, true);
    for (i = 0; i < 4; i++) AESIV[i] = 0;
    AESUNKREG0 = 1;
    udelay(100);
    AESUNKREG0 = 0;
    AESCONTROL = 1;
    AESUNKREG1 = 0;
    AESTYPE = keyidx;
    AESKEYLEN = direction == HWKEYAES_ENCRYPT ? 0xf : 0xe;
    AESOUTSIZE = size;
    AESOUTADDR = data;
    AESINSIZE = size;
    AESINADDR = data;
    AESAUXSIZE = size;
    AESAUXADDR = data;
    AESSIZE3 = size;
    AESSTATUS = 0xf;
    invalidate_dcache();
    AESGO = 1;
    while (!(AESSTATUS & 0xf)) sleep(MIN(10000, size >> 4));
    AESSTATUS = 0xf;
    AESCONTROL = 0;
    invalidate_dcache();
    clockgate_enable(10, false);
    mutex_unlock(&hwkeyaes_mutex);
}
