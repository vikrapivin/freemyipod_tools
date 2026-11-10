/* Boot milestones for live USB/LCD diagnostics only. SRAM is NOT evidence
 * after a reset: collect the LCD/USB output before a risky operation.
 * BOOT_HALT_AT can stop at a milestone during bring-up. */
#ifndef BOOT_BEACON_H
#define BOOT_BEACON_H
#include "global.h"
#define BOOT_BEACON_BASE   0x22020000u
#define BOOT_BEACON_MAGIC  0x314e4342u
#define BCN_INIT_ENTRY      1
#define BCN_SCHED           2
#define BCN_CONSOLE         3
#define BCN_LCD             4
#define BCN_LCDCONSOLE      5
#define BCN_MALLOC          6
#define BCN_INITTHREAD_MADE 7
#define BCN_TIMER           8
#define BCN_INTERRUPT       9
#define BCN_INITTHREAD      10
#define BCN_I2C             11
#define BCN_POWER           12
#define BCN_STORAGE_THREAD  13
#define BCN_USB             14
#define BCN_BACKLIGHT       15
#define BCN_BUTTON          16
#define BCN_INITTHREAD_DONE 17
#define BCN_STORAGE_ENTRY   20
#define BCN_STORAGE_INIT    21
#define BCN_DISK_MOUNT      22
#define BCN_NAND_INIT       30
#define BCN_NAND_POC_INIT   31
#define BCN_PANIC           40   /* aux = panic mode */
void boot_beacon(uint32_t id, uint32_t aux);
#endif
