#!/usr/bin/env python3
"""usb_cmds.py -- SINGLE SOURCE OF TRUTH for the emCORE USB debug command numbers (nano 4G bundle).

Must match the C enum in emCORE/emCORE/usb/usbdebug.c. Host code (libemcore.py, n4g_nand.py) imports Cmd from
here instead of hardcoding numbers.

PROTOCOL OVERVIEW
=================
The usbdebug dispatcher routes a command number to ONE of two switch blocks:
  * the SYNC switch (usbdebug_handle_data, DBGSTATE_SETUP), handled inline in USB IRQ context;
  * the ASYNC switch (dbgthread), the stock emCORE monitor commands.
The SYNC switch is matched FIRST; any number not in it falls through to the ASYNC switch, so sync numbers must not
collide with stock async numbers. Every reply is a 16-byte header (word 0 = response code 1 OK / 2 UNSUPPORTED /
3 BUSY, words 1..3 results) plus an optional payload (READ_MEM, READ_CONSOLE, I2C read).

NAND PROOF OF CONCEPT (2026-10-17)
========================================================
Commands 120..127 drive target/ipodnano4g/nand_poc.c. Each only queues a job for the NAND worker thread and
returns at once (reply word 1 = accept code, word 2 = job id, word 3 = status record address); NAND_STATUS
returns {status record address, size, result buffer address}. The host polls the status record and reads the
result buffer with READ_MEM. Heartbeat (async 200) words 2/3 = NAND state word / FIL page counter.
"""

from enum import IntEnum


class Cmd(IntEnum):
    """Canonical emCORE USB debug command numbers (sync + referenced async)."""

    # ---- Core protocol primitives (stock numbers, never renumber) ----
    GETINFO    = 1    # sub-cases: 0=VERSION, 1=USER MEM / pool bounds
    READ_MEM   = 4
    WRITE_MEM  = 5

    # ---- Console / scheduler (sync) ----
    READ_CONSOLE  = 10
    WRITE_CONSOLE = 11
    GETPROCINFO   = 15
    FREEZE        = 16
    SUSPEND       = 17
    KILL          = 18
    CREATETHREAD  = 19
    FLUSHCACHE    = 20

    WEDGE_FIX = 68    # set wedge-fix bitmask (buf[1]=new mask, reply word 1 = old); EP0 tolerance fixes
    RESET     = 104   # sync RESET (buf[1]=0 resets at once), moved from 2 (collided with async 2 RESET)

    # ---- nandlib proof of concept (nand_poc.c) ----
    NAND_STATUS        = 120   # -> {&status, status size, &result buffer}
    NAND_STEP          = 121   # (step 1..5, arg): 1 gates, 2 pins (arg 1 = stock table), 3 PMU power, 4 FIL, 5 FTL
    NAND_READ_RESERVED = 122   # (page, reset_first): ROM-style FMC read of bank 0 page -> 0x600 bytes
    NAND_READ_PAGE     = 123   # (bank, page, flags bit0 = uncached variant): FIL[1] read -> 4096 data + 12 meta + maxcorr + secstat
    NAND_FTL_OPEN      = 124   # = NAND_STEP 5 (WMR_Dispatch 0x70, then disk_mount(0))
    NAND_FTL_READ      = 125   # (lpn, count <= 64): WMR_Dispatch 0x65 in chunks of 16 -> count*4096 bytes
    NAND_GET_STRUCT = 126 # (id, input word): WMR_Dispatch 0x78 (ids ) -> length in job_detail[0]
    NAND_WATCHDOG = 127 # (seconds, 0 = disarm): stub, reply word 1 = 4 "unknown register semantics" 
    # ---- P2b write path (NAND_POC_WRITE build only; read-only builds answer accept code 6 NO_WRITE) ----
    NAND_WRITE_INFO    = 128   # -> {flags, staging address, staging bytes}
    NAND_SNAPSHOT      = 129   # reserved region + contexts into the result buffer; required before any destructive job
    NAND_OPEN_RW       = 130   # 130..136: word 3 = token = 0x4e414e44 ^ (job<<24) ^ arg0 ^ ror32(arg1 by 8... i.e. (arg1<<8)|(arg1>>24))
    NAND_ERASE_BLOCK   = 131   # (bank, block | 0x80000000 force)
    NAND_PROGRAM_PAGE  = 132   # (bank, page | force); data+meta from the staging buffer
    NAND_FTL_WRITE     = 133   # (lpn, count <= 16); data from the staging buffer
    NAND_FTL_FLUSH     = 134
    NAND_BOOTAREA_WR   = 135   # (page, force required): one 0x600-byte boot page from the staging buffer
    NAND_STORAGE_WR    = 136   # (enable)

    # ---- Stock async monitor commands referenced from the host (NOT sync) ----
    RESET_ASYNC     = 2     # async RESET; sync RESET = 104
    POWEROFF_ASYNC  = 3
    HEARTBEAT_ASYNC = 200   # word1 heartbeat, word2 NAND state word, word3 FIL pages read


# Sync numbers (all unique, none in the stock async set).
SYNC_CMDS = (
    Cmd.GETINFO, Cmd.READ_MEM, Cmd.WRITE_MEM,
    Cmd.READ_CONSOLE, Cmd.WRITE_CONSOLE,
    Cmd.GETPROCINFO, Cmd.FREEZE, Cmd.SUSPEND, Cmd.KILL, Cmd.CREATETHREAD, Cmd.FLUSHCACHE,
    Cmd.WEDGE_FIX, Cmd.RESET,
    Cmd.NAND_STATUS, Cmd.NAND_STEP, Cmd.NAND_READ_RESERVED, Cmd.NAND_READ_PAGE,
    Cmd.NAND_FTL_OPEN, Cmd.NAND_FTL_READ, Cmd.NAND_GET_STRUCT, Cmd.NAND_WATCHDOG,
    Cmd.NAND_WRITE_INFO, Cmd.NAND_SNAPSHOT, Cmd.NAND_OPEN_RW, Cmd.NAND_ERASE_BLOCK, Cmd.NAND_PROGRAM_PAGE,
    Cmd.NAND_FTL_WRITE, Cmd.NAND_FTL_FLUSH, Cmd.NAND_BOOTAREA_WR, Cmd.NAND_STORAGE_WR,
)

# NEVER REUSE: numbers of the removed (non-working) NAND code. Old host scripts or old images may still send or
# answer them; giving them a new meaning would make a stale tool drive the new firmware with wrong arguments.
RETIRED = {
    6: "NAND_M2_READ",
    62: "APPLE_READ / NRO_*", 63: "NRO_*", 64: "NRO_*", 65: "NRO_*", 66: "NRO_*", 67: "NRO_*",
    69: "FTL_L2P_*", 70: "FTL_L2P_*", 71: "FTL_L2P_*", 72: "FTL_L2P_*",
    73: "PMAP_*", 74: "PMAP_*", 75: "PMAP_*", 76: "PMAP_*", 77: "PMAP_*", 78: "PMAP_*", 79: "PMAP_*",
    82: "PMAP_*",
    83: "FMSS_*", 84: "FMSS_*", 85: "FMSS_*",
    86: "FTL_LIB_REPORT", 87: "FTL_LIB_RESOLVE", 88: "MEM_VERIFY",
    89: "NAND_BRINGUP", 90: "RESERVED_READ", 91: "M2_FTLRESTORE", 92: "M2_READ_PAGE", 93: "M2_KICK_WORD",
    94: "NRO_OS_SCAN", 95: "NRO_POSTREAD_MODE", 96: "ECC_CTX_UPLOAD", 97: "PROGA_READ / NRO_DONESUB_MODE",
    98: "FTL_LIB_INIT", 99: "FTL_LIB_RESTORE",
    100: "raw NAND read", 101: "raw NAND read", 102: "raw NAND read", 103: "raw NAND read",
    105: "NRO_SETTLE_MODE", 106: "NRO_INITDETECT_MODE",
}

# Stock async numbers (dbgthread switch) that sync numbers must not shadow.
STOCK_ASYNC = frozenset([2, 3, 8, 9, 12, 13, 14] + list(range(21, 62)) + [200])

assert not set(int(c) for c in SYNC_CMDS) & set(RETIRED), "a sync command reuses a retired number"
assert not set(int(c) for c in SYNC_CMDS) & STOCK_ASYNC, "a sync command shadows a stock async command"
