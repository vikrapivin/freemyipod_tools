# Copyright (C) 2026 Vikotr Krapivin
#
# This file is part of emCORE.
#
# emCORE is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# emCORE is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with emCORE. If not, see <http://www.gnu.org/licenses/>.

"""Nano3g monitor commands. Device definitions live in emcore/target/ipodnano3g/nand3g.h."""
from enum import IntEnum

class Cmd(IntEnum):
    GETINFO = 1
    READ_MEM = 4
    WRITE_MEM = 5
    READ_CONSOLE = 10
    WRITE_CONSOLE = 11
    RESET = 2
    NAND_STATUS = 120
    NAND_STEP = 121
    NAND_READ_PAGE = 123
    NAND_FTL_OPEN = 124
    NAND_FTL_READ = 125
    NAND_WRITE_CAPS = 128
    NAND_WRITE_OPEN = 129
    NAND_BLOCK_WRITE = 130
    NAND_FLUSH = 131
    NAND_BACKEND = 135
    HEARTBEAT_ASYNC = 200
