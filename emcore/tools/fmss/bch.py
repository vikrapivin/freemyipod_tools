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

"""Encoder of the nano 3G ECC engine's code (binary BCH, 8 bits over GF(2^13), generator below): builds chunks the
engine should accept, and checks chunks read from the flash. Message = 512 data bytes + 3 metadata bytes, MSB first;
chunk = [3 metadata bytes][13 parity bytes, big-endian]."""
POLY = 0x115f914e07b0c138741c5c4fb23
NPAR = 104
_MASK = (1 << NPAR) - 1
_TAB = []
for _b in range(256):
    _v = _b << NPAR
    for _i in range(NPAR + 7, NPAR - 1, -1):
        if _v >> _i & 1:
            _v ^= POLY << (_i - NPAR)
    _TAB.append(_v)


def parity(data, meta3):
    r = 0
    for b in bytes(data) + bytes(meta3):
        r = ((r << 8) & _MASK) ^ _TAB[(r >> (NPAR - 8)) ^ b]
    return r.to_bytes(13, "big")


def chunk(data, meta3):
    return bytes(meta3) + parity(data, meta3)


def is_codeword(data, chunk16):
    return parity(data, chunk16[:3]) == bytes(chunk16[3:16])
