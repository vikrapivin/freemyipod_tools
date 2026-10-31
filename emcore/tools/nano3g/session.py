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

"""Shared nano3g USB session (libemcore transport); commands execute once."""
import struct
import time

class ToolError(Exception):
    pass

class Session:
    def __init__(self, settle=0.05, poll=0.05, ep0=True):
        import libemcore
        self.settle, self.poll = settle, poll
        self.em = libemcore.Emcore()
        if ep0:
            self.em.lib.bulkin = False

    def close(self):
        self.em.close()

    def cmd(self, command, a=0, b=0, d=0):
        return self.em.lib.monitorcommand(struct.pack("<IIII", command, a, b, d), "III")[1:]

    def read_chunk(self, addr, size):
        return self.em.read(addr, size)

    def write_memory(self, addr, data):
        return self.em.write(addr, bytes(data))

    sleep = staticmethod(time.sleep)
    now = staticmethod(time.monotonic)
