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

"""Registry of the experiments: name, function, the capabilities of the target it needs."""
from dataclasses import dataclass

RESULTS = {}                  # results of the experiments run in this process (optime reads opsweep's)
REGISTRY = {}


@dataclass
class Experiment:
    name: str
    fn: object                # fn(s, target) -> (data, summary)
    needs: tuple              # capability names (Target.has_<name>) the target must have

    @property
    def doc(self):
        return self.fn.__doc__

    def missing(self, target):
        return [c for c in self.needs if not target.has(c)]

    def bind(self, target):
        """fn(s) bound to a target (the form the tool had before it was split)"""
        def run(s):
            return self.fn(s, target)
        run.__doc__, run.__name__ = self.fn.__doc__, self.fn.__name__
        return run


def experiment(name, needs=()):
    def register(fn):
        if name in REGISTRY:
            raise ValueError("experiment %s registered twice" % name)
        REGISTRY[name] = Experiment(name, fn, tuple(needs))
        return fn
    return register
