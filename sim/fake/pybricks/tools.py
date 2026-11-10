import builtins
from pybricks._world import WORLD

print = builtins.print


def wait(ms):
    WORLD.advance(ms)


class StopWatch:
    def __init__(self):
        self.start = WORLD.t

    def time(self):
        return int(WORLD.t - self.start)

    def reset(self):
        self.start = WORLD.t
