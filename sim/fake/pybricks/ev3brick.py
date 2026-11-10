from pybricks._world import WORLD
from pybricks.parameters import Button


class _Sound:
    def beep(self, *args, **kwargs):
        pass

    def file(self, name):
        WORLD.advance(1000)  # the real sound file blocks for about 1 s


sound = _Sound()


def buttons():
    # CENTER is pressed once, at WORLD.press_at, for 150 ms. DOWN never.
    if WORLD.press_at <= WORLD.t < WORLD.press_at + 150:
        return [Button.CENTER]
    return []
