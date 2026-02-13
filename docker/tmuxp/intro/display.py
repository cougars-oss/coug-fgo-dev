#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 3 Pro), Feb 2025

from asciimatics.effects import Print, Stars, Effect
from asciimatics.renderers import ColourImageFile, FigletText, SpeechBubble
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.exceptions import ResizeScreenError
import os
import sys
import re

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)


class Background(Effect):
    def __init__(self, screen, **kwargs):
        super().__init__(screen, **kwargs)

    def reset(self):
        self._screen.clear_buffer(7, 0, 0)

    def _update(self, frame_no):
        self._screen.clear_buffer(7, 0, 0)

    @property
    def stop_frame(self):
        return self._stop_frame


def display(screen):
    """
    Displays a spinning gif of a wave and the text "COUGARS" in ASCII art.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Feb 2026
    """

    scenes = []
    renderer = ColourImageFile(
        screen, "wave.gif", screen.height, uni=screen.unicode_aware
    )
    renderer._index = len(renderer._images) // 2

    new_images = []
    for image in renderer._images:
        new_image = re.sub(r"\$\{\d+,2,\d+\}\.", " ", image)
        new_images.append(new_image)
    renderer._images = new_images

    effects = [
        Background(screen),
        Print(
            screen,
            FigletText("COUGARS", font="banner3" if screen.width > 80 else "banner"),
            screen.height // 4,
            colour=7,
            bg=7 if screen.unicode_aware else 0,
            speed=1,
            transparent=True,
        ),
        Print(screen, renderer, 0, speed=1, transparent=True),
    ]
    scenes.append(Scene(effects, -1))
    screen.play(scenes, stop_on_resize=True, repeat=False)


if __name__ == "__main__":
    while True:
        try:
            Screen.wrapper(display)
            sys.exit(0)
        except ResizeScreenError:
            pass
