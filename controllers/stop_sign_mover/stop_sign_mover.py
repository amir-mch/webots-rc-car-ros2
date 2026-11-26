#!/usr/bin/env python3
"""Webots Supervisor controller that smoothly moves the start Stop Sign off the track after 4.5s."""

import math
import time
from typing import Optional
from controller import Supervisor

START_POS = [-7.8962, 6.3122, 0.0]
END_POS = [-8.4290, 13.2920, 0.0]
HOLD_SEC = 4.5
MOVE_SEC = 1.2


def main() -> None:
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())
    self_node = robot.getSelf()
    trans_field = self_node.getField("translation") if self_node is not None else None

    if trans_field is not None:
        trans_field.setSFVec3f(list(START_POS))

    start_wall_t: Optional[float] = None
    done = False
    while robot.step(time_step) != -1:
        if trans_field is None or done:
            continue
        now = time.monotonic()
        if start_wall_t is None:
            start_wall_t = now
        t = now - start_wall_t
        if t < HOLD_SEC:
            pos = list(START_POS)
        elif t <= HOLD_SEC + MOVE_SEC:
            frac = (t - HOLD_SEC) / MOVE_SEC
            u = 0.5 * (1.0 - math.cos(math.pi * frac))
            pos = [s + (e - s) * u for s, e in zip(START_POS, END_POS)]
        else:
            pos = list(END_POS)
            done = True
        trans_field.setSFVec3f(pos)


if __name__ == "__main__":
    main()
