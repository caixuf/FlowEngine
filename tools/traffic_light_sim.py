#!/usr/bin/env python3
"""红绿灯相位与黄灯停车判据的确定性仿真门禁。"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Phase:
    state: str
    remain_s: float


def phase_at(t: float, green_s: float, flashing_green_s: float,
             yellow_s: float, red_s: float) -> Phase:
    cycle = green_s + flashing_green_s + yellow_s + red_s
    p = t % cycle
    boundaries = (
        ("green", green_s),
        ("flashing_green", green_s + flashing_green_s),
        ("yellow", green_s + flashing_green_s + yellow_s),
        ("red", cycle),
    )
    for state, end in boundaries:
        if p < end:
            return Phase(state, end - p)
    raise AssertionError("unreachable")


def should_stop_on_yellow(speed_mps: float, distance_m: float,
                          comfortable_decel: float = 4.0,
                          reaction_s: float = 0.8,
                          margin_m: float = 2.0) -> bool:
    stop_distance = speed_mps * reaction_s
    stop_distance += speed_mps * speed_mps / (2.0 * comfortable_decel)
    stop_distance += margin_m
    return distance_m >= stop_distance


def run_all() -> None:
    sequence = [phase_at(t, 20, 3, 3, 15).state for t in (0, 19.9, 20, 22.9, 23, 25.9, 26)]
    assert sequence == [
        "green", "green", "flashing_green", "flashing_green",
        "yellow", "yellow", "red",
    ]
    assert phase_at(20, 20, 3, 3, 15).remain_s == 3
    assert should_stop_on_yellow(10, 25)
    assert not should_stop_on_yellow(10, 8)
    print("traffic light simulation: PASS")


if __name__ == "__main__":
    run_all()
