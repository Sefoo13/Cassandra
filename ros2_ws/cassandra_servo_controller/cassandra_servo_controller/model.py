"""Pure conversion and pose data for Cassandra's LX-16A servos."""

from __future__ import annotations

import math


LX16A_RADIANS_PER_UNIT = math.radians(240.0) / 1000.0

# Atomic poses extracted from test_scripts/servo_movement.py. Long behaviors
# were split into atomic commands so callbacks never block the ROS executor.
POSES: dict[str, dict[int, int]] = {
    "base": {
        1: 450,
        2: 730,
        3: 500,
        4: 450,
        5: 400,
        6: 660,
        7: 100,
        8: 470,
        9: 200,
        10: 500,
        11: 550,
        12: 500,
        13: 500,
    },
    "pray_pose": {1: 450, 2: 700, 3: 500, 4: 450},
    "pray": {1: 275, 2: 730, 3: 560, 4: 230, 6: 850, 7: 100, 8: 400, 9: 410},
    "pray_right": {6: 850, 7: 100, 8: 400, 9: 410},
    "pray_left": {1: 275, 2: 730, 3: 560, 4: 230},
    "whole": {1: 250, 2: 500, 3: 300, 4: 200, 6: 800, 7: 300, 8: 685, 9: 400},
    "head_right_down": {10: 650, 11: 650},
    "head_left_up": {10: 350, 11: 350},
    "head_left": {10: 300},
    "head_right": {10: 700},
    "head_up": {11: 300},
    "head_down": {11: 700},
    "head_center": {10: 460, 11: 550},
    "crest": {2: 350, 7: 500},
    "torso_left": {5: 200},
    "torso_center": {5: 400},
    "torso_right": {5: 600},
}


def clamp_raw(position: int, raw_min: int, raw_max: int) -> int:
    """Clamp an LX-16A raw position to configured safe limits."""
    return max(raw_min, min(raw_max, int(position)))


def radians_to_raw(
    radians: float,
    home_raw: int,
    direction: float,
    radians_per_unit: float = LX16A_RADIANS_PER_UNIT,
    raw_min: int = 0,
    raw_max: int = 1000,
) -> int:
    """Convert a joint offset in radians into an LX-16A raw position."""
    if not math.isfinite(radians):
        raise ValueError("Joint position must be finite")
    if direction not in (-1.0, 1.0):
        raise ValueError("Servo direction must be -1 or 1")
    if radians_per_unit <= 0.0:
        raise ValueError("radians_per_unit must be positive")
    raw = round(home_raw + direction * radians / radians_per_unit)
    return clamp_raw(raw, raw_min, raw_max)


def raw_to_radians(
    raw: int,
    home_raw: int,
    direction: float,
    radians_per_unit: float = LX16A_RADIANS_PER_UNIT,
) -> float:
    """Convert an LX-16A raw position into a joint offset in radians."""
    if direction not in (-1.0, 1.0):
        raise ValueError("Servo direction must be -1 or 1")
    return (int(raw) - home_raw) * radians_per_unit * direction
