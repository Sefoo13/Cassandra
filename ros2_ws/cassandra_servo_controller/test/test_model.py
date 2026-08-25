import math

import pytest

from cassandra_servo_controller.model import clamp_raw
from cassandra_servo_controller.model import radians_to_raw, raw_to_radians


def test_zero_radians_is_home_position():
    assert radians_to_raw(0.0, 450, 1.0) == 450


def test_round_trip_conversion():
    radians = 0.25
    raw = radians_to_raw(radians, 500, -1.0)
    assert raw_to_radians(raw, 500, -1.0) == pytest.approx(
        radians, abs=math.radians(0.25)
    )


def test_conversion_honors_direction():
    assert radians_to_raw(0.1, 500, 1.0) > 500
    assert radians_to_raw(0.1, 500, -1.0) < 500


def test_raw_position_is_clamped():
    assert clamp_raw(-20, 0, 1000) == 0
    assert clamp_raw(1200, 0, 1000) == 1000


def test_non_finite_position_is_rejected():
    with pytest.raises(ValueError):
        radians_to_raw(float("nan"), 500, 1.0)
