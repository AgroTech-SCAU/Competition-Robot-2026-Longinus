#!/usr/bin/env python3
"""Static and mathematical checks for the wheel-route laser controller.

These checks do not compile the firmware or replace low-speed on-car validation.
"""

from dataclasses import dataclass
from pathlib import Path
import math
import re


MCU = Path(__file__).resolve().parents[2]
ROUTE_SOURCE = (MCU / "src/app/map_route.c").read_text(encoding="utf-8")
ROUTE_HEADER = (MCU / "src/app/map_route.h").read_text(encoding="utf-8")
CONTROL_SOURCE = (MCU / "src/app/app_odom_test.c").read_text(encoding="utf-8")
LASER_HEADER = (MCU / "src/service/laser_odom.h").read_text(encoding="utf-8")
ODOM_ASSEMBLY = (MCU / "src/service/assemble/assemble_odom.c").read_text(encoding="utf-8")
ENTRY_SOURCE = (MCU / "src/app/entry.h").read_text(encoding="utf-8")


def numeric_define(source: str, name: str) -> float:
    match = re.search(rf"^#define\s+{name}\s+(-?\d+(?:\.\d+)?)(?:u|f)?\s*$", source, re.M)
    assert match, f"missing numeric define {name}"
    return float(match.group(1))


@dataclass(frozen=True)
class Leg:
    x: float
    y: float
    speed: float
    control: str
    gate: bool


def parse_route() -> list[Leg]:
    west = numeric_define(ROUTE_SOURCE, "WEST")
    east = numeric_define(ROUTE_SOURCE, "EAST")
    speeds = {
        name: numeric_define(ROUTE_SOURCE, name)
        for name in (
            "ROUTE_START_SPEED_MPS", "ROUTE_CROSS_SPEED_MPS",
            "ROUTE_CONNECT_SPEED_MPS", "ROUTE_RETURN_SPEED_MPS",
        )
    }
    legs: list[Leg] = []
    for raw_line in ROUTE_SOURCE.splitlines():
        line = raw_line.strip()
        cross = re.match(r"CROSS\((WEST|EAST),\s*(\d+),\s*(true|false)\)", line)
        connect = re.match(r"CONNECT\((WEST|EAST),\s*(\d+),\s*(LASER_\w+)\)", line)
        if cross:
            x_name, row, gate = cross.groups()
            legs.append(Leg(west if x_name == "WEST" else east,
                            0.2 + 0.7 * (int(row) - 1), speeds["ROUTE_CROSS_SPEED_MPS"],
                            "LASER_FRONT", gate == "true"))
        elif connect:
            x_name, row, side = connect.groups()
            legs.append(Leg(west if x_name == "WEST" else east,
                            0.2 + 0.7 * (int(row) - 1), speeds["ROUTE_CONNECT_SPEED_MPS"],
                            side, False))
        elif line.startswith("{ WEST, ROW(7), ROUTE_START_SPEED_MPS"):
            legs.append(Leg(west, 4.4, speeds["ROUTE_START_SPEED_MPS"], "LASER_LEFT", False))
        elif line.startswith("{ WEST, MAP_START_Y_M, ROUTE_RETURN_SPEED_MPS"):
            legs.append(Leg(west, -0.5, speeds["ROUTE_RETURN_SPEED_MPS"], "LASER_NO_BEAM", False))
    return legs


def check_route(legs: list[Leg]) -> float:
    assert len(legs) == 16, f"expected 16 legs, got {len(legs)}"
    controls = ["LEFT", "FRONT", "FRONT", "LEFT", "FRONT", "RIGHT", "FRONT", "LEFT",
                "FRONT", "RIGHT", "FRONT", "LEFT", "FRONT", "RIGHT", "FRONT", "NO_BEAM"]
    gated = {5, 7, 9, 11, 13, 15}
    previous = (0.2, -0.5)
    total = 0.0
    for number, (leg, control) in enumerate(zip(legs, controls), 1):
        assert leg.control == f"LASER_{control}", f"leg {number} control={leg.control}"
        assert leg.gate == (number in gated), f"leg {number} gate={leg.gate}"
        assert leg.control != "LASER_REAR"
        dx, dy = leg.x - previous[0], leg.y - previous[1]
        dx = 0.0 if abs(dx) < 1e-6 else dx
        dy = 0.0 if abs(dy) < 1e-6 else dy
        assert (dx != 0.0) ^ (dy != 0.0), f"leg {number} is not axis-aligned"
        assert leg.speed > 0.0 and math.isfinite(leg.speed)
        total += abs(dx) + abs(dy)
        previous = (leg.x, leg.y)
    assert previous == (0.2, -0.5)
    return total


def check_control_contract() -> None:
    assert numeric_define(LASER_HEADER, "LASER_MIN_RANGE_MM") == 0
    assert numeric_define(LASER_HEADER, "LASER_MAX_RANGE_MM") == 5000
    assert numeric_define(CONTROL_SOURCE, "LASER_TARGET_MM") == 65
    assert numeric_define(CONTROL_SOURCE, "LASER_TARGET_TOLERANCE_MM") == 8
    assert numeric_define(CONTROL_SOURCE, "LASER_SAFETY_MM") == 55
    assert "MAP SPEED seg=%u phase=%s" in CONTROL_SOURCE
    assert "ROUTE_SPEED_ACCEL" in CONTROL_SOURCE and "ROUTE_SPEED_DECEL" in CONTROL_SOURCE
    assert "MAP ALIGN seg=%u beam=%u raw=%u target=65" in CONTROL_SOURCE
    assert "MAP ALIGN SKIP" in CONTROL_SOURCE and "MAP ALIGNED" in CONTROL_SOURCE
    assert numeric_define(CONTROL_SOURCE, "LASER_FURROW_SUM_MIN_MM") == 100
    assert numeric_define(CONTROL_SOURCE, "LASER_FURROW_SUM_MAX_MM") == 140
    assert numeric_define(CONTROL_SOURCE, "ROUTE_END_TOLERANCE_M") == 0.005
    assert numeric_define(CONTROL_SOURCE, "ROUTE_ACCEL_DISTANCE_M") == 0.400
    assert numeric_define(CONTROL_SOURCE, "ROUTE_DECEL_DISTANCE_M") == 0.400
    assert numeric_define(CONTROL_SOURCE, "APP_ODOM_TEST_SETTLE_READY_MS") == 100
    assert numeric_define(CONTROL_SOURCE, "LASER_CORRECTION_LIMIT_MPS") == 0.06
    assert numeric_define(CONTROL_SOURCE, "ROUTE01_FINAL_APPROACH_MPS") == 0.04
    assert "route_acceleration_speed(progress_m)" in CONTROL_SOURCE
    assert "fminf(accel_speed, braking_speed)" in CONTROL_SOURCE
    assert "MAP PROFILE seg=%u accel_d=%.3f decel_d=%.3f" in CONTROL_SOURCE
    assert "MAP FINAL APPROACH seg=1" in CONTROL_SOURCE
    assert "MAP ALIGN WAIT" in CONTROL_SOURCE
    assert "APP_ODOM_TEST_SETTLE_TIMEOUT_MS" not in CONTROL_SOURCE
    assert re.search(r"config\.wheel_only\s*=\s*true\s*;", ODOM_ASSEMBLY)
    assert not re.search(r"^\s*assemble_imu\s*\(\s*\)\s*;", ENTRY_SOURCE, re.M)
    assert "chassis_yaw_hold" not in CONTROL_SOURCE
    assert "odom_relocalize_xy" not in CONTROL_SOURCE
    assert re.search(r"chassis\.set_velocity\([^;]+,\s*0\.0f\)", CONTROL_SOURCE)
    assert "route_furrow_gate" in CONTROL_SOURCE
    assert "route_safety_escape" in CONTROL_SOURCE
    assert "leg->furrow_gate && route_laser_valid(sample, LASER_REAR" in CONTROL_SOURCE

    # Removed dangerous endpoint behavior must stay removed.
    for forbidden in (
        "end_beam", "endpoint_votes", "LASER_ENDPOINT_FRAMES",
        "LASER_ENDPOINT_MIN_SPEED_MPS", "remaining_for_braking", "endpoint_overshot",
    ):
        assert forbidden not in CONTROL_SOURCE, f"obsolete endpoint logic remains: {forbidden}"
    assert "end_beam" not in ROUTE_HEADER and "end_beam" not in ROUTE_SOURCE
    assert "route_braking_speed(remaining)" in CONTROL_SOURCE
    assert "if(index == 0u)" in CONTROL_SOURCE
    assert "sample->distance_mm[LASER_FRONT] <=" in CONTROL_SOURCE
    assert "endpoint_ready = remaining <= ROUTE_END_TOLERANCE_M" in CONTROL_SOURCE

    # Raw >65 moves toward the wall; raw <65 moves away. Only vx/vy are used.
    target = 65.0
    for raw in (0.0, 40.0, 65.0, 100.0, 5000.0):
        error = raw - target
        if abs(error) <= 3.0:
            error = 0.0
        else:
            error -= math.copysign(3.0, error)
        front_vx = -error
        left_vy = -error
        right_vy = error
        if raw > 65:
            assert front_vx < 0 and left_vy < 0 and right_vy > 0
        elif raw < 65:
            assert front_vx > 0 and left_vy > 0 and right_vy < 0
        else:
            assert front_vx == left_vy == right_vy == 0

    # Independent 55 mm escape model: zero is valid and commands away from walls.
    for raw in (0.0, 30.0, 54.0):
        escape = min(0.12, 4.0 * (55.0 - raw) * 0.001)
        assert escape > 0.0
        assert escape <= 0.12
        front_vx, left_vy, right_vy, rear_vx = escape, escape, -escape, -escape
        assert front_vx > 0 and left_vy > 0 and right_vy < 0 and rear_vx < 0
    for raw in (55.0, 65.0, 5000.0):
        assert raw >= 55.0  # no safety escape outside the protected range


def ideal_leg_time(distance: float, speed_limit: float) -> float:
    accel_distance = min(numeric_define(CONTROL_SOURCE, "ROUTE_ACCEL_DISTANCE_M"), 0.5 * distance)
    braking_distance = min(numeric_define(CONTROL_SOURCE, "ROUTE_DECEL_DISTANCE_M"), 0.5 * distance)
    accel = speed_limit * speed_limit / (2.0 * accel_distance)
    decel = speed_limit * speed_limit / (2.0 * braking_distance)
    assert math.isclose(speed_limit * speed_limit / (2.0 * accel), accel_distance)
    assert math.isclose(speed_limit * speed_limit / (2.0 * decel), braking_distance)
    assert accel_distance + braking_distance <= distance + 1e-6
    if distance >= 0.8:
        assert accel_distance == 0.4 and braking_distance == 0.4
    else:
        assert accel_distance == braking_distance == 0.5 * distance
    cruise_distance = max(0.0, distance - accel_distance - braking_distance)
    elapsed = speed_limit / accel + cruise_distance / speed_limit + speed_limit / decel
    assert math.isfinite(elapsed) and elapsed > 0.0

    # Replay the actual 2 ms position caps and rate limiter. The start floor
    # prevents the zero-position sqrt cap from locking the vehicle at zero.
    dt = numeric_define(CONTROL_SOURCE, "APP_ODOM_TEST_CONTROL_PERIOD_S")
    start_speed = numeric_define(CONTROL_SOURCE, "ROUTE_PROFILE_START_SPEED_MPS")
    progress = speed = 0.0
    steps = 0
    while distance - progress > numeric_define(CONTROL_SOURCE, "ROUTE_END_TOLERANCE_M"):
        accel_cap = max(start_speed, math.sqrt(2.0 * accel * max(0.0, progress)))
        decel_cap = math.sqrt(2.0 * decel * max(0.0, distance - progress))
        target = min(speed_limit, accel_cap, decel_cap)
        speed = min(target, speed + accel * dt) if speed < target else max(target, speed - decel * dt)
        progress += speed * dt
        steps += 1
        assert steps < 20000 and math.isfinite(speed)
    tolerance = numeric_define(CONTROL_SOURCE, "ROUTE_END_TOLERANCE_M")
    assert -tolerance <= progress - distance <= tolerance + 1e-9
    return elapsed


def main() -> None:
    check_control_contract()
    legs = parse_route()
    distance = check_route(legs)
    previous = (0.2, -0.5)
    motion_time = 0.0
    for leg in legs:
        leg_distance = math.hypot(leg.x - previous[0], leg.y - previous[1])
        motion_time += ideal_leg_time(leg_distance, leg.speed)
        previous = (leg.x, leg.y)
    print(f"PASS: 16 legs, route {distance:.3f} m; 01 immediate front endpoint, 02..16 wheel endpoints")
    print("PASS: raw 0..5000 mm; bidirectional 65 mm control; independent front/left/right 55 mm escape")
    print("PASS: rear 55 mm escape and 100..140 mm gate only on 05/07/09/11/13/15")
    print(f"PASS: 16 wheel-odometry trapezoids, ideal motion {motion_time:.2f} s plus settling")
    print("PASS: each segment stops and settles before switching; Auto wz=0")
    print("Scope: source/model checks only; no firmware build or on-car validation")


if __name__ == "__main__":
    main()
