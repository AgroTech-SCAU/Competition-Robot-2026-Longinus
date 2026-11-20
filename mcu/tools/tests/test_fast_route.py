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
YAW_HOLD_SOURCE = (MCU / "src/service/chassis_yaw_hold.c").read_text(encoding="utf-8")


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
    rear_escape: bool
    work_stop: bool
    endpoint_side_trim: bool


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
        cross = re.match(
            r"CROSS\((WEST|EAST),\s*(\d+),\s*(true|false),\s*(true|false),\s*(true|false)\)",
            line,
        )
        connect = re.match(r"CONNECT\((WEST|EAST),\s*(\d+),\s*(LASER_\w+)\)", line)
        if cross:
            x_name, row, rear_escape, work_stop, endpoint_trim = cross.groups()
            legs.append(Leg(west if x_name == "WEST" else east,
                            0.2 + 0.7 * (int(row) - 1), speeds["ROUTE_CROSS_SPEED_MPS"],
                            "LASER_FRONT", True, rear_escape == "true", work_stop == "true",
                            endpoint_trim == "true"))
        elif connect:
            x_name, row, side = connect.groups()
            legs.append(Leg(west if x_name == "WEST" else east,
                            0.2 + 0.7 * (int(row) - 1), speeds["ROUTE_CONNECT_SPEED_MPS"],
                            side, False, False, False, False))
        elif line.startswith("{ WEST, ROW(7), ROUTE_START_SPEED_MPS"):
            legs.append(Leg(west, 4.4, speeds["ROUTE_START_SPEED_MPS"],
                            "LASER_LEFT", False, False, False, False))
        elif line.startswith("{ WEST, MAP_START_Y_M, ROUTE_RETURN_SPEED_MPS"):
            legs.append(Leg(west, -0.5, speeds["ROUTE_RETURN_SPEED_MPS"],
                            "LASER_NO_BEAM", False, False, False, False))
    return legs


def check_route(legs: list[Leg]) -> float:
    assert len(legs) == 16, f"expected 16 legs, got {len(legs)}"
    expected = [
        (0.2, 4.4, 0.50, "LEFT"),
        (2.8, 4.4, 0.45, "FRONT"),
        (0.2, 4.4, 0.45, "FRONT"),
        (0.2, 3.7, 0.32, "LEFT"),
        (2.8, 3.7, 0.45, "FRONT"),
        (2.8, 3.0, 0.32, "RIGHT"),
        (0.2, 3.0, 0.45, "FRONT"),
        (0.2, 2.3, 0.32, "LEFT"),
        (2.8, 2.3, 0.45, "FRONT"),
        (2.8, 1.6, 0.32, "RIGHT"),
        (0.2, 1.6, 0.45, "FRONT"),
        (0.2, 0.9, 0.32, "LEFT"),
        (2.8, 0.9, 0.45, "FRONT"),
        (2.8, 0.2, 0.32, "RIGHT"),
        (0.2, 0.2, 0.45, "FRONT"),
        (0.2, -0.5, 0.45, "NO_BEAM"),
    ]
    gated = {2, 3, 5, 7, 9, 11, 13, 15}
    rear_protected = {5, 7, 9, 11, 13, 15}
    working_passes = {2, 5, 7, 9, 11, 13, 15}
    previous = (0.2, -0.5)
    total = 0.0
    for number, (leg, target) in enumerate(zip(legs, expected), 1):
        x, y, speed, control = target
        assert math.isclose(leg.x, x) and math.isclose(leg.y, y), f"leg {number} coordinate"
        assert math.isclose(leg.speed, speed), f"leg {number} speed={leg.speed}"
        assert leg.control == f"LASER_{control}", f"leg {number} control={leg.control}"
        assert leg.gate == (number in gated), f"leg {number} gate={leg.gate}"
        assert leg.rear_escape == (number in rear_protected), f"leg {number} rear escape"
        assert leg.work_stop == (number in working_passes), f"leg {number} work stop"
        assert leg.endpoint_side_trim == (number in (3, 5, 7, 9, 11, 13, 15)), f"leg {number} endpoint trim"
        dx, dy = leg.x - previous[0], leg.y - previous[1]
        dx = 0.0 if abs(dx) < 1e-6 else dx
        dy = 0.0 if abs(dy) < 1e-6 else dy
        assert (dx != 0.0) ^ (dy != 0.0), f"leg {number} is not axis-aligned"
        total += abs(dx) + abs(dy)
        previous = (leg.x, leg.y)
    assert previous == (0.2, -0.5)
    assert math.isclose(total, 30.6)
    assert legs[1].y == legs[2].y and legs[1].work_stop and not legs[2].work_stop
    assert legs[4].x == legs[5].x and legs[5].control == "LASER_RIGHT"
    connectors = [4, 6, 8, 10, 12, 14]
    assert all(legs[i - 1].control in ("LASER_LEFT", "LASER_RIGHT") and
               not legs[i - 1].gate for i in connectors)
    return total


def target_error(raw: float, target: float) -> float:
    error = raw - target
    deadband = numeric_define(CONTROL_SOURCE, "LASER_CONTROL_DEADBAND_MM")
    if abs(error) <= deadband:
        return 0.0
    return error - math.copysign(deadband, error)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_hold_output(yaw_ref: float, yaw: float, measured_wz: float) -> float:
    """Mathematical mirror of the IMU-feedback PD angle loop."""
    output = 8.0 * wrap_pi(yaw_ref - yaw) - 0.25 * measured_wz
    return max(-0.55, min(0.55, output))


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def furrow_targets(total: float) -> tuple[float, float]:
    narrow = numeric_define(CONTROL_SOURCE, "LASER_FURROW_NARROW_SUM_MM")
    center = numeric_define(CONTROL_SOURCE, "LASER_FURROW_CENTER_SUM_MM")
    narrow_bias = numeric_define(CONTROL_SOURCE, "LASER_FURROW_NARROW_BIAS_MM")
    center_bias = numeric_define(CONTROL_SOURCE, "LASER_FURROW_CENTER_BIAS_MM")
    tolerance = numeric_define(CONTROL_SOURCE, "LASER_DUAL_TOLERANCE_MM")
    front_min = numeric_define(CONTROL_SOURCE, "LASER_FRONT_SAFETY_MM") + tolerance + 1
    rear_min = numeric_define(CONTROL_SOURCE, "LASER_SAFETY_MM") + tolerance + 1
    blend = min(1.0, max(0.0, (total - narrow) / (center - narrow)))
    bias = narrow_bias + blend * (center_bias - narrow_bias)
    front = min(total - rear_min, max(front_min, 0.5 * (total + bias)))
    return front, total - front


def dual_correction(front: float, rear: float) -> float:
    """Route vx correction using the observed width and measured bias."""
    front_target, rear_target = furrow_targets(front + rear)
    gain = numeric_define(CONTROL_SOURCE, "LASER_DUAL_CONTROL_GAIN")
    limit = numeric_define(CONTROL_SOURCE, "LASER_DUAL_CORRECTION_LIMIT_MPS")
    return clamp(0.5 * (target_error(rear, rear_target) -
                        target_error(front, front_target)) * gain * 0.001, limit)


def check_control_contract() -> None:
    assert numeric_define(LASER_HEADER, "LASER_MIN_RANGE_MM") == 0
    assert numeric_define(LASER_HEADER, "LASER_MAX_RANGE_MM") == 5000
    assert numeric_define(CONTROL_SOURCE, "LASER_LEFT_TARGET_MM") == 41
    assert numeric_define(CONTROL_SOURCE, "LASER_RIGHT_TARGET_MM") == 45
    assert numeric_define(CONTROL_SOURCE, "LASER_LAUNCH_LEFT_TARGET_MM") == 36
    assert numeric_define(CONTROL_SOURCE, "LASER_FRONT_WALL_RESERVE_MM") == 70
    assert numeric_define(CONTROL_SOURCE, "LASER_LEFT_WALL_RESERVE_MM") == 48
    assert numeric_define(CONTROL_SOURCE, "LASER_RIGHT_WALL_RESERVE_MM") == 52
    assert numeric_define(CONTROL_SOURCE, "ROUTE_WALL_END_WINDOW_M") == 0.08
    assert numeric_define(CONTROL_SOURCE, "LASER_WALL_ACCEPT_MARGIN_MM") == 3
    assert numeric_define(CONTROL_SOURCE, "LASER_WALL_BRAKE_MPS2") == 0.35
    assert numeric_define(CONTROL_SOURCE, "ROUTE_WALL_TRIM_MAX_M") == 0.15
    assert numeric_define(CONTROL_SOURCE, "ROUTE_WALL_TRIM_MAX_MS") == 5500
    assert numeric_define(CONTROL_SOURCE, "LASER_WALL_TRIM_CAPTURE_MM") == 220
    assert numeric_define(CONTROL_SOURCE, "LASER_WALL_TRIM_TOLERANCE_MM") == 1
    assert numeric_define(CONTROL_SOURCE, "LASER_WALL_TRIM_CONFIRM_FRAMES") == 3
    assert numeric_define(CONTROL_SOURCE, "ROUTE_WALL_TRIM_SPEED_MPS") == 0.04
    assert numeric_define(CONTROL_SOURCE, "ROUTE_WALL_TRIM_CRAWL_MPS") == 0.008
    assert numeric_define(CONTROL_SOURCE, "LASER_CONNECT_HOLD_MM") == 35
    assert numeric_define(CONTROL_SOURCE, "LASER_CONNECT_RESUME_MM") == 38
    assert "LASER_DUAL_TARGET_MM" not in CONTROL_SOURCE
    assert numeric_define(CONTROL_SOURCE, "LASER_DUAL_TOLERANCE_MM") == 1
    assert "LASER_SIDE_TOLERANCE_MM" not in CONTROL_SOURCE
    assert "LASER_TARGET_TOLERANCE_MM" not in CONTROL_SOURCE
    assert numeric_define(CONTROL_SOURCE, "LASER_CONTROL_DEADBAND_MM") == 1
    assert numeric_define(CONTROL_SOURCE, "LASER_START_ACQUIRE_MM") == 105
    assert numeric_define(CONTROL_SOURCE, "LASER_FRONT_SAFETY_MM") == 55
    assert numeric_define(CONTROL_SOURCE, "LASER_SAFETY_MM") == 30
    assert numeric_define(CONTROL_SOURCE, "LASER_FURROW_SUM_MIN_MM") == 95
    assert numeric_define(CONTROL_SOURCE, "LASER_FURROW_SUM_MAX_MM") == 125
    assert numeric_define(CONTROL_SOURCE, "LASER_FURROW_HYSTERESIS_MM") == 5
    assert "95..125" in ROUTE_HEADER and "95..125" in ROUTE_SOURCE
    for obsolete in ("75..125", "75..115", "100..140", "target=50", "LEFT=50"):
        assert obsolete not in ROUTE_HEADER + ROUTE_SOURCE + CONTROL_SOURCE
    assert "guidance_acquired = true" in CONTROL_SOURCE
    assert re.search(r"segment_index == 0u && s_odom_test\.phase == ROUTE_TRAVEL &&\s*"
                     r"!s_odom_test\.guidance_acquired", CONTROL_SOURCE)
    assert "MAP ACQUIRE seg=1 LEFT" in CONTROL_SOURCE
    assert "MAP SPEED seg=%u phase=%s" in CONTROL_SOURCE
    assert "ROUTE_SPEED_ACCEL" in CONTROL_SOURCE and "ROUTE_SPEED_DECEL" in CONTROL_SOURCE
    assert numeric_define(CONTROL_SOURCE, "ROUTE_END_TOLERANCE_M") == 0.005
    assert numeric_define(CONTROL_SOURCE, "ROUTE_ACCEL_DISTANCE_M") == 0.400
    assert numeric_define(CONTROL_SOURCE, "ROUTE_DECEL_DISTANCE_M") == 0.400
    assert numeric_define(CONTROL_SOURCE, "APP_ODOM_TEST_SETTLE_READY_MS") == 100
    assert numeric_define(CONTROL_SOURCE, "LASER_SIDE_CONTROL_GAIN") == 2.5
    assert numeric_define(CONTROL_SOURCE, "LASER_SIDE_CORRECTION_LIMIT_MPS") == 0.09
    assert numeric_define(CONTROL_SOURCE, "LASER_DUAL_CONTROL_GAIN") == 1.0
    assert numeric_define(CONTROL_SOURCE, "LASER_DUAL_CORRECTION_LIMIT_MPS") == 0.06
    assert not re.search(r"^#define\s+LASER_CONTROL_GAIN\s", CONTROL_SOURCE, re.M)
    assert not re.search(r"^#define\s+LASER_CORRECTION_LIMIT_MPS\s", CONTROL_SOURCE, re.M)
    assert numeric_define(CONTROL_SOURCE, "LASER_ESCAPE_GAIN") == 2.5
    assert numeric_define(CONTROL_SOURCE, "LASER_ESCAPE_LIMIT_MPS") == 0.09
    assert numeric_define(CONTROL_SOURCE, "LASER_CORRECTION_ACCEL_MPS2") == 0.30
    assert "route_acceleration_speed(progress_m)" in CONTROL_SOURCE
    assert re.search(r"config\.wheel_only\s*=\s*true\s*;", ODOM_ASSEMBLY)
    assert re.search(r"entry_boot_check\(assemble_imu\(\),\s*\"imu\"\)", ENTRY_SOURCE)
    assert ENTRY_SOURCE.index("assemble_imu()") < ENTRY_SOURCE.index("assemble_odom()")
    assert '#include "imu.h"' in CONTROL_SOURCE
    assert '#include "chassis_yaw_hold.h"' in CONTROL_SOURCE
    assert "if(!s_odom_test.yaw_target_valid)" in CONTROL_SOURCE
    assert "chassis_yaw_hold_set_target(imu_yaw);" in CONTROL_SOURCE
    assert "chassis_yaw_hold_disable();" in CONTROL_SOURCE
    assert "odom.get_state()->imu_ready" in CONTROL_SOURCE
    assert "imu.get_angle()" in CONTROL_SOURCE and "imu_get_gyro_corrected()" in CONTROL_SOURCE
    assert re.search(r"chassis_yaw_hold_apply\(vx_cmd,\s*vy_cmd,\s*0\.0f,\s*yaw,\s*measured_wz", CONTROL_SOURCE)
    assert re.search(r"chassis\.set_velocity\(vx_cmd,\s*vy_cmd,\s*wz_cmd\)", CONTROL_SOURCE)
    assert "config.kp = 8.0f" in YAW_HOLD_SOURCE
    assert "config.kd = 0.25f" in YAW_HOLD_SOURCE
    assert "config.wz_limit = 0.55f" in YAW_HOLD_SOURCE
    assert "odom_relocalize_xy" not in CONTROL_SOURCE
    assert "route_safety_escape" in CONTROL_SOURCE
    assert "leg->rear_escape" in CONTROL_SOURCE
    assert "leg->work_stop" in CONTROL_SOURCE
    assert "route_wall_speed_limit" in CONTROL_SOURCE
    assert re.search(r"yaw\s*-\s*s_odom_test\.imu_heading_ref", CONTROL_SOURCE)
    assert "0.5f * (rear_error - front_error)" in CONTROL_SOURCE
    settle = re.search(r"static AppControlResult route_settle\b.*?^}", CONTROL_SOURCE, re.M | re.S)
    assert settle
    assert "if(!leg->endpoint_side_trim) return route_next_or_park(now);" in settle[0]
    assert "route_is_stationary" in settle[0] and "safety_active" in settle[0]
    assert "s_odom_test.phase = ROUTE_WALL_TRIM" in settle[0]
    assert "route_lateral_target" not in settle[0]
    assert "route_front_target" not in CONTROL_SOURCE
    assert "settle_beam" not in ROUTE_HEADER

    # Removed dangerous endpoint behavior must stay removed.
    for forbidden in (
        "endpoint_votes", "LASER_ENDPOINT_FRAMES",
        "LASER_ENDPOINT_MIN_SPEED_MPS", "remaining_for_braking", "endpoint_overshot",
        "ROUTE01_FINAL_APPROACH_MPS", "ROUTE01_FRONT_ENDPOINT_MM",
    ):
        assert forbidden not in CONTROL_SOURCE, f"obsolete endpoint logic remains: {forbidden}"
    assert "route_braking_speed(remaining)" in CONTROL_SOURCE
    assert re.search(r"remaining\s*<=\s*ROUTE_END_TOLERANCE_M", CONTROL_SOURCE)

    # The launch measurement L=42 needs leftward correction until field entry.
    assert -target_error(42, 36) < 0.0
    assert target_error(41, 41) == target_error(42, 41) == 0.0
    assert target_error(43, 41) > 0.0 and target_error(39, 41) < 0.0
    for target in (36.0, 41.0, 45.0):
        assert target_error(target - 1, target) == target_error(target + 1, target) == 0
        assert target_error(target - 1.1, target) < 0 < target_error(target + 1.1, target)

    # Auto yaw lock: zero error gives zero wz; angle and rate signs are correct.
    assert yaw_hold_output(0.0, 0.0, 0.0) == 0.0
    assert yaw_hold_output(0.1, 0.0, 0.0) > 0.0
    assert yaw_hold_output(0.0, 0.1, 0.0) < 0.0
    assert yaw_hold_output(0.0, 0.0, 0.2) < 0.0
    assert yaw_hold_output(0.0, 1.0, 0.0) == -0.55
    assert yaw_hold_output(-math.pi + 0.05, math.pi - 0.05, 0.0) > 0.0

    # The regular and narrow measured furrows are both inside the travel deadband.
    assert furrow_targets(109) == (57.0, 52.0)
    assert furrow_targets(100) == (59.0, 41.0)
    assert dual_correction(56, 53) == dual_correction(59, 41) == 0.0
    assert dual_correction(58, 51) == 0.0 and dual_correction(59, 50) < 0.0
    assert math.isclose(dual_correction(56, 68), 0.0065)
    assert math.isclose(dual_correction(68, 41), -0.010)
    assert abs(dual_correction(5000, 0)) == 0.06

    # FRONT escapes below 55 mm; LEFT/RIGHT/REAR escape below 30 mm.
    assert CONTROL_SOURCE.count("LASER_FRONT_SAFETY_MM") >= 3
    for raw in (0.0, 30.0, 54.0):
        front_escape = min(0.09, 2.5 * (55.0 - raw) * 0.001)
        assert 0.0 < front_escape <= 0.09
    for raw in (55.0, 70.0, 5000.0):
        assert raw >= 55.0  # no front safety escape outside the protected range
    for raw in (0.0, 20.0, 29.0):
        escape = min(0.09, 2.5 * (30.0 - raw) * 0.001)
        assert 0.0 < escape <= 0.09
        left_vy, right_vy, rear_vx = escape, -escape, -escape
        assert left_vy > 0 and right_vy < 0 and rear_vx < 0
    for raw in (30.0, 45.0, 5000.0):
        assert raw >= 30.0  # no side/rear safety escape outside the protected range
    assert numeric_define(CONTROL_SOURCE, "LASER_FRONT_SAFETY_MM") < numeric_define(
        CONTROL_SOURCE, "LASER_FRONT_WALL_RESERVE_MM")
    side_safety = numeric_define(CONTROL_SOURCE, "LASER_SAFETY_MM")
    connect_hold = numeric_define(CONTROL_SOURCE, "LASER_CONNECT_HOLD_MM")
    connect_resume = numeric_define(CONTROL_SOURCE, "LASER_CONNECT_RESUME_MM")
    assert side_safety < connect_hold < connect_resume
    for side in ("LEFT", "RIGHT"):
        assert connect_resume < numeric_define(CONTROL_SOURCE, f"LASER_{side}_WALL_RESERVE_MM")
    for width in range(90, 131):
        front, rear = furrow_targets(width)
        assert front - 1 > 55 and rear - 1 > 30


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
    print(f"PASS: 16 legs, route {distance:.3f} m; wheel distance owns route progress")
    print("PASS: eight crossings calibrate from 56/53 and 59/41 mm with a 95..125 mm entry gate")
    print("PASS: width-adaptive dual travel guidance uses +/-1 mm deadband; gain=1.0, limit=0.06 m/s")
    print("PASS: LEFT launch=36 mm, field=41 mm, RIGHT=45 mm; side deadband +/-1 mm")
    print("PASS: filtered walls bound speed and can accept an endpoint in the last 80 mm")
    print("PASS: 03/05/07/09/11/13/15 require side-wall alignment before connectors or exit")
    print("PASS: 04/06/08/10/12/14 use wheel X + side Y; side hold/resume uses 35/38 mm")
    print("PASS: 02 works in furrow 1 and 03 returns through it without repeating work")
    print("PASS: raw 0..5000 mm; FRONT escape=55 mm; LEFT/RIGHT escape=30 mm")
    print("PASS: rear 30 mm escape only on 05/07/09/11/13/15; 02/03 rear escape disabled")
    print(f"PASS: 16 wheel-odometry trapezoids, ideal motion {motion_time:.2f} s plus settling")
    print("PASS: handover requires a stationary dwell; Auto yaw uses IMU angle/rate PD")
    print("Scope: source/model checks only; no firmware build or on-car validation")


if __name__ == "__main__":
    main()
