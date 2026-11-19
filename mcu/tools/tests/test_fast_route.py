#!/usr/bin/env python3
"""Host-side static and mathematical checks for the wheel/laser route.

This does not compile the firmware and does not model steering latency, slip,
laser rejection, actuator saturation, or the fruit-thinning mechanism.
"""

from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import re


MCU = Path(__file__).resolve().parents[2]
ROUTE_SOURCE = (MCU / "src/app/map_route.c").read_text(encoding="utf-8")
CONTROL_SOURCE = (MCU / "src/app/app_odom_test.c").read_text(encoding="utf-8")
ODOM_ASSEMBLY = (MCU / "src/service/assemble/assemble_odom.c").read_text(encoding="utf-8")
ENTRY_SOURCE = (MCU / "src/app/entry.h").read_text(encoding="utf-8")


def numeric_define(source: str, name: str) -> float:
    match = re.search(rf"^#define\s+{name}\s+(-?\d+(?:\.\d+)?)f\s*$", source, re.M)
    assert match, f"missing numeric define {name}"
    return float(match.group(1))


@dataclass(frozen=True)
class Leg:
    x: float
    y: float
    speed: float
    kind: str
    beams: tuple[str, ...]
    end_beam: str
    row: int | None


def parse_route() -> list[Leg]:
    west = numeric_define(ROUTE_SOURCE, "WEST")
    east = numeric_define(ROUTE_SOURCE, "EAST")
    speeds = {
        name: numeric_define(ROUTE_SOURCE, name)
        for name in (
            "ROUTE_START_SPEED_MPS",
            "ROUTE_CROSS_SPEED_MPS",
            "ROUTE_CONNECT_SPEED_MPS",
            "ROUTE_RETURN_SPEED_MPS",
        )
    }
    legs: list[Leg] = []
    invocation = re.compile(
        r"^(CROSS|CONNECT)\((WEST|EAST),\s*(\d+),\s*"
        r"(?:(LEFT_WALL|RIGHT_WALL),\s*)?(LASER_\w+)\),?$"
    )
    for raw_line in ROUTE_SOURCE.splitlines():
        line = raw_line.strip()
        if line.startswith("{ WEST, ROW(7), ROUTE_START_SPEED_MPS"):
            legs.append(Leg(west, 0.2 + 0.7 * 6, speeds["ROUTE_START_SPEED_MPS"],
                            "START", ("LEFT_WALL",), "LASER_FRONT", 7))
            continue
        match = invocation.match(line)
        if match:
            kind, x_name, row_text, wall, end_beam = match.groups()
            row = int(row_text)
            legs.append(Leg(
                west if x_name == "WEST" else east,
                0.2 + 0.7 * (row - 1),
                speeds["ROUTE_CROSS_SPEED_MPS" if kind == "CROSS" else "ROUTE_CONNECT_SPEED_MPS"],
                kind,
                ("FRONT_WALL", "REAR_WALL") if kind == "CROSS" else (wall,),
                end_beam,
                row,
            ))
            continue
        if line.startswith("{ WEST, MAP_START_Y_M, ROUTE_RETURN_SPEED_MPS"):
            legs.append(Leg(west, -0.5, speeds["ROUTE_RETURN_SPEED_MPS"],
                            "RETURN", ("LEFT_WALL",), "LASER_NO_BEAM", None))
    return legs


def check_route(legs: list[Leg]) -> float:
    assert len(legs) == 16, f"expected 16 legs, got {len(legs)}"
    assert legs[-1].beams == ("LEFT_WALL",), "parking leg must use the left beam"
    assert legs[-1].end_beam == "LASER_NO_BEAM"

    previous = (0.2, -0.5)
    total_distance = 0.0
    crossed_rows: Counter[int] = Counter()
    for number, leg in enumerate(legs, 1):
        dx, dy = leg.x - previous[0], leg.y - previous[1]
        assert (dx != 0.0) ^ (dy != 0.0), f"leg {number} is not axis-aligned"
        assert leg.speed > 0.0 and math.isfinite(leg.speed)
        distance = abs(dx) + abs(dy)
        total_distance += distance
        if dx != 0.0:
            assert set(leg.beams) == {"FRONT_WALL", "REAR_WALL"}, f"leg {number} wrong cross beams"
            expected_end = "LASER_RIGHT" if dx > 0.0 else "LASER_LEFT"
            crossed_rows[leg.row] += 1
        else:
            assert len(leg.beams) == 1 and leg.beams[0] in {"LEFT_WALL", "RIGHT_WALL"}, (
                f"leg {number} wrong side beam"
            )
            expected_end = "LASER_FRONT" if dy > 0.0 else "LASER_REAR"
        assert leg.end_beam in {"LASER_NO_BEAM", expected_end}, f"leg {number} wrong end beam"
        previous = (leg.x, leg.y)

    assert crossed_rows == Counter({7: 2, 6: 1, 5: 1, 4: 1, 3: 1, 2: 1, 1: 1}), crossed_rows
    assert previous == (0.2, -0.5), "route must return to the launch-side target"
    return total_distance


def simulate_leg(distance: float, vmax: float, initial: float, end: float,
                 accel: float = 1.0, decel: float = 1.6, dt: float = 0.002) -> tuple[float, float]:
    remaining = distance
    speed = initial
    elapsed = 0.0
    while remaining > 0.005:
        assert elapsed < 120.0, "route model failed to reach an endpoint"
        braking = math.sqrt(end * end + 2.0 * decel * max(remaining, 0.0))
        target = min(vmax, braking)
        previous = speed
        if speed < target:
            speed = min(target, speed + accel * dt)
            assert speed - previous <= accel * dt + 1e-9
        else:
            speed = max(target, speed - decel * dt)
            assert previous - speed <= decel * dt + 1e-9
        assert -1e-9 <= speed <= vmax + 1e-9
        remaining -= speed * dt
        elapsed += dt
    return elapsed, speed


def check_static_control() -> None:
    route_header = (MCU / "src/app/map_route.h").read_text(encoding="utf-8")
    runtime = (MCU / "src/app/app_runtime.c").read_text(encoding="utf-8")
    assert re.search(r"config\.wheel_only\s*=\s*true\s*;", ODOM_ASSEMBLY)
    assert not re.search(r"^\s*assemble_imu\s*\(\s*\)\s*;", ENTRY_SOURCE, re.M)
    assert re.search(r"^#define\s+MAP_ROUTE_LASER_GUIDANCE_ENABLED\s+0$", route_header, re.M)
    assert "laser_odom_is_online()" not in runtime
    assert "get_gyro_corrected" not in CONTROL_SOURCE
    assert "route_laser_anchor" in CONTROL_SOURCE
    assert "odom_relocalize_xy" in CONTROL_SOURCE
    assert "route_update_speed" in CONTROL_SOURCE
    assert "APP_ODOM_TEST_TRANSITION_SPEED_MPS" in CONTROL_SOURCE
    assert "if(motion_scale == 0.0f)" in CONTROL_SOURCE
    assert "const bool wait_laser = use_laser && !laser_odom_is_online();" in CONTROL_SOURCE
    assert "const float drive = (test_vy ? sign : -sign)" in CONTROL_SOURCE
    assert "const float wheel_x = -dx * s_odom_test.heading_sin + dy * s_odom_test.heading_cos;" in CONTROL_SOURCE


def check_body_commands(legs: list[Leg]) -> None:
    """Every map displacement must use the confirmed right-handed body axes."""
    previous = (0.2, -0.5)
    for number, leg in enumerate(legs, 1):
        dx, dy = leg.x - previous[0], leg.y - previous[1]
        if dx:
            expected = "+vy" if dx > 0.0 else "-vy"
        else:
            expected = "-vx" if dy > 0.0 else "+vx"
        leg_command = ("+vy" if dx > 0.0 else "-vy") if dx else ("-vx" if dy > 0.0 else "+vx")
        assert leg_command == expected, f"leg {number} wrong body command"
        previous = (leg.x, leg.y)


def check_anchor_geometry() -> None:
    """A positive cross residual must shift field position by minus residual."""
    for yaw in (-2.4, -0.7, 0.0, 0.9, 2.8):
        cosine, sine = math.cos(yaw), math.sin(yaw)
        for test_vy in (False, True):
            residual = 0.03
            dx = residual * cosine if test_vy else residual * sine
            dy = residual * sine if test_vy else -residual * cosine
            field_x_delta = -dx * sine + dy * cosine
            field_y_delta = -dx * cosine - dy * sine
            field_delta = field_y_delta if test_vy else field_x_delta
            assert math.isclose(field_delta, -residual, abs_tol=1e-6)
            assert math.isclose(-field_delta, residual, abs_tol=1e-6)


def check_transport_configuration() -> None:
    fdcan = (MCU / "Core/Src/fdcan.c").read_text(encoding="utf-8")
    uart = (MCU / "Core/Src/usart.c").read_text(encoding="utf-8")
    ioc = (MCU / "robot.ioc").read_text(encoding="utf-8")
    clock_match = re.search(r"^RCC\.FDCANFreq_Value=(\d+)$", ioc, re.M)
    assert clock_match
    clock = int(clock_match.group(1))
    for bus in (1, 2, 3):
        match = re.search(rf"void MX_FDCAN{bus}_Init\(void\)(.*?)(?=void MX_FDCAN|\Z)", fdcan, re.S)
        assert match
        block = match.group(1)
        prescaler = int(re.search(r"NominalPrescaler = (\d+);", block).group(1))
        seg1 = int(re.search(r"NominalTimeSeg1 = (\d+);", block).group(1))
        seg2 = int(re.search(r"NominalTimeSeg2 = (\d+);", block).group(1))
        assert clock / (prescaler * (1 + seg1 + seg2)) == 1_000_000
        assert math.isclose((1 + seg1) / (1 + seg1 + seg2), 0.70)
    assert re.findall(r"huart(?:\d+)\.Init\.BaudRate = (\d+);", uart) == ["115200"] * 4


def main() -> None:
    check_static_control()
    check_anchor_geometry()
    check_transport_configuration()
    legs = parse_route()
    distance = check_route(legs)
    check_body_commands(legs)
    elapsed = 0.0
    for index, leg in enumerate(legs):
        previous = (0.2, -0.5) if index == 0 else (legs[index - 1].x, legs[index - 1].y)
        leg_distance = abs(leg.x - previous[0]) + abs(leg.y - previous[1])
        initial = 0.0 if index == 0 else 0.20
        end = 0.0 if index + 1 == len(legs) else 0.20
        leg_time, _ = simulate_leg(leg_distance, leg.speed, initial, end)
        elapsed += leg_time
    elapsed += 0.100  # Commanded final settling window; not physical stopping time.
    print(f"PASS: {len(legs)} legs, all 7 furrows covered, route {distance:.3f} m")
    print(f"Idealized route motion + commanded settle: {elapsed:.3f} s")
    print("Scope: host-side source/model checks only; no firmware build or on-car validation")


if __name__ == "__main__":
    main()
