"""Closed-loop sanity check with a toy 1-D + lateral kinematic model.

NOT a substitute for CARLA results — only proves the logic is stable
(no collision, no oscillation, returns to lane) before testing in the simulator.
"""

from avoidance.collision_avoidance import AvoidanceConfig, CollisionAvoidance, Mode, Perception
import math

DT = 0.05
MAX_ACCEL = 3.0
LATERAL_RATE = 6.0  # m/s lateral per unit steer at cruise (toy model)
CAR_HALF_WIDTH = 1.0


def test_standstill_pullout_with_heading_dynamics():
    from avoidance.collision_avoidance import AvoidanceState

    cfg = AvoidanceConfig()
    ctrl = CollisionAvoidance(cfg)
    ctrl.state = AvoidanceState(mode=Mode.STOPPED)
    x = y = speed = heading = 0.0
    minimum = float("inf")
    modes = []
    for _ in range(2400):
        gap = 11.0 - x if x < 11.0 and abs(y) < cfg.lane_width / 2 else None
        cmd = ctrl.step(Perception(0.0, gap, speed, True, y, x, heading))
        modes.append(cmd.mode)
        speed = max(0.0, speed + (cmd.throttle * MAX_ACCEL - cmd.brake * cfg.max_decel) * DT)
        heading += speed / 2.8 * math.tan(cmd.steer * 0.7) * DT
        x += speed * math.cos(heading) * DT
        y += speed * math.sin(heading) * DT
        if abs(y) < cfg.lane_width / 2 and x < 11.0:
            minimum = min(minimum, 11.0 - x)
        if Mode.RETURN in modes and cmd.mode == Mode.CRUISE:
            break
    assert minimum >= 0.8
    assert {Mode.AVOID, Mode.PASSING, Mode.RETURN} <= set(modes)
    assert modes[-1] == Mode.CRUISE and x > 11.0 and abs(y) < 0.3


def simulate(obstacle_x, adjacent_clear, steps=1200, v0=8.0):
    cfg = AvoidanceConfig()
    ctrl = CollisionAvoidance(cfg)
    x, y, v = 0.0, 0.0, v0
    min_gap, modes = float("inf"), []
    for _ in range(steps):
        in_path = obstacle_x is not None and abs(y) < cfg.lane_width / 2 and obstacle_x > x
        dist = (obstacle_x - x) if in_path else None
        cmd = ctrl.step(Perception(0.0, dist, v, adjacent_clear, y, x))
        modes.append(cmd.mode)
        accel = cmd.throttle * MAX_ACCEL - cmd.brake * cfg.max_decel
        v = max(0.0, v + accel * DT)
        x += v * DT
        y += cmd.steer * LATERAL_RATE * DT
        if obstacle_x is not None and abs(y) < CAR_HALF_WIDTH * 2:
            min_gap = min(min_gap, obstacle_x - x if x < obstacle_x else float("inf"))
    return x, y, v, min_gap, modes


def test_empty_lane_cruises_whole_time():
    _, y, v, _, modes = simulate(None, False)
    assert set(modes) == {Mode.CRUISE}
    assert abs(y) < 1e-9 and v > 0


def test_obstacle_stop_halts_before_obstacle_with_margin():
    x, _, v, gap, modes = simulate(60.0, False)
    assert v == 0.0
    assert 60.0 - x >= 2.0, f"stopped too close: {60.0 - x:.2f} m"
    assert Mode.STOPPED in modes and Mode.AVOID not in modes


def test_obstacle_avoid_passes_and_returns_to_lane():
    x, y, v, _, modes = simulate(60.0, True)
    assert x > 60.0, "never passed obstacle"
    assert abs(y) < 0.5, "did not return to original lane"
    assert v > 0
    for m in (Mode.AVOID, Mode.PASSING, Mode.RETURN):
        assert m in modes
    assert modes[-1] == Mode.CRUISE


def simulate_lead(brakes: bool = False) -> tuple[float, float, list[Mode]]:
    cfg = AvoidanceConfig()
    ctrl = CollisionAvoidance(cfg)
    x, lead, speed = 0.0, 35.0, 4.0
    minimum, modes = float("inf"), []
    for _ in range(1600):
        lead_speed = 0.0 if brakes and lead >= 75.0 else 4.0
        gap = lead - x
        cmd = ctrl.step(Perception(0.0, gap, speed, not brakes,
                                  obstacle_speed=lead_speed, distance_travelled=x, dt=DT))
        modes.append(cmd.mode)
        # Feed-forward balances rolling/air resistance at the lead speed.
        drag = cfg.throttle_feedforward * MAX_ACCEL if speed > 0 else 0.0
        speed = max(0.0, speed + (cmd.throttle * MAX_ACCEL -
                                cmd.brake * cfg.max_decel - drag) * DT)
        x += speed * DT
        lead += lead_speed * DT
        minimum = min(minimum, lead - x)
        if not brakes:
            assert lead - x >= cfg.standstill_gap + cfg.time_gap * speed - 0.05
    return minimum, speed, modes


def test_constant_lead_keeps_time_gap_without_overtaking():
    gap, speed, modes = simulate_lead()
    assert gap >= 2 and abs(speed - 4.0) < 0.1
    assert set(modes) == {Mode.CRUISE, Mode.FOLLOW}
    first_follow = modes.index(Mode.FOLLOW)
    assert all(mode == Mode.CRUISE for mode in modes[:first_follow])
    assert all(mode == Mode.FOLLOW for mode in modes[first_follow:])


def test_braking_lead_stops_safely():
    gap, speed, modes = simulate_lead(brakes=True)
    assert gap >= 2 and speed == 0.0
    assert Mode.FOLLOW in modes and Mode.STOPPED in modes and Mode.AVOID not in modes


def test_oncoming_car_passes_before_overtake_starts():
    from dataclasses import replace
    from perception.tracking import lane_free_for_manoeuvre

    cfg = replace(AvoidanceConfig(), safety_margin=6.0)
    ctrl = CollisionAvoidance(cfg)
    x, y, speed, other = 0.0, 0.0, 8.0, 105.0
    modes = []
    for _ in range(1400):
        other_gap = other - x - 4.0
        visible = other_gap if other - x > -15.0 else None
        clear = (visible is None or other - x > 12.0) and lane_free_for_manoeuvre(
            visible, speed + 7.0, cfg.manoeuvre_horizon, cfg.manoeuvre_margin)
        distance = 50.0 - x if x < 50.0 and abs(y) < cfg.lane_width / 2 else None
        cmd = ctrl.step(Perception(0.0, distance, speed, clear, y, x, obstacle_speed=0.0))
        if cmd.mode == Mode.AVOID and Mode.AVOID not in modes:
            assert other < x - 15.0
            assert Mode.STOPPED in modes or Mode.BRAKE in modes
        modes.append(cmd.mode)
        speed = max(0.0, speed + (cmd.throttle * MAX_ACCEL - cmd.brake * cfg.max_decel) * DT)
        x += speed * DT
        y += cmd.steer * LATERAL_RATE * DT
        other -= 7.0 * DT
    assert {Mode.AVOID, Mode.PASSING, Mode.RETURN} <= set(modes)
    assert x > 75.0 and abs(y) < 0.5 and modes[-1] == Mode.CRUISE
