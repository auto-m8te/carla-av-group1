"""Closed-loop sanity check with a toy 1-D + lateral kinematic model.

NOT a substitute for CARLA results — only proves the logic is stable
(no collision, no oscillation, returns to lane) before testing in the simulator.
"""

from avoidance.collision_avoidance import AvoidanceConfig, CollisionAvoidance, Mode, Perception

DT = 0.05
MAX_ACCEL = 3.0
LATERAL_RATE = 6.0  # m/s lateral per unit steer at cruise (toy model)
CAR_HALF_WIDTH = 1.0


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
