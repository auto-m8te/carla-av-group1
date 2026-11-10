import pytest

from avoidance.collision_avoidance import (
    AvoidanceConfig,
    AvoidanceState,
    CollisionAvoidance,
    Mode,
    Perception,
    decide,
)

CFG = AvoidanceConfig()


def perceive(**kw) -> Perception:
    base = dict(lane_steer=0.05, obstacle_distance=None, speed=8.0, adjacent_lane_clear=False)
    base.update(kw)
    return Perception(**base)


def test_stopping_distance_grows_with_speed():
    assert CFG.stopping_distance(0) == pytest.approx(CFG.safety_margin)
    assert CFG.stopping_distance(10) > CFG.stopping_distance(5)


def test_clear_path_cruises_and_follows_lane_steer():
    _, cmd = decide(AvoidanceState(), perceive(), CFG)
    assert cmd.mode == Mode.CRUISE
    assert cmd.steer == pytest.approx(0.05)
    assert cmd.brake == 0.0 and cmd.throttle > 0


def test_far_obstacle_eases_throttle_but_keeps_cruising():
    _, clear = decide(AvoidanceState(), perceive(), CFG)
    _, cmd = decide(AvoidanceState(), perceive(obstacle_distance=80.0), CFG)
    assert cmd.mode == Mode.CRUISE
    assert cmd.throttle < clear.throttle


def test_speed_controller_accelerates_when_slow_and_coasts_when_fast():
    _, slow = decide(AvoidanceState(), perceive(speed=2.0), CFG)
    _, fast = decide(AvoidanceState(), perceive(speed=CFG.target_speed + 3), CFG)
    assert slow.throttle > CFG.throttle_feedforward
    assert fast.throttle == 0.0


def test_blocked_adjacent_lane_brakes():
    _, cmd = decide(AvoidanceState(), perceive(obstacle_distance=12.0), CFG)
    assert cmd.mode == Mode.BRAKE
    assert cmd.throttle == 0.0 and cmd.brake >= 0.3


def test_stopped_vehicle_holds_instead_of_creeping_forward():
    state = AvoidanceState(mode=Mode.STOPPED)
    new, cmd = decide(state, perceive(obstacle_distance=8.0, speed=0.0), CFG)
    assert cmd.mode == Mode.STOPPED and cmd.brake == 1.0 and cmd.throttle == 0.0
    assert new.mode == Mode.STOPPED


def test_stopped_vehicle_resumes_when_obstacle_removed():
    _, cmd = decide(AvoidanceState(mode=Mode.STOPPED), perceive(speed=0.0), CFG)
    assert cmd.mode == Mode.CRUISE and cmd.throttle > 0


def test_stopped_vehicle_steers_around_once_adjacent_lane_clears():
    new, _ = decide(
        AvoidanceState(mode=Mode.STOPPED),
        perceive(obstacle_distance=8.0, speed=0.0, adjacent_lane_clear=True),
        CFG,
    )
    assert new.mode == Mode.AVOID


def test_emergency_distance_full_brake_even_if_lane_clear():
    _, cmd = decide(AvoidanceState(), perceive(obstacle_distance=3.0, adjacent_lane_clear=True), CFG)
    assert cmd.mode == Mode.EMERGENCY_STOP and cmd.brake == 1.0


def test_clear_adjacent_lane_starts_avoidance_and_steers_to_avoid_side():
    state, cmd = decide(AvoidanceState(), perceive(obstacle_distance=15.0, adjacent_lane_clear=True), CFG)
    assert state.mode == Mode.AVOID
    assert state.obstacle_odometer == pytest.approx(15.0)
    assert cmd.steer < 0  # avoid_side = -1 -> steer left


def test_avoid_transitions_to_passing_when_in_target_lane():
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=20.0)
    new, cmd = decide(state, perceive(lateral_offset=-3.4, obstacle_distance=6.0, adjacent_lane_clear=True), CFG)
    assert new.mode == Mode.PASSING and cmd.brake == 0.0


def test_passing_holds_lane_until_obstacle_cleared_then_returns():
    state = AvoidanceState(mode=Mode.PASSING, obstacle_odometer=20.0)
    held, _ = decide(state, perceive(lateral_offset=-3.5, distance_travelled=22.0), CFG)
    assert held.mode == Mode.PASSING
    ret, cmd = decide(state, perceive(lateral_offset=-3.5, distance_travelled=27.0), CFG)
    assert ret.mode == Mode.RETURN and cmd.steer > 0  # steer back right


def test_avoid_aborts_to_brake_if_adjacent_lane_becomes_blocked_early():
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=20.0)
    new, cmd = decide(state, perceive(lateral_offset=-0.5, obstacle_distance=12.0), CFG)
    assert new.mode == Mode.BRAKE and cmd.throttle == 0.0


def test_passing_brakes_for_new_obstacle_ahead_in_passing_lane():
    state = AvoidanceState(mode=Mode.PASSING, obstacle_odometer=20.0)
    _, cmd = decide(state, perceive(lateral_offset=-3.5, obstacle_distance=3.0, speed=6.0), CFG)
    assert cmd.mode == Mode.EMERGENCY_STOP and cmd.brake == 1.0


def test_passing_ignores_emergency_trigger_from_side_obstacle():
    state = AvoidanceState(mode=Mode.PASSING, obstacle_odometer=20.0)
    # negative gap = obstacle is alongside us (already overlapping), not ahead
    new, cmd = decide(state, perceive(lateral_offset=-3.5, obstacle_distance=-1.0, distance_travelled=19.0), CFG)
    assert new.mode == Mode.PASSING and cmd.brake == 0.0


def test_return_completes_back_to_cruise():
    state = AvoidanceState(mode=Mode.RETURN, obstacle_odometer=20.0)
    new, cmd = decide(state, perceive(lateral_offset=0.1), CFG)
    assert new == AvoidanceState() and cmd.mode == Mode.CRUISE


def test_steer_is_clamped_during_lane_change():
    cfg = AvoidanceConfig(lateral_kp=10.0)
    _, cmd = decide(AvoidanceState(), perceive(obstacle_distance=15.0, adjacent_lane_clear=True), cfg)
    assert abs(cmd.steer) <= cfg.max_avoid_steer


def test_heading_term_counter_steers_when_already_angled_toward_target():
    # Car is 1 m short of the target lane but already pointing 20 deg left:
    # it must NOT keep steering left (that is what caused the CARLA overshoot).
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=50.0)
    _, cmd = decide(state, perceive(lateral_offset=-2.5, heading_error=-0.35, adjacent_lane_clear=True), CFG)
    assert cmd.steer > 0


def test_avoid_trigger_distance_scales_with_speed():
    # 25 m ahead: at low speed too early to swerve, at high speed must already start
    slow, _ = decide(AvoidanceState(), perceive(obstacle_distance=25.0, speed=3.0, adjacent_lane_clear=True), CFG)
    fast, _ = decide(AvoidanceState(), perceive(obstacle_distance=25.0, speed=10.0, adjacent_lane_clear=True), CFG)
    assert slow.mode == Mode.CRUISE
    assert fast.mode == Mode.AVOID


def test_mid_lane_change_close_obstacle_keeps_manoeuvring_not_forgetting():
    # Half-way across, obstacle 4 m ahead (partially overlapping): keep avoiding.
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=34.0)
    new, cmd = decide(state, perceive(obstacle_distance=4.0, lateral_offset=-2.0,
                                      adjacent_lane_clear=True, distance_travelled=30.0), CFG)
    assert new.mode == Mode.AVOID and new.obstacle_odometer == 34.0
    assert cmd.steer < 0 and cmd.brake == 0.0


def test_mid_lane_change_very_close_obstacle_still_hard_stops():
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=31.5)
    new, cmd = decide(state, perceive(obstacle_distance=1.5, lateral_offset=-1.0,
                                      adjacent_lane_clear=True, distance_travelled=30.0), CFG)
    assert cmd.brake == 1.0
    assert new.mode == Mode.AVOID  # state kept so it can resume, not reset to CRUISE


def test_committed_lane_change_brakes_if_target_lane_blocked_and_keeps_state():
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=40.0)
    new, cmd = decide(state, perceive(lateral_offset=-2.5, speed=8.0, adjacent_lane_clear=False,
                                      distance_travelled=30.0), CFG)
    assert cmd.throttle == 0.0 and cmd.brake > 0.0
    assert new.mode == Mode.AVOID and new.obstacle_odometer == 40.0


def test_controller_wrapper_tracks_state_and_resets():
    ctrl = CollisionAvoidance()
    ctrl.step(perceive(obstacle_distance=15.0, adjacent_lane_clear=True))
    assert ctrl.state.mode == Mode.AVOID
    ctrl.reset()
    assert ctrl.state == AvoidanceState()
