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


def test_adjacent_clear_hold_default():
    assert CFG.adjacent_clear_hold == 1.0


@pytest.mark.parametrize("mode,target,offset", [(Mode.AVOID, -3.5, 0.0), (Mode.ABORT, 0.0, -2.5)])
@pytest.mark.parametrize("speed,scale,limit", [(1.0, 0.35, 0.7), (3.0, 0.35, 0.7), (4.0, 0.675, 0.55), (5.0, 1.0, 0.4), (8.0, 1.0, 0.4)])
def test_pullout_heading_gain_and_clamp(mode, target, offset, speed, scale, limit):
    cfg = AvoidanceConfig()
    state = AvoidanceState(mode=mode, obstacle_odometer=50.0)
    for heading in (-0.3, 2.0):
        _, cmd = decide(state, perceive(speed=speed, lateral_offset=offset,
                                       heading_error=heading, adjacent_lane_clear=True), cfg)
        expected = cfg.lateral_kp * (target - offset) - cfg.heading_kd * scale * heading
        assert cmd.steer == pytest.approx(max(-limit, min(limit, expected)))


@pytest.mark.parametrize("side", [-1, 1])
def test_stopped_avoid_creeps_with_full_pullout_steer(side):
    cfg = AvoidanceConfig(avoid_side=side)
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=20.0)
    new, cmd = decide(state, perceive(obstacle_distance=1.6, speed=0.0,
                                    adjacent_lane_clear=True), cfg)
    assert new == state and cmd.mode == Mode.AVOID
    assert cmd.brake == 0.0 and cmd.steer == side * 0.7
    assert cmd.throttle == pytest.approx(cfg.throttle_feedforward + cfg.speed_kp * 1.0)


@pytest.mark.parametrize("gap,clear,offset,speed", [
    (0.7, True, 0.0, 0.0), (0.8, True, 0.0, 0.0),
    (1.6, False, 0.0, 0.0), (1.6, True, -3.6, 0.0), (1.6, True, 0.0, 2.0)])
def test_creep_requires_safe_gap_clear_lane_slow_and_not_yet_over(gap, clear, offset, speed):
    _, cmd = decide(AvoidanceState(mode=Mode.AVOID), perceive(
        obstacle_distance=gap, adjacent_lane_clear=clear, lateral_offset=offset, speed=speed), CFG)
    assert cmd.brake == 1.0 and cmd.throttle == 0.0


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


@pytest.mark.parametrize("mode", [Mode.CRUISE, Mode.BRAKE, Mode.STOPPED])
def test_moving_lead_follows_and_never_starts_avoid(mode):
    state, cmd = decide(AvoidanceState(mode=mode, moving_elapsed=CFG.moving_confirm_time), perceive(
        obstacle_distance=20.0, obstacle_speed=4.0, adjacent_lane_clear=True, distance_travelled=3.0), CFG)
    assert state.mode == cmd.mode == Mode.FOLLOW


def test_follow_brakes_for_speed_error_and_ttc():
    for gap in (6.0, 20.0):
        _, cmd = decide(AvoidanceState(moving_elapsed=CFG.moving_confirm_time), perceive(
            obstacle_distance=gap, obstacle_speed=2.0, distance_travelled=3.0), CFG)
        assert cmd.mode == Mode.FOLLOW and cmd.brake > 0 and cmd.throttle == 0


def test_follow_emergency_disappearance_and_stationary_fallback():
    state = AvoidanceState(mode=Mode.FOLLOW)
    _, cmd = decide(state, perceive(obstacle_distance=3.0, obstacle_speed=4.0), CFG)
    assert cmd.mode == Mode.EMERGENCY_STOP
    _, cmd = decide(state, perceive(), CFG)
    assert cmd.mode == Mode.CRUISE
    _, cmd = decide(state, perceive(obstacle_distance=12.0, obstacle_speed=1.0,
                                   adjacent_lane_clear=True), CFG)
    assert cmd.mode == Mode.AVOID


@pytest.mark.parametrize("side", [-1, 1])
def test_committed_blocked_lane_aborts_home_with_room(side):
    cfg = AvoidanceConfig(avoid_side=side)
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=52.4)
    new, cmd = decide(state, perceive(lateral_offset=side * 2.5,
                                     distance_travelled=30.0), cfg)
    assert new.mode == cmd.mode == Mode.ABORT
    assert cmd.steer * side < 0
    assert cmd.brake > 0 and cmd.throttle == 0


@pytest.mark.parametrize("speed, expected", [(0.0, Mode.STOPPED), (8.0, Mode.BRAKE)])
def test_abort_reaching_home_resets_to_normal_stop(speed, expected):
    state = AvoidanceState(mode=Mode.ABORT, obstacle_odometer=42.4)
    new, cmd = decide(state, perceive(lateral_offset=0.1, speed=speed,
                                     distance_travelled=30.0), CFG)
    assert new.mode == cmd.mode == expected
    assert new.obstacle_odometer is None
    assert cmd.brake > 0 and cmd.throttle == 0
    retry, _ = decide(new, perceive(obstacle_distance=10.0, speed=0.0,
                                   adjacent_lane_clear=True), CFG)
    assert retry.mode == Mode.AVOID


def test_committed_blocked_lane_does_not_abort_alongside_obstacle():
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=35.4)
    new, cmd = decide(state, perceive(lateral_offset=-2.5,
                                     distance_travelled=30.0), CFG)
    assert new.mode == Mode.AVOID
    assert cmd.steer < 0 and cmd.brake > 0


def test_abort_emergency_stop_has_priority():
    state = AvoidanceState(mode=Mode.ABORT, obstacle_odometer=52.4)
    _, cmd = decide(state, perceive(lateral_offset=-2.5, obstacle_distance=1.0), CFG)
    assert cmd.mode == Mode.EMERGENCY_STOP and cmd.brake == 1.0


@pytest.mark.parametrize("mode", [Mode.STOPPED, Mode.AVOID])
def test_pull_out_caps_target_from_standstill(mode):
    cfg = AvoidanceConfig(max_throttle=1.0)
    _, cmd = decide(AvoidanceState(mode=mode), perceive(
        obstacle_distance=11.0, speed=0.0, adjacent_lane_clear=True), cfg)
    target = (11.0 - cfg.safety_margin) / cfg.lane_change_time
    assert cmd.mode == Mode.AVOID
    assert cmd.throttle == pytest.approx(cfg.throttle_feedforward + cfg.speed_kp * target)


def test_pull_out_brakes_gently_above_cap():
    _, cmd = decide(AvoidanceState(mode=Mode.AVOID), perceive(
        obstacle_distance=11.0, speed=4.0, adjacent_lane_clear=True), CFG)
    assert cmd.mode == Mode.AVOID
    assert cmd.throttle == 0.0
    assert cmd.brake == 0.3


@pytest.mark.parametrize("gap,offset", [(40.0, 0.0), (None, 0.0), (11.0, -1.75)])
def test_pull_out_preserves_cruise_when_cap_not_needed(gap, offset):
    _, cmd = decide(AvoidanceState(mode=Mode.AVOID), perceive(
        obstacle_distance=gap, lateral_offset=offset, speed=8.0,
        adjacent_lane_clear=True), CFG)
    assert cmd.throttle == pytest.approx(CFG.throttle_feedforward + CFG.speed_kp * (CFG.target_speed - 8))
    assert cmd.brake == 0.0


def test_creep_also_recovers_a_pullout_stuck_halfway_over():
    # CARLA Town01 oncoming_overtake: stopped 2.1 m over, 1.2 m behind the obstacle.
    cfg = AvoidanceConfig(lane_width=4.0)
    state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=20.0)
    _, cmd = decide(state, perceive(obstacle_distance=1.2, speed=0.0, adjacent_lane_clear=True,
                                    lateral_offset=-2.1), cfg)
    assert cmd.mode == Mode.AVOID and cmd.brake == 0.0 and cmd.throttle > 0.0


def test_spawn_motion_blip_never_follows():
    ctrl = CollisionAvoidance()
    for _ in range(5):
        assert ctrl.step(perceive(obstacle_distance=60.0, obstacle_speed=1.6,
                                 distance_travelled=0.5)).mode == Mode.CRUISE


def test_lead_requires_continuous_confirmation_and_odometer():
    ctrl = CollisionAvoidance()
    for _ in range(9):
        assert ctrl.step(perceive(obstacle_distance=60.0, obstacle_speed=4.0,
                                 distance_travelled=4.0)).mode == Mode.CRUISE
    for _ in range(3):
        cmd = ctrl.step(perceive(obstacle_distance=60.0, obstacle_speed=4.0,
                                distance_travelled=4.0))
    assert cmd.mode == Mode.FOLLOW
    ctrl.reset()
    for _ in range(20):
        assert ctrl.step(perceive(obstacle_distance=60.0, obstacle_speed=4.0,
                                 distance_travelled=2.9)).mode != Mode.FOLLOW


@pytest.mark.parametrize("gap,speed", [(None, 4.0), (60.0, None), (60.0, 1.0)])
def test_motion_confirmation_resets(gap, speed):
    ctrl = CollisionAvoidance()
    for _ in range(8):
        ctrl.step(perceive(obstacle_distance=60.0, obstacle_speed=4.0, distance_travelled=4.0))
    ctrl.step(perceive(obstacle_distance=gap, obstacle_speed=speed, distance_travelled=4.0))
    assert ctrl.state.moving_elapsed == 0.0
    for _ in range(8):
        assert ctrl.step(perceive(obstacle_distance=60.0, obstacle_speed=4.0,
                                 distance_travelled=4.0)).mode == Mode.CRUISE


def test_never_overtakes_an_obstacle_that_may_be_moving():
    # CARLA lead_vehicle: lead pulling away at 1.9 m/s, not yet confirmed as moving.
    state = AvoidanceState(mode=Mode.CRUISE)
    _, cmd = decide(state, perceive(obstacle_distance=24.8, speed=8.0, adjacent_lane_clear=True,
                                    obstacle_speed=1.9, distance_travelled=10.0), CFG)
    assert cmd.mode != Mode.AVOID
