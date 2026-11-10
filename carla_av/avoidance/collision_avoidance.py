"""Collision avoidance: stop vs. steer-around decision logic.

Owner: Omar (Obstacle Detection & Avoidance)

Pure-Python state machine with NO CARLA dependency, so it can be unit-tested
on any machine. The CARLA side (carla_adapter.py) builds a `Perception` each
tick and applies the returned `ControlCommand` to the vehicle.

Agreed module interfaces (see Setup card):
    lane module      -> steering_angle     (float, normalised [-1, 1])
    obstacle module  -> obstacle_distance  (float metres, or None if path clear)

Decision summary:
    path clear                         -> CRUISE   (follow lane)
    obstacle inside avoid distance
        and adjacent lane is clear     -> AVOID    (shift into adjacent lane)
        and adjacent lane is blocked   -> BRAKE    (slow down / stop)
    obstacle inside emergency distance -> EMERGENCY_STOP (full brake, always)
    passed the obstacle                -> RETURN   (shift back to original lane)
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import logging
from typing import Optional

log = logging.getLogger(__name__)

class Mode(str, Enum):
    CRUISE = "CRUISE"
    FOLLOW = "FOLLOW"
    BRAKE = "BRAKE"
    STOPPED = "STOPPED"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    AVOID = "AVOID"
    ABORT = "ABORT"
    PASSING = "PASSING"
    RETURN = "RETURN"


@dataclass(frozen=True)
class AvoidanceConfig:
    """Tunable thresholds. All distances in metres, speeds in m/s."""

    target_speed: float = 8.33        # m/s cruise speed (30 km/h, urban)
    throttle_feedforward: float = 0.35  # throttle that roughly holds cruise speed
    speed_kp: float = 0.15            # throttle per m/s of speed error
    max_throttle: float = 0.75
    approach_speed_factor: float = 0.6  # slow to 60% when an obstacle is seen far ahead
    max_decel: float = 6.0            # comfortable-max braking deceleration
    reaction_time: float = 1.0        # perception + actuation latency (conservative)
    safety_margin: float = 4.0        # gap we want to keep when stopped
    emergency_distance: float = 5.0   # always full-brake below this
    hard_stop_distance: float = 2.0   # full-brake limit while already mid lane change
    lane_change_time: float = 2.5     # s, time a lane change takes -> start it early enough
    avoid_min_speed: float = 2.0     # m/s minimum pull-out target
    adjacent_clear_hold: float = 1.0  # s of continuous clearance before pull-out
    pullout_speed: float = 3.0       # full pull-out gains below this speed
    pullout_heading_scale: float = 0.35
    pullout_max_steer: float = 0.7
    creep_speed: float = 1.0
    creep_min_gap: float = 0.8
    avoid_lookahead: float = 3.0      # extra buffer before the avoid trigger point
    stop_hold_buffer: float = 8.0     # once stopped, stay stopped within d_stop + this
    abort_min_ahead: float = 8.0      # minimum bumper clearance to return home safely
    ego_bumper_margin: float = 2.4   # conservative centre-to-front-bumper allowance
    lane_width: float = 3.5
    lateral_kp: float = 0.10          # P-gain: lateral error (m) -> steer
    heading_kd: float = 1.5           # D-like gain: heading error (rad) -> steer (damping)
    lateral_tolerance: float = 0.3    # "close enough" to target lane centre
    max_avoid_steer: float = 0.4      # clamp steer during lane change
    pass_clear_distance: float = 6.0  # distance driven past obstacle before returning
    stopped_speed: float = 0.2
    avoid_side: int = -1              # -1 = left lane, +1 = right lane
    moving_speed: float = 1.0
    standstill_gap: float = 6.0
    time_gap: float = 1.8
    follow_gain: float = 0.4
    ttc_brake: float = 3.0
    manoeuvre_extra_time: float = 4.0
    manoeuvre_margin: float = 10.0
    adjacent_detection_range: float = 60.0

    @property
    def manoeuvre_horizon(self) -> float:
        return 2 * self.lane_change_time + self.manoeuvre_extra_time

    def stopping_distance(self, speed: float) -> float:
        """d = v*t_react + v^2 / (2a) + margin."""
        speed = max(0.0, speed)
        return (
            speed * self.reaction_time
            + speed ** 2 / (2.0 * self.max_decel)
            + self.safety_margin
        )


@dataclass(frozen=True)
class Perception:
    """Everything the decision logic needs for one tick."""

    lane_steer: float                    # steering_angle from lane module [-1, 1]
    obstacle_distance: Optional[float]   # metres to nearest obstacle in path, None = clear
    speed: float                         # ego speed, m/s
    adjacent_lane_clear: bool            # is the avoid-side lane free?
    lateral_offset: float = 0.0          # metres from ORIGINAL lane centre (+ = right)
    distance_travelled: float = 0.0      # odometer, metres (used to know when we passed)
    heading_error: float = 0.0           # rad, car yaw minus lane yaw (+ = pointing right)
    obstacle_speed: Optional[float] = None


@dataclass(frozen=True)
class ControlCommand:
    throttle: float
    steer: float
    brake: float
    mode: Mode


@dataclass(frozen=True)
class AvoidanceState:
    mode: Mode = Mode.CRUISE
    obstacle_odometer: Optional[float] = None   # odometer value at the obstacle position


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _steer_to_offset(
    target: float, p: Perception, cfg: AvoidanceConfig, *, pullout: bool = False,
) -> float:
    """PD-style lateral controller toward a lateral offset.

    The heading term damps the motion: a real car's steering changes its heading,
    so a pure P-controller on lateral offset overshoots the target lane.
    """
    error = target - p.lateral_offset
    if pullout and p.speed < 5.0:
        blend = _clamp((p.speed - cfg.pullout_speed) / (5.0 - cfg.pullout_speed), 0.0, 1.0)
        heading_scale = cfg.pullout_heading_scale + blend * (1.0 - cfg.pullout_heading_scale)
        limit = cfg.pullout_max_steer + blend * (cfg.max_avoid_steer - cfg.pullout_max_steer)
        steer = cfg.lateral_kp * error - cfg.heading_kd * heading_scale * p.heading_error
        return _clamp(steer, -limit, limit)
    steer = cfg.lateral_kp * error - cfg.heading_kd * p.heading_error
    return _clamp(steer, -cfg.max_avoid_steer, cfg.max_avoid_steer)


def _cruise_throttle(p: Perception, cfg: AvoidanceConfig, speed_factor: float = 1.0) -> float:
    """Feed-forward + P speed controller so the car holds a target speed."""
    error = cfg.target_speed * speed_factor - p.speed
    if error < -1.0:
        return 0.0  # well above target: coast
    return _clamp(cfg.throttle_feedforward + cfg.speed_kp * error, 0.0, cfg.max_throttle)


def _avoid_command(p: Perception, cfg: AvoidanceConfig, steer: float) -> ControlCommand:
    target = cfg.target_speed
    if abs(p.lateral_offset) < cfg.lane_width / 2 and p.obstacle_distance is not None and p.obstacle_distance >= 0:
        target = min(target, max(cfg.avoid_min_speed,
                     (p.obstacle_distance - cfg.safety_margin) / cfg.lane_change_time))
        if p.speed > target + 1.0:
            return ControlCommand(0.0, steer, 0.3, Mode.AVOID)
    throttle = _cruise_throttle(p, cfg, target / cfg.target_speed) if cfg.target_speed > 0 else 0.0
    return ControlCommand(throttle, steer, 0.0, Mode.AVOID)


def _brake_command(p: Perception, cfg: AvoidanceConfig) -> ControlCommand:
    """Brake proportionally to how deep we are inside the stopping distance."""
    if p.speed <= cfg.stopped_speed:
        return ControlCommand(0.0, p.lane_steer, 1.0, Mode.STOPPED)
    d_stop = cfg.stopping_distance(p.speed)
    distance = p.obstacle_distance if p.obstacle_distance is not None else d_stop
    usable = max(distance - cfg.safety_margin, 0.01)
    # required decel to stop in `usable` metres, as fraction of max_decel
    required = (p.speed ** 2) / (2.0 * usable)
    brake = _clamp(required / cfg.max_decel, 0.3, 1.0)
    return ControlCommand(0.0, p.lane_steer, brake, Mode.BRAKE)


def _follow_command(p: Perception, cfg: AvoidanceConfig) -> ControlCommand:
    """Adaptive cruise with proportional gap control and closing-speed braking."""
    gap = p.obstacle_distance
    assert gap is not None and p.obstacle_speed is not None
    desired = cfg.standstill_gap + cfg.time_gap * p.speed
    target = _clamp(p.obstacle_speed + cfg.follow_gain * (gap - desired),
                    0.0, cfg.target_speed)
    closing = p.speed - p.obstacle_speed
    ttc = gap / closing if closing > 0 else float("inf")
    if p.speed - target > 1.0 or ttc < cfg.ttc_brake:
        usable = max(gap - cfg.standstill_gap, 0.01)
        required = max(p.speed - target, closing ** 2 / (2 * usable))
        brake = _clamp(required / cfg.max_decel, 0.3, 1.0)
        return ControlCommand(0.0, p.lane_steer, brake, Mode.FOLLOW)
    throttle = _cruise_throttle(p, cfg, target / cfg.target_speed) if cfg.target_speed > 0 else 0.0
    return ControlCommand(throttle, p.lane_steer, 0.0, Mode.FOLLOW)


class CollisionAvoidance:
    """Stateful wrapper. Call `step(perception)` once per simulation tick."""

    def __init__(self, config: AvoidanceConfig | None = None) -> None:
        self.config = config or AvoidanceConfig()
        self.state = AvoidanceState()

    def reset(self) -> None:
        self.state = AvoidanceState()

    def step(self, p: Perception) -> ControlCommand:
        new_state, command = decide(self.state, p, self.config)
        self.state = new_state
        return command


def _manoeuvre_step(
    state: AvoidanceState, p: Perception, cfg: AvoidanceConfig,
) -> tuple[AvoidanceState, ControlCommand]:
    mode = state.mode
    target_offset = cfg.avoid_side * cfg.lane_width
    if mode == Mode.ABORT:
        return _abort_step(state, p, cfg)
    # 2. Lane change in progress: steer to adjacent lane.
    if mode == Mode.AVOID:
        still_mostly_home = abs(p.lateral_offset) < cfg.lane_width / 2
        if not p.adjacent_lane_clear:
            command = _brake_command(p, cfg)
            if still_mostly_home:
                # Target lane got occupied before we committed -> abort and brake.
                return AvoidanceState(mode=command.mode), command
            ahead = (state.obstacle_odometer - p.distance_travelled - cfg.ego_bumper_margin
                     if state.obstacle_odometer is not None else float("-inf"))
            if ahead >= cfg.abort_min_ahead:
                return _abort_step(replace(state, mode=Mode.ABORT), p, cfg)
            # Already committed: brake hard but keep the manoeuvre state so we
            # resume (or wait) instead of steering back into the obstacle.
            steer = _steer_to_offset(target_offset, p, cfg, pullout=True)
            return state, ControlCommand(0.0, steer, max(command.brake, 0.6), command.mode)
        if abs(target_offset - p.lateral_offset) <= cfg.lateral_tolerance:
            return replace(state, mode=Mode.PASSING), ControlCommand(
                _cruise_throttle(p, cfg), 0.0, 0.0, Mode.PASSING
            )
        steer = _steer_to_offset(target_offset, p, cfg, pullout=True)
        return state, _avoid_command(p, cfg, steer)

    # 3. In adjacent lane: hold it until we are past the obstacle.
    if mode == Mode.PASSING:
        passed = (
            state.obstacle_odometer is not None
            and p.distance_travelled >= state.obstacle_odometer + cfg.pass_clear_distance
        )
        if passed:
            return replace(state, mode=Mode.RETURN), ControlCommand(
                _cruise_throttle(p, cfg), _steer_to_offset(0.0, p, cfg), 0.0, Mode.RETURN
            )
        steer = _steer_to_offset(target_offset, p, cfg)
        return state, ControlCommand(_cruise_throttle(p, cfg), steer, 0.0, Mode.PASSING)

    return _return_step(state, p, cfg)


def _abort_step(
    state: AvoidanceState, p: Perception, cfg: AvoidanceConfig,
) -> tuple[AvoidanceState, ControlCommand]:
    # The home-lane obstacle can leave the perception band during the manoeuvre.
    # Keep braking against its remembered position until we are back home.
    ahead = (state.obstacle_odometer - p.distance_travelled - cfg.ego_bumper_margin
             if state.obstacle_odometer is not None else p.obstacle_distance)
    stopping = replace(p, obstacle_distance=ahead)
    command = _brake_command(stopping, cfg)
    if abs(p.lateral_offset) <= cfg.lateral_tolerance:
        return AvoidanceState(mode=command.mode), command
    return state, replace(command, steer=_steer_to_offset(0.0, p, cfg, pullout=True), mode=Mode.ABORT)


def _return_step(
    state: AvoidanceState, p: Perception, cfg: AvoidanceConfig,
) -> tuple[AvoidanceState, ControlCommand]:
    if abs(p.lateral_offset) <= cfg.lateral_tolerance:
        return AvoidanceState(), ControlCommand(
            _cruise_throttle(p, cfg), p.lane_steer, 0.0, Mode.CRUISE
        )
    steer = _steer_to_offset(0.0, p, cfg)
    return state, ControlCommand(_cruise_throttle(p, cfg), steer, 0.0, Mode.RETURN)



def decide(
    state: AvoidanceState, p: Perception, cfg: AvoidanceConfig
) -> tuple[AvoidanceState, ControlCommand]:
    """Pure transition function: (state, perception) -> (new_state, command)."""
    dist = p.obstacle_distance
    blocked = dist is not None
    target_offset = cfg.avoid_side * cfg.lane_width

    # 1. Emergency stop overrides everything. A negative gap means the obstacle is
    #    alongside us (we are overtaking it), which is not a threat to brake for.
    #    Mid lane change the car's nose may still overlap the obstacle's path a little,
    #    so only a much closer gap counts, and the AVOID state is kept (not reset) so
    #    the car does not "forget" the manoeuvre and steer back into the obstacle.
    manoeuvring = state.mode == Mode.AVOID
    limit = cfg.hard_stop_distance if manoeuvring else cfg.emergency_distance
    if blocked and 0.0 <= dist <= limit:
        # A stopped pull-out can recover from the hard-stop zone only while
        # there is room ahead and the target lane is still free.
        if (manoeuvring and cfg.creep_min_gap < dist < cfg.hard_stop_distance
                and p.speed <= cfg.creep_speed + 0.5 and p.adjacent_lane_clear
                and abs(p.lateral_offset) < cfg.lane_width):
            throttle = (_cruise_throttle(p, cfg, cfg.creep_speed / cfg.target_speed)
                        if cfg.target_speed > 0 else 0.0)
            direction = 1.0 if target_offset > p.lateral_offset else -1.0
            return state, ControlCommand(
                throttle, direction * cfg.pullout_max_steer, 0.0, Mode.AVOID
            )
        # Already standing still -> just hold; it is not an emergency any more.
        mode = Mode.STOPPED if p.speed <= cfg.stopped_speed else Mode.EMERGENCY_STOP
        if manoeuvring:
            steer = _steer_to_offset(target_offset, p, cfg, pullout=True)
            return state, ControlCommand(0.0, steer, 1.0, mode)
        return replace(state, mode=mode), ControlCommand(0.0, p.lane_steer, 1.0, mode)

    mode = state.mode

    if mode in (Mode.AVOID, Mode.ABORT, Mode.PASSING, Mode.RETURN):
        return _manoeuvre_step(state, p, cfg)

    return _road_step(state, p, cfg)


def _road_step(
    state: AvoidanceState, p: Perception, cfg: AvoidanceConfig,
) -> tuple[AvoidanceState, ControlCommand]:
    dist = p.obstacle_distance
    mode = state.mode
    d_stop = cfg.stopping_distance(p.speed)
    target_offset = cfg.avoid_side * cfg.lane_width
    # 5. CRUISE / BRAKE / STOPPED / EMERGENCY_STOP: decide stop vs. steer-around.
    if dist is None:
        return AvoidanceState(), ControlCommand(
            _cruise_throttle(p, cfg), p.lane_steer, 0.0, Mode.CRUISE
        )

    if p.obstacle_speed is not None and p.obstacle_speed > cfg.moving_speed:
        return AvoidanceState(mode=Mode.FOLLOW), _follow_command(p, cfg)

    # Hysteresis: hold near a stationary obstacle to prevent stop-go oscillation.
    braking_modes = (Mode.BRAKE, Mode.STOPPED, Mode.EMERGENCY_STOP)
    hold_zone = d_stop + cfg.stop_hold_buffer if mode in braking_modes else d_stop

    avoid_trigger = d_stop + p.speed * cfg.lane_change_time + cfg.avoid_lookahead
    if mode in braking_modes:
        avoid_trigger = max(avoid_trigger, hold_zone)  # stopped car may pull out
    if dist <= avoid_trigger and p.adjacent_lane_clear:
        obstacle_odometer = p.distance_travelled + dist
        new_state = AvoidanceState(mode=Mode.AVOID, obstacle_odometer=obstacle_odometer)
        steer = _steer_to_offset(target_offset, p, cfg, pullout=True)
        return new_state, _avoid_command(p, cfg, steer)

    if dist <= hold_zone:
        command = _brake_command(p, cfg)
        return replace(state, mode=command.mode), command

    # Obstacle seen but still far away: keep driving, ease off throttle.
    return replace(state, mode=Mode.CRUISE), ControlCommand(
        _cruise_throttle(p, cfg, cfg.approach_speed_factor), p.lane_steer, 0.0, Mode.CRUISE
    )
