"""Ideas adopted from Prithiv's Lanebot (ROS 2): Stanley route steering, traffic-light
stops, target-lane continuation, lane-marking rules and indicators. No CARLA server."""
from types import SimpleNamespace as NS

import pytest

from avoidance.collision_avoidance import AvoidanceConfig, CollisionAvoidance, Mode, Perception
from navigation.route_follower import RouteFollower
from navigation.route_planner import RoutePoint
from navigation.traffic_lights import (
    AMBER, GREEN, RED, RouteTrafficLights, StopLine, light_stop_gap, match_stop_lines,
)


def car(x=0.0, y=0.0, yaw=0.0, speed=5.0):
    return NS(get_location=lambda: NS(x=x, y=y, z=0.0),
              get_transform=lambda: NS(rotation=NS(yaw=yaw)),
              get_velocity=lambda: NS(x=speed, y=0.0, z=0.0))


def straight(length=60):
    return [RoutePoint(x, 0, 0, 0, False, 1, -1) for x in range(0, length + 1, 1)]


# ---------- Stanley steering ----------

def test_stanley_is_zero_on_a_straight_path():
    assert RouteFollower(straight()).steer(car()) == pytest.approx(0.0)


def test_stanley_steers_back_toward_the_path():
    # CARLA: +y is to the right. Car 1 m left of the path -> steer right (+).
    assert RouteFollower(straight()).steer(car(y=-1.0)) > 0
    assert RouteFollower(straight()).steer(car(y=1.0)) < 0


def test_stanley_corrects_heading_error():
    # Pointing 10 degrees left of the path -> steer right.
    assert RouteFollower(straight()).steer(car(yaw=-10.0)) > 0


def test_stanley_curve_feed_forward_turns_into_the_curve():
    # A right-hand curve: yaw grows 3 degrees per metre after x = 10.
    route = [RoutePoint(x, 0, 0, max(0, x - 10) * 3.0, False, 1, -1) for x in range(0, 40)]
    follower = RouteFollower(route)
    follower.steer(car(x=8.0))
    assert follower.curvature() > 0
    assert follower.steer(car(x=8.0)) > 0


def test_pursuit_mode_is_kept_and_bad_modes_are_rejected():
    assert RouteFollower(straight(), steer_mode="pursuit").steer(car(y=-1.0)) > 0
    with pytest.raises(ValueError):
        RouteFollower(straight(), steer_mode="magic")


# ---------- traffic lights ----------

def line(arc, state):
    return StopLine(arc, lambda: state)


def test_red_light_gives_a_stop_gap_from_the_bumper():
    assert light_stop_gap([line(50, RED)], progress_m=10, speed=8, bumper_m=2.4) == pytest.approx(37.6)


def test_green_and_far_lights_are_ignored():
    assert light_stop_gap([line(50, GREEN)], 10, 8, 2.4) is None
    assert light_stop_gap([line(500, RED)], 10, 8, 2.4) is None


def test_amber_stops_only_when_a_comfortable_stop_is_possible():
    assert light_stop_gap([line(50, AMBER)], 10, 8, 2.4) is not None      # 8 m needed, 37 m left
    assert light_stop_gap([line(14, AMBER)], 10, 8, 2.4) is None          # 1.6 m left: drive on


def test_a_line_already_under_the_bumper_is_ignored():
    assert light_stop_gap([line(11, RED), line(80, RED)], 10, 0, 2.4) == pytest.approx(67.6)


def test_stop_lines_match_route_by_road_lane_and_distance():
    route = straight()
    arc = [float(x) for x in range(len(route))]
    stop = NS(transform=NS(location=NS(x=30.4, y=0.5)), road_id=1, lane_id=-1)
    other_lane = NS(transform=NS(location=NS(x=20.0, y=0.0)), road_id=1, lane_id=1)
    lines = match_stop_lines(route, arc, [(stop, lambda: RED), (other_lane, lambda: RED)])
    assert [s.arc_m for s in lines] == [30.0]


def test_red_light_crossings_are_counted():
    state = {"now": RED}
    lights = RouteTrafficLights([StopLine(30.0, lambda: state["now"])])
    lights.gap(20.0, 5.0, 2.4)
    lights.gap(29.0, 5.0, 2.4)          # nose passes 30 m while red
    assert lights.violations == 1
    state["now"] = GREEN
    lights2 = RouteTrafficLights([StopLine(30.0, lambda: state["now"])])
    lights2.gap(20.0, 5.0, 2.4)
    lights2.gap(29.0, 5.0, 2.4)
    assert lights2.violations == 0


def test_controller_stops_for_a_red_light_and_never_overtakes_it():
    # The bridge hands a red light over as a stationary obstacle with the next lane closed.
    ctrl = CollisionAvoidance(AvoidanceConfig(target_speed=8.0))
    p = Perception(lane_steer=0.0, obstacle_distance=12.0, speed=8.0,
                   adjacent_lane_clear=False, obstacle_speed=0.0)
    cmd = ctrl.step(p)
    assert cmd.mode in (Mode.BRAKE, Mode.EMERGENCY_STOP) and cmd.brake > 0


# ---------- bridge: target lane continues, markings, indicators ----------

class Wp:
    """Waypoint on a straight road along x with an optional left lane until `left_until`."""

    def __init__(self, x, left_until=1e9, lane_id=-1, marking="Broken", change="Both"):
        import carla
        self.x, self.left_until, self.lane_id = x, left_until, lane_id
        self.transform = carla.Transform(carla.Location(x=x), carla.Rotation(yaw=0))
        self.lane_type = carla.LaneType.Driving
        self.left_lane_marking = NS(type=marking)
        self.right_lane_marking = NS(type="Solid")
        self.lane_change = getattr(carla.LaneChange, change)

    def next(self, d):
        return [Wp(self.x + d, self.left_until, self.lane_id,
                   self.left_lane_marking.type, self.lane_change.name)]

    def previous(self, d):
        return [Wp(self.x - d, self.left_until, self.lane_id)]

    def get_left_lane(self):
        if self.x > self.left_until:
            return None
        import carla
        return NS(lane_type=carla.LaneType.Driving, lane_id=self.lane_id - 1)

    def get_right_lane(self):
        return None


def bridge(cfg=None):
    import carla
    from avoidance.carla_adapter import CarlaAvoidanceBridge
    ego = car()
    ego.bounding_box = NS(extent=NS(x=2, y=1))
    ego.id = 1
    ego.lights = []
    ego.set_light_state = ego.lights.append
    ego.apply_control = lambda control: None
    world = NS(get_map=lambda: NS(get_waypoint=lambda loc: Wp(0)),
               get_snapshot=lambda: NS(frame=1, timestamp=NS(delta_seconds=.05)))
    return CarlaAvoidanceBridge(world, ego, cfg or AvoidanceConfig(),
                                obstacle_distance_fn=lambda v: None, home_waypoint=Wp(0)), carla


def test_target_lane_must_continue_for_the_whole_manoeuvre():
    b, _ = bridge()
    assert b.target_lane_continues(Wp(0), 60.0)
    assert not b.target_lane_continues(Wp(0, left_until=30.0), 60.0)


def test_solid_lines_block_overtakes_only_when_markings_are_obeyed():
    b, _ = bridge()
    assert b.lane_change_allowed(Wp(0, marking="Solid"))
    strict, _ = bridge(AvoidanceConfig(obey_lane_markings=True))
    assert not strict.lane_change_allowed(Wp(0, marking="Solid"))
    assert strict.lane_change_allowed(Wp(0, marking="Broken"))
    assert not strict.lane_change_allowed(Wp(0, marking="Broken", change="NONE"))


def test_indicator_follows_the_manoeuvre():
    from avoidance.collision_avoidance import ControlCommand
    b, carla = bridge()
    b.apply(ControlCommand(0.3, -0.2, 0.0, Mode.AVOID))
    b.apply(ControlCommand(0.3, -0.2, 0.0, Mode.PASSING))   # unchanged: no second call
    b.apply(ControlCommand(0.3, 0.2, 0.0, Mode.RETURN))
    lights = b.vehicle.lights
    assert len(lights) == 2
    assert lights[0] & carla.VehicleLightState.LeftBlinker      # avoid_side -1: out to the left
    assert lights[1] & carla.VehicleLightState.RightBlinker


# ---------- emergency vehicles, pedestrians, closer stop at lights ----------

def test_car_makes_way_for_an_emergency_vehicle_behind():
    cfg = AvoidanceConfig(target_speed=8.0)
    p = Perception(lane_steer=0.0, obstacle_distance=None, speed=8.0, adjacent_lane_clear=True,
                   emergency_distance=-60.0)
    cmd = CollisionAvoidance(cfg).step(p)
    assert cmd.mode == Mode.YIELD
    assert cmd.steer > 0                      # towards the right edge
    far = Perception(lane_steer=0.0, obstacle_distance=None, speed=8.0, adjacent_lane_clear=True,
                     emergency_distance=-200.0)
    assert CollisionAvoidance(cfg).step(far).mode == Mode.CRUISE


def test_no_overtaking_near_pedestrians_or_emergency_vehicles():
    cfg = AvoidanceConfig(target_speed=8.0)
    base = dict(lane_steer=0.0, obstacle_distance=20.0, speed=8.0, adjacent_lane_clear=True,
                obstacle_speed=0.0)
    assert CollisionAvoidance(cfg).step(Perception(**base)).mode == Mode.AVOID
    assert CollisionAvoidance(cfg).step(Perception(**base, pedestrian_near=True)).mode != Mode.AVOID
    assert CollisionAvoidance(cfg).step(Perception(**base, emergency_distance=-50.0)).mode != Mode.AVOID


def test_cluster_state_converts_lidar_and_modes():
    import numpy as np
    from scenarios.cluster import ClusterState, STATES
    st = ClusterState(points=np.array([[10.0, 0.0, 1.0]]), mode="YIELD")
    assert st.points.shape == (1, 3)
    assert "YIELD" in STATES and all(m.value in STATES for m in Mode)


# ---------- speed rules, rule checks, headlights ----------

def test_speed_rules_follow_limit_sign_and_sight():
    from avoidance.traffic_rules import rule_speed_limit, sight_speed, sign_speed, is_speeding
    assert rule_speed_limit(30.0, None, 60.0, 6.0) == pytest.approx(0.9 * 30 / 3.6)
    # a 30 sign 5 m ahead while the limit here is 50: already down to 27 km/h
    assert sign_speed(30.0, 5.0) == pytest.approx(0.9 * 30 / 3.6)
    assert rule_speed_limit(50.0, (30.0, 5.0), 60.0, 6.0) == pytest.approx(0.9 * 30 / 3.6)
    assert rule_speed_limit(None, None, 20.0, 6.0) == pytest.approx(sight_speed(20.0, 6.0))
    assert is_speeding(40 / 3.6, 30.0) and not is_speeding(32 / 3.6, 30.0)


def test_rule_metrics_count_seconds_and_near_miss_events():
    from analysis.analyze_results import rule_metrics
    rows = [dict(t=i * 0.05, speeding=i < 20, off_road=False, near_miss=i in (5, 6, 30)) for i in range(40)]
    m = rule_metrics(rows)
    assert m["speeding_s"] == pytest.approx(1.0)
    assert m["off_road_s"] == 0
    assert m["near_misses"] == 2          # two separate events


def test_high_beam_only_at_night_without_traffic_ahead():
    b, carla = bridge()
    assert not b.high_beam_allowed()
    b.night = True
    b.world.get_actors = lambda: NS(filter=lambda pattern: [])
    assert b.high_beam_allowed()
