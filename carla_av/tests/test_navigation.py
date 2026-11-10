from types import SimpleNamespace as NS
import pytest
from navigation.route_planner import RoutePoint, shortest_path
from navigation.route_follower import RouteFollower
from avoidance.collision_avoidance import AvoidanceConfig, CollisionAvoidance, Mode, Perception


def vehicle(x=0, y=0, yaw=0):
    return NS(get_location=lambda: NS(x=x,y=y,z=0), get_transform=lambda: NS(rotation=NS(yaw=yaw)))


def test_directed_shortest_path():
    assert shortest_path([0,1,2], [(0,1,2),(1,2,3),(0,2,9)],0,2)==[0,1,2]
    assert shortest_path([0,1], [(0,1,2)],1,0)==[]


def test_follower_policy_and_progress():
    route=[RoutePoint(x,0,0,0,20<=x<=24,1,-1) for x in range(0,101,2)]
    f=RouteFollower(route)
    assert f.steer(vehicle())==0
    assert f.speed_limit() is None
    f.steer(vehicle(2))
    assert f.speed_limit()==5
    f.steer(vehicle(50))
    assert f.remaining_m==50
    f.steer(vehicle(0))
    assert f.progress_m>=45
    f.steer(vehicle(98))
    assert f.done


def test_turn_and_frame():
    f=RouteFollower([RoutePoint(0,0,0,0,False,1,-1), RoutePoint(5,5,0,90,True,1,-1)])
    assert f.steer(vehicle())>0
    assert f.speed_limit()==4
    assert f.path_ahead(vehicle(),20)[-1]==(5,5)


def test_junction_prevents_overtake_and_caps_speed():
    p=Perception(0,15,5,True,near_junction=True,speed_limit=4)
    assert CollisionAvoidance().step(p).mode!=Mode.AVOID
    assert CollisionAvoidance().step(Perception(0,None,6,False,speed_limit=4)).throttle==0

class Waypoint:
    def __init__(self, x, y=0, road=1, lane=-1, junction=False):
        self.transform=NS(location=NS(x=x,y=y,z=0),rotation=NS(yaw=0))
        self.road_id,self.lane_id,self.is_junction=road,lane,junction
        self.s=x
        self.lane_type='Driving'
        self.following=[]

    def next(self, step):
        return self.following


def test_fake_topology_connects_segments_and_trims_endpoints():
    from navigation.route_planner import plan_route
    w=[Waypoint(x,road=1 if x<=4 else 2) for x in range(0,11,2)]
    for a,b in zip(w,w[1:]):
        a.following=[b]
    start2=Waypoint(4,road=2)
    start2.following=[w[3]]
    cmap=NS(get_topology=lambda:[(w[0],w[2]),(start2,w[-1])],
            get_waypoint=lambda loc: min(w,key=lambda p: abs(p.s-loc.x)))
    route=plan_route(cmap,w[1].transform.location,w[-2].transform.location)
    assert [p.x for p in route]==[2,4,4,6,8]


def test_lane_change_cost_and_junction_exclusion():
    from navigation.route_planner import plan_route
    a,b,c,d=Waypoint(0),Waypoint(2),Waypoint(0,3,lane=-2),Waypoint(2,3,lane=-2)
    a.following=[b]
    c.following=[d]
    cmap=NS(get_topology=lambda:[(a,b),(c,d)],
            get_waypoint=lambda loc: a if loc.y==0 else d)
    assert plan_route(cmap,a.transform.location,d.transform.location)
    for wp in (a,b,c,d):
        wp.is_junction=True
    assert plan_route(cmap,a.transform.location,d.transform.location)==[]


def test_route_ground_truth_measures_around_turn():
    from avoidance.carla_adapter import route_obstacle_distance
    points=[RoutePoint(0,0,0,0,False,1,-1),RoutePoint(10,0,0,90,True,2,-1),
            RoutePoint(10,10,0,90,False,2,-1)]
    ego=vehicle()
    ego.id=1
    ego.bounding_box=NS(extent=NS(x=2,y=1))
    other=vehicle(10,8)
    other.id=2
    other.bounding_box=NS(extent=NS(x=2,y=1))
    world=NS(get_actors=lambda:NS(filter=lambda _: [ego,other]))
    assert route_obstacle_distance(world,RouteFollower(points))(ego)==pytest.approx(14)


def test_route_analysis_ignores_junction_offset_but_requires_goal():
    from analysis.analyze_results import _successful
    rows=[dict(speed=0,lateral_offset=4,in_junction=1,route_remaining=0),
          dict(speed=0,lateral_offset=1,in_junction=0,route_remaining=4)]
    assert _successful('route',rows,None,4,False)
    rows[-1]['route_remaining']=6
    assert not _successful('route',rows,None,4,False)
    assert _successful('route_obstacle',rows,2,4,False)
    assert not _successful('route_obstacle',rows,1.9,4,False)


def test_bridge_uses_route_reference_and_supplies_detector_path():
    import carla
    from avoidance.carla_adapter import CarlaAvoidanceBridge
    ego=vehicle(2,1)
    ego.get_location=lambda:carla.Location(x=2,y=1)
    ego.get_velocity=lambda:NS(x=0,y=0,z=0)
    ego.bounding_box=NS(extent=NS(x=2,y=1))
    ego.id=1
    lane=NS(transform=carla.Transform(carla.Location(y=20)),get_left_lane=lambda:None)
    world=NS(get_map=lambda:NS(get_waypoint=lambda loc:lane),
             get_snapshot=lambda:NS(frame=1,timestamp=NS(delta_seconds=.05)))
    class Detector:
        path_fn=None
        def __call__(self, ego):
            return None
    detector=Detector()
    route=RouteFollower([RoutePoint(x,0,0,0,x==10,1,-1) for x in range(0,21,2)])
    bridge=CarlaAvoidanceBridge(world,ego,AvoidanceConfig(),route=route,
                                obstacle_distance_fn=detector)
    p=bridge.perceive()
    assert p.lateral_offset==pytest.approx(1)
    assert p.near_junction and p.speed_limit==5
    assert p.lane_steer<0
    assert detector.path_fn==route.path_ahead


def test_navigation_csv_analysis(tmp_path):
    import csv
    from analysis.analyze_results import analyze
    fields=('t','speed','lateral_offset','odometer','obstacle_distance','collision','mode',
            'route_progress','route_remaining','in_junction')
    path=tmp_path/'route_run1.csv'
    with path.open('w',newline='') as fh:
        writer=csv.DictWriter(fh,fieldnames=fields)
        writer.writeheader()
        for i,junction in enumerate([0,1,0]):
            writer.writerow(dict(t=i,speed=4,lateral_offset=0,odometer=i*4,
                                 obstacle_distance='',collision=0,mode='CRUISE',
                                 route_progress=i*148,route_remaining=300-i*148,in_junction=junction))
    metric,=analyze(tmp_path,make_plots=False)
    assert metric.success
    assert metric.route_length_m==300
    assert metric.junctions_passed==1
    assert metric.mean_speed==4
    assert metric.time_s==2
    assert 'Route length (m)' in (tmp_path/'summary.md').read_text()


def test_navigation_flags_and_empty_town(monkeypatch):
    import random
    from scenarios.run_scenarios import _parse_args, choose_navigation_route, NoUsableRoad
    monkeypatch.setattr('sys.argv',['runner','--scenario','navigation'])
    assert _parse_args().scenario=='navigation'
    with pytest.raises(NoUsableRoad):
        choose_navigation_route(NS(generate_waypoints=lambda _:[]),random.Random(1))


def test_path_ahead_stops_where_a_sharp_turn_turns_away():
    """90-degree right turn 10 m ahead: the corridor must not mix points after the corner."""
    import math
    from types import SimpleNamespace

    from navigation.route_follower import RouteFollower
    from navigation.route_planner import RoutePoint

    straight = [RoutePoint(x=float(i), y=0.0, z=0.0, yaw=0.0, is_junction=False, road_id=1, lane_id=1)
                for i in range(0, 12, 2)]
    turn = [RoutePoint(x=10.0, y=float(j), z=0.0, yaw=90.0, is_junction=True, road_id=2, lane_id=1)
            for j in range(2, 22, 2)]
    follower = RouteFollower(straight + turn)
    car = SimpleNamespace(get_location=lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0),
                          get_transform=lambda: SimpleNamespace(rotation=SimpleNamespace(yaw=0.0)))
    path = follower.path_ahead(car, 40.0)
    xs = [p[0] for p in path]
    assert xs == sorted(xs) and len(set(xs)) == len(xs)
    assert max(xs) <= 10.5 and all(abs(p[1]) < 1e-6 for p in path)


def test_route_obstacle_uses_any_junction_with_room_after_it():
    import random
    from types import SimpleNamespace

    from scenarios.run_scenarios import route_obstacle_index

    # junction A at 10-14 m followed by only 15 m of road, then junction B at 30-34 m
    # followed by 80 m of road: only B qualifies.
    flags = [10 <= x <= 14 or 30 <= x <= 34 for x in range(0, 120, 2)]
    pts = [SimpleNamespace(is_junction=f) for f in flags]
    route = SimpleNamespace(route=pts, arc=[float(x) for x in range(0, 120, 2)])
    idx = route_obstacle_index(route, random.Random(1))
    assert idx is not None and 36 + 25 <= route.arc[idx] <= 36 + 60
    assert route_obstacle_index(SimpleNamespace(route=pts[:20], arc=route.arc[:20]), random.Random(1)) is None
