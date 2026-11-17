"""Watches a test drive and records every problem."""
import json, math, subprocess, sys, time
import carla, rclpy
from std_msgs.msg import Float32MultiArray, String
from carla_msgs.msg import CarlaCollisionEvent

DURATION = float(sys.argv[1]); OUT = sys.argv[2]
c = carla.Client('localhost', 2000); c.set_timeout(20); w = c.get_world(); m = w.get_map()
rclpy.init(); n = rclpy.create_node('monitor')
st = {'state': 'CRUISE', 'lane': None}
ev = {'collisions': [], 'off_lane': [], 'off_road': [], 'speeding': [], 'red_light': [], 'near_miss': [],
      'crash': [], 'states': {}, 'min_gap_m': 99.0, 'max_speed_kmh': 0.0, 'distance_m': 0.0, 'overtakes': 0, 'yields': 0}
def on_state(msg):
    if msg.data != st['state']:
        ev['states'][msg.data] = ev['states'].get(msg.data, 0) + 1
        if msg.data == 'CHANGE_LEFT': ev['overtakes'] += 1
        if msg.data == 'YIELD': ev['yields'] += 1
    st['state'] = msg.data
n.create_subscription(String, '/lanebot/state', on_state, 10)
n.create_subscription(Float32MultiArray, '/lanebot/lane', lambda msg: st.__setitem__('lane', list(msg.data)), 10)
def on_coll(msg):
    a = w.get_actor(msg.other_actor_id)
    ev['collisions'].append({'t': round(time.monotonic() - t0, 1), 'with': a.type_id if a else msg.other_actor_id})
n.create_subscription(CarlaCollisionEvent, '/carla/ego_vehicle/collision', on_coll, 10)
def log(kind, t, **kw):
    if not ev[kind] or t - ev[kind][-1]['t'] > 5: ev[kind].append({'t': round(t, 1), **kw})
crossing = {}
t0 = time.monotonic(); last_check = 0; last_loc = None; off_lane_since = None
while time.monotonic() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.05)
    t = time.monotonic() - t0
    if t - last_check < 0.25: continue
    last_check = t
    procs = subprocess.run(['pgrep', '-af', 'CarlaUE4-Linux-Shipping|lib/carla_ros_bridge/bridge|lib/lanebot_ros/'], capture_output=True, text=True).stdout
    for name in ('CarlaUE4-Linux-Shipping', 'carla_ros_bridge/bridge', 'lane_node', 'obstacle_node', 'control_node', 'signals_node', 'traffic_node'):
        if name not in procs: log('crash', t, what=name)
    try:
        vs = list(w.get_actors().filter('vehicle.*'))
    except RuntimeError as e:
        log('crash', t, what=f'carla: {e}'); continue
    ego = next((v for v in vs if v.attributes.get('role_name') == 'ego_vehicle'), None)
    if ego is None: log('crash', t, what='ego missing'); continue
    loc = ego.get_location(); kmh = ego.get_velocity().length() * 3.6
    if last_loc: ev['distance_m'] += loc.distance(last_loc)
    last_loc = loc; ev['max_speed_kmh'] = max(ev['max_speed_kmh'], kmh)
    if kmh > ego.get_speed_limit() + 3: log('speeding', t, kmh=round(kmh), limit=ego.get_speed_limit())
    tl = ego.get_traffic_light()
    if tl is not None:
        tf0 = ego.get_transform(); fw = tf0.get_forward_vector()
        for sw in tl.get_stop_waypoints():
            d = sw.transform.location - tf0.location
            if abs(d.x * fw.y - d.y * fw.x) > 2.0: continue        # another lane's stop line
            front = d.x * fw.x + d.y * fw.y - 2.4                   # stop line ahead of our bumper (m)
            key = (tl.id, sw.lane_id)
            if key in crossing and crossing[key] > 0 >= front and tl.get_state() == carla.TrafficLightState.Red:
                log('red_light', t, kmh=round(kmh))
            crossing[key] = front
    wp = m.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
    if wp is None:   # CARLA's map has thin seams on lane lines: look 30 cm either side
        rv = ego.get_transform().get_right_vector()
        for dl in (-0.3, 0.3):
            wp = wp or m.get_waypoint(loc + rv * dl, project_to_road=False, lane_type=carla.LaneType.Any)
    if wp is None or wp.lane_type != carla.LaneType.Driving:
        log('off_road', t, lane_type=str(wp.lane_type) if wp else 'none', state=st['state'], offset=round(st['lane'][0], 2) if st['lane'] else None, x=round(loc.x), y=round(loc.y))
    else:
        left = wp.get_left_lane(); right = wp.get_right_lane()
        multi = any(q is not None and q.lane_type == carla.LaneType.Driving and q.lane_id * wp.lane_id > 0 for q in (left, right))
        if not multi and not wp.is_junction: log('off_road', t, lane_type='single-lane road (left the motorway)')
    if st['lane'] and st['state'] == 'CRUISE':
        if abs(st['lane'][0]) > 1.0:
            off_lane_since = off_lane_since or t
            if t - off_lane_since > 1.0: log('off_lane', t, offset=round(st['lane'][0], 2))
        else: off_lane_since = None
    else: off_lane_since = None
    tf = ego.get_transform(); fwd = tf.get_forward_vector()
    for v in vs:
        if v.id == ego.id: continue
        d = v.get_location() - loc
        ahead = d.x * fwd.x + d.y * fwd.y; side = math.sqrt(max(0.0, d.x * d.x + d.y * d.y - ahead * ahead))
        if side < 2.2:
            gap = abs(ahead) - 4.7
            ev['min_gap_m'] = min(ev['min_gap_m'], round(gap, 2))
            if gap < 1.0: log('near_miss', t, other=v.type_id, gap=round(gap, 2))
ev['duration_s'] = round(time.monotonic() - t0)
json.dump(ev, open(OUT, 'w'), indent=1)
bad = sum(len(ev[k]) for k in ('collisions', 'off_lane', 'off_road', 'speeding', 'red_light', 'near_miss', 'crash'))
print(('PASS' if bad == 0 else f'FAIL ({bad} problems)'), json.dumps({k: ev[k] for k in ev}, separators=(',', ':')))
