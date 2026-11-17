"""Drives the car: steering, speed and decisions, following German traffic rules (StVO).

Rules built in:
  §2  keep right       home lane is the rightmost lane; we go back to it after passing
  §3  speed            never above the signed limit, and only as fast as we can
                       stop within what our sensors can see ("Fahren auf Sicht")
  §4  distance         a 2-second gap to the car ahead
  §5  overtaking       only on the left, only across a broken line, with the
                       indicator on for 3 s first; no passing on the right
  §7  lane changes     only when the target lane is free ahead and behind
  §37 traffic lights   stop at red and amber
  §38 blue lights      make way at once: back to the right lane, to its right edge,
                       slow down, until the emergency vehicle has passed
"""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
from carla_msgs.msg import CarlaEgoVehicleControl

# --- car ---
MAX_WHEEL_ANGLE = 1.2  # rad, how far the front wheels turn at steer = 1.0
WHEELBASE = 2.9        # m
K_OFFSET = 1.0         # how strongly to correct sideways offset
K_SPEED = 0.3          # throttle per m/s of speed error
DT = 0.05              # s, time between control steps

# --- speed rules ---
LIMIT_SHARE = 0.9      # drive at 90 % of the speed limit
SENSOR_RANGE = 45.0    # m we can see obstacles in (lidar range minus margin)
BRAKE_DECEL = 3.0      # m/s^2, comfortable braking we plan with
SIGHT_DECEL = 4.0      # m/s^2, firm braking for "Fahren auf Sicht" (what we can see)
AMBER_DECEL = 4.0      # m/s^2: at amber, stop only if this is enough (StVO §37)
TIME_GAP = 2.0         # s, gap to the car ahead (German "halber Tacho" is 1.8 s)
MIN_GAP = 6.0          # m, gap when standing
YIELD_SHARE = 0.5      # drive at half speed while making way
OVERTAKE_MARGIN = 2.8  # m/s (10 km/h): only overtake vehicles at least this much slower

# --- lane changes ---
SIGNAL_TIME = 3.0      # s of indicator before we move sideways
SHIFT_RATE = 1.0       # m/s, how fast we move sideways
LEFT_CLEAR_AHEAD = 20.0    # m of free left lane needed in front of us
LEFT_CLEAR_BEHIND = 40.0   # m of free left lane needed behind us
LEFT_ROOM_NEEDED = 150.0   # m the left lane must continue ahead before we pull out
LEFT_ROOM_RETURN = 60.0    # m: move back right at once when the left lane ends sooner
HOME_CLEAR_AHEAD = 15.0    # m of free home lane needed ahead to move back
HOME_CLEAR_BEHIND = 10.0   # m between us and the car we passed before moving back
MAX_PASS_TIME = 25.0   # s: never stay in the left lane longer than this (keep right)
YIELD_EDGE = -0.7      # m right of the lane centre while making way (stays inside the lane)
EMERGENCY_BEHIND = 120.0   # m, make way when an emergency vehicle is this close behind
EMERGENCY_AHEAD = 30.0     # m, until it is this far in front of us
FRONT = 2.4            # m from the middle of the car to the front bumper
INF = float('inf')


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.offset = self.heading_error = self.curvature = 0.0
        self.left_shift = 0.0
        self.left_allowed = False
        self.left_room = 0.0           # m the left lane continues ahead
        self.speed = 0.0
        self.obstacle = INF
        self.obstacle_time = 0.0
        self.lead_speed = 0.0          # m/s, estimated speed of whatever is ahead of us
        self.own_ahead = self.left_ahead = self.own_behind = self.left_behind = INF
        self.left_ahead_before = INF   # to tell whether we are catching a car in the left lane
        self.emergency = INF           # m to the nearest emergency vehicle (negative = behind)
        self.limit = 8.3               # m/s, until signals_node tells us the real limit
        self.stop_line = INF           # m to the stop line of a red or amber light
        self.light = 0                 # 0 green or none, 1 amber, 2 red
        self.have_lane = False

        self.state = 'CRUISE'
        self.goal_offset = 0.0     # where we want to end up, sideways
        self.target_offset = 0.0   # where we aim right now (slides smoothly to the goal)
        self.signal_until = 0.0    # time the indicator phase ends
        self.after_signal = None   # (goal offset, next state) once the indicator phase is over
        self.travelled = 0.0
        self.pass_until = 0.0      # "travelled" value at which we are level with the obstacle
        self.overtaking_gap = 30.0
        self.pass_started = 0.0
        self.upcoming_limit = (INF, INF)   # (m/s, metres) of the next speed sign
        self.count = 0

        self.create_subscription(Float32MultiArray, '/lanebot/lane', self.on_lane, 10)
        self.create_subscription(Float32, '/carla/ego_vehicle/speedometer', self.on_speed, 10)
        self.create_subscription(Float32, '/lanebot/obstacle_distance', self.on_obstacle, 10)
        self.create_subscription(Float32MultiArray, '/lanebot/lanes', self.on_lanes, 10)
        self.create_subscription(Float32MultiArray, '/lanebot/emergency', self.on_emergency, 10)
        self.create_subscription(Float32MultiArray, '/lanebot/rules', self.on_rules, 10)
        self.pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.state_pub = self.create_publisher(String, '/lanebot/state', 10)
        self.create_timer(DT, self.tick)

    # ---------- inputs ----------
    def on_lane(self, msg):
        d = msg.data
        self.offset, self.heading_error, self.curvature, self.left_shift = d[0], d[1], d[2], d[3]
        self.left_allowed = d[4] > 0.5
        self.left_room = d[6] if len(d) > 6 else 0.0
        self.have_lane = True

    def on_speed(self, msg):
        self.speed = msg.data

    def on_obstacle(self, msg):
        # Estimate how fast the thing ahead moves: our speed + how fast the gap grows.
        now = time.monotonic()
        if math.isfinite(msg.data) and math.isfinite(self.obstacle) and now > self.obstacle_time:
            rate = (msg.data - self.obstacle) / (now - self.obstacle_time)
            lead = min(max(0.0, self.speed + rate), self.speed + 3.0)
            self.lead_speed += 0.1 * (lead - self.lead_speed)      # smooth out lidar noise
        elif math.isfinite(msg.data):
            self.lead_speed = self.speed
        self.obstacle, self.obstacle_time = msg.data, now

    def on_lanes(self, msg):
        self.left_ahead_before = self.left_ahead
        self.own_ahead, self.left_ahead, self.own_behind, self.left_behind = msg.data[:4]

    def on_emergency(self, msg):
        self.emergency = msg.data[0]

    def on_rules(self, msg):
        self.limit, self.stop_line, self.light = msg.data[0], msg.data[1], int(msg.data[2])
        self.upcoming_limit = (msg.data[3], msg.data[4])

    # ---------- checks ----------
    def left_lane_free(self):
        return (self.left_allowed and self.left_shift > 0.0
                and self.left_room >= LEFT_ROOM_NEEDED
                and self.left_ahead > LEFT_CLEAR_AHEAD
                and self.left_behind > max(LEFT_CLEAR_BEHIND, 3.0 * self.speed))

    def home_lane_free(self):
        return self.own_ahead > max(HOME_CLEAR_AHEAD, TIME_GAP * self.speed) and self.own_behind > HOME_CLEAR_BEHIND

    def emergency_near(self):
        return -EMERGENCY_BEHIND < self.emergency < EMERGENCY_AHEAD

    # ---------- the state machine ----------
    def set_state(self, state):
        self.get_logger().info(f'{self.state} -> {state}')
        self.state = state

    def signal(self, goal, next_state, direction):
        """Indicator on first; move only after SIGNAL_TIME (StVO §5, §7)."""
        if next_state == 'PASS':   # remember how far ahead the thing we overtake is
            self.overtaking_gap = self.obstacle if math.isfinite(self.obstacle) else 30.0
        self.after_signal = (goal, next_state)
        self.signal_until = time.monotonic() + SIGNAL_TIME
        self.set_state(f'SIGNAL_{direction}')

    def decide(self):
        now = time.monotonic()
        overtake_at = max(30.0, 3.0 * self.speed)

        if self.state == 'CRUISE':
            if self.emergency_near():
                self.goal_offset = YIELD_EDGE
                self.set_state('YIELD')
            elif (self.obstacle < overtake_at and self.left_lane_free()
                  and self.lead_speed < LIMIT_SHARE * self.limit - OVERTAKE_MARGIN):
                # Only overtake something clearly slower than we may drive (§5).
                self.signal(self.left_shift, 'PASS', 'LEFT')

        elif self.state.startswith('SIGNAL_'):
            goal, next_state = self.after_signal
            moving_left = goal > self.goal_offset
            if moving_left and (not self.left_lane_free() or self.emergency_near()):
                self.set_state('CRUISE')                 # gap closed: stay where we are
            elif not moving_left and not self.home_lane_free():
                self.set_state('PASS')                   # right lane not free yet: wait
            elif now >= self.signal_until:
                self.goal_offset = goal
                if next_state == 'PASS':
                    # level with it after we have closed the gap (+ its length, + margin)
                    self.pass_until = self.travelled + self.overtaking_gap + 10.0
                    self.pass_started = now
                self.set_state('CHANGE_LEFT' if moving_left else 'CHANGE_RIGHT')

        elif self.state == 'CHANGE_LEFT':
            if self.left_shift > 0.0:
                self.goal_offset = self.left_shift          # follow the left lane as it is
            if abs(self.offset - self.goal_offset) < 0.5:
                self.set_state('PASS')

        elif self.state == 'PASS':
            if self.left_shift > 0.0:
                self.goal_offset = self.left_shift          # follow the left lane as it is
            # Back to the right as soon as we are past and the home lane is free (§2, §5),
            # or straight away when the left lane is about to end.
            past = self.travelled > self.pass_until or now - self.pass_started > MAX_PASS_TIME
            lane_ends = self.left_room < LEFT_ROOM_RETURN or self.left_shift == 0.0
            if lane_ends and self.home_lane_free():
                self.goal_offset = 0.0
                self.set_state('CHANGE_RIGHT')               # indicator is on while changing
            elif (past or self.emergency_near()) and self.home_lane_free():
                self.signal(0.0, 'CRUISE', 'RIGHT')

        elif self.state == 'CHANGE_RIGHT':
            if abs(self.offset - self.goal_offset) < 0.3:
                self.set_state('CRUISE')

        elif self.state == 'YIELD':
            if not self.emergency_near():
                self.goal_offset = 0.0
                self.set_state('CRUISE')

    # ---------- speed ----------
    def safe_speed(self, distance):
        """Fastest speed from which we can still stop with a 2 s gap:
        v^2 / (2a) + T v + MIN_GAP = distance   ->   solve for v."""
        room = max(0.0, distance - MIN_GAP)
        a, t = BRAKE_DECEL, TIME_GAP
        return a * (-t + math.sqrt(t * t + 2.0 * room / a))

    def target_speed(self):
        # "Auf Sicht" (§3): able to stop within the distance our sensors can see.
        sight = math.sqrt(2.0 * SIGHT_DECEL * (SENSOR_RANGE - MIN_GAP))
        # Following (§4): the car ahead also needs room to stop, so count its braking distance.
        room = self.obstacle + self.lead_speed ** 2 / (2.0 * BRAKE_DECEL)
        speed = min(LIMIT_SHARE * self.limit, sight, self.safe_speed(room))
        # A lower limit ahead (§3): be down to it by the time we reach the sign.
        new_limit, sign = self.upcoming_limit
        if math.isfinite(sign):
            speed = min(speed, math.sqrt((LIMIT_SHARE * new_limit) ** 2 + 2.0 * BRAKE_DECEL * max(0.0, sign - 5.0)))
        if self.state == 'YIELD':
            speed = min(speed, YIELD_SHARE * self.limit)
        # Traffic lights (§37): stop at red; at amber stop only if we comfortably can.
        if self.light and math.isfinite(self.stop_line):
            can_stop = self.speed ** 2 / (2.0 * AMBER_DECEL) < self.stop_line
            if self.light == 2 or can_stop:
                # stop_line is measured from the middle of the car; our bumper is 2.4 m ahead of that
                speed = min(speed, math.sqrt(2.0 * BRAKE_DECEL * max(0.0, self.stop_line - FRONT - 1.5)))
        # No passing on the right (§5): when catching a car in the left lane, match it.
        if self.state == 'CRUISE' and self.left_ahead < 25.0 and self.left_ahead < self.left_ahead_before:
            speed = min(speed, self.speed)
        return speed

    # ---------- every 50 ms ----------
    def tick(self):
        cmd = CarlaEgoVehicleControl()
        if not self.have_lane:
            cmd.brake = 1.0          # no lane information yet: stay put
            self.pub.publish(cmd)
            return

        self.travelled += self.speed * DT
        self.decide()
        self.state_pub.publish(String(data=self.state))

        # Slide the aiming point sideways toward the goal, a little each step.
        step = SHIFT_RATE * DT
        self.target_offset += max(-step, min(step, self.goal_offset - self.target_offset))

        # Steering: Stanley controller + curve feed-forward.
        error = self.offset - self.target_offset
        wheel_angle = (self.heading_error
                       + math.atan2(K_OFFSET * error, self.speed + 1.0)
                       - math.atan(WHEELBASE * self.curvature))
        cmd.steer = max(-1.0, min(1.0, wheel_angle / MAX_WHEEL_ANGLE))

        # Speed: proportional throttle / brake toward the target speed.
        target = self.target_speed()
        speed_error = target - self.speed
        if target < 0.1 and self.speed < 0.5:
            cmd.brake = 1.0
        elif speed_error >= 0.0:
            cmd.throttle = min(0.7, K_SPEED * speed_error)
        else:
            cmd.brake = min(1.0, -K_SPEED * speed_error)
        self.pub.publish(cmd)

        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(
                f'{self.state:12s} speed {self.speed * 3.6:5.1f} km/h (limit {self.limit * 3.6:3.0f})   '
                f'obstacle {self.obstacle:5.1f} m   offset {self.offset:+.2f} m   '
                f'lanes own {self.own_ahead:5.1f}/{self.own_behind:5.1f} left {self.left_ahead:5.1f}/{self.left_behind:5.1f}')


def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
