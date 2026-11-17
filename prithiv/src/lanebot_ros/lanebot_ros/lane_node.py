"""Where is our lane? Uses CARLA's road map.

Publishes /lanebot/lane = [offset, heading_error, curvature, left_shift,
                           left_allowed, lane_width]
  offset        m the car is left (+) of our lane's centre
  heading_error rad, car direction minus lane direction
  curvature     1/m, how fast the lane turns over the next 5 m (+ = left)
  left_shift    m to the centre of the lane on our left (0 = no such lane)
  left_allowed  1 if we may change into it here (broken line, not a junction)
  lane_width    m
  left_room     m ahead for which the left lane continues (up to 250 m)
"""
import math

import carla
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def wrap(angle):
    """Bring an angle into the range -pi..pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def left_driving_lane(wp):
    """The lane on our left going our way, or None."""
    side = wp.get_left_lane()
    if side is not None and side.lane_type == carla.LaneType.Driving and side.lane_id * wp.lane_id > 0:
        return side
    return None


def choose(current, options):
    """At a fork: stay on the motorway (a road with a lane to our left), then go straightest.

    Taking the straightest branch alone led us down an exit ramp from the right lane.
    """
    yaw = current.transform.rotation.yaw
    return min(options, key=lambda w: (left_driving_lane(w) is None,
                                       abs(wrap(math.radians(w.transform.rotation.yaw - yaw)))))


def ros_pose(wp):
    """Position and direction of a waypoint in ROS coordinates (y to the left)."""
    t = wp.transform
    return t.location.x, -t.location.y, -math.radians(t.rotation.yaw)


class LaneNode(Node):
    def __init__(self):
        super().__init__('lane_node')
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.map = client.get_world().get_map()
        self.ref = None   # the waypoint on OUR route that the car is next to

        self.pub = self.create_publisher(Float32MultiArray, '/lanebot/lane', 10)
        self.sub = self.create_subscription(
            Odometry, '/carla/ego_vehicle/odometry', self.on_odom, 10)
        self.count = 0

    def left_room(self):
        """How far ahead the lane on our left continues (lanes end at merges and exits)."""
        wp, room = self.ref, 0.0
        while room < 250.0 and left_driving_lane(wp) is not None:
            options = wp.next(5.0)
            if not options:
                break
            wp, room = choose(wp, options), room + 5.0
        return room

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # First message only: start the route in the lane the car is in.
        if self.ref is None:
            self.ref = self.map.get_waypoint(carla.Location(x=x, y=-y, z=0.0))

        # Move the reference forward along the route until it is level with the car.
        for _ in range(50):
            cx, cy, lane_yaw = ros_pose(self.ref)
            ahead = math.cos(lane_yaw) * (x - cx) + math.sin(lane_yaw) * (y - cy)
            if ahead < 0.5:
                break
            options = self.ref.next(0.5)
            if not options:
                break
            self.ref = choose(self.ref, options)

        offset = -math.sin(lane_yaw) * (x - cx) + math.cos(lane_yaw) * (y - cy)
        heading_error = wrap(yaw - lane_yaw)

        # Curvature: how fast the lane direction changes over the next 5 m.
        curvature = 0.0
        options = self.ref.next(5.0)
        if options:
            _, _, yaw_ahead = ros_pose(choose(self.ref, options))
            curvature = wrap(yaw_ahead - lane_yaw) / 5.0

        # The lane on our left, and whether the rules let us change into it here:
        # only across a broken line, never inside a junction.
        left_shift, left_allowed = 0.0, 0.0
        side = left_driving_lane(self.ref)
        if side is not None:
            sx, sy, _ = ros_pose(side)
            left_shift = -math.sin(lane_yaw) * (sx - cx) + math.cos(lane_yaw) * (sy - cy)
            broken = self.ref.left_lane_marking.type in (carla.LaneMarkingType.Broken,
                                                          carla.LaneMarkingType.BrokenBroken,
                                                          carla.LaneMarkingType.BrokenSolid)
            permitted = self.ref.lane_change in (carla.LaneChange.Left, carla.LaneChange.Both)
            left_allowed = float(broken and permitted and not self.ref.is_junction)

        out = Float32MultiArray()
        out.data = [offset, heading_error, curvature, left_shift, left_allowed, self.ref.lane_width,
                    self.left_room()]
        self.pub.publish(out)

        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(
                f'offset {offset:+.2f} m   heading error {math.degrees(heading_error):+.1f} deg')


def main():
    rclpy.init()
    node = LaneNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
