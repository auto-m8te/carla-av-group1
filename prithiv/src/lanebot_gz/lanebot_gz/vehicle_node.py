"""Turn CARLA-style driving commands into movement of the Gazebo car.

Your control node publishes throttle, brake and steer, as it does for CARLA.
This node plays the part of the car's engine and steering:
  * throttle and brake change the speed (a simple model with air drag),
  * steer sets the front-wheel angle, and the turn rate follows from the
    bicycle model:  turn rate = speed * tan(wheel angle) / wheelbase.
It sends speed and turn rate to Gazebo, and publishes the speed on the same
speedometer topic the CARLA bridge uses.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl

DT = 0.02               # s, 50 updates per second
MAX_WHEEL_ANGLE = 1.2   # rad at steer = 1.0, the same number the control node assumes
WHEEL_LIMIT = 0.6       # rad, a real car cannot turn its wheels further
WHEELBASE = 2.9         # m
ENGINE = 5.0            # m/s^2 at full throttle
BRAKES = 8.0            # m/s^2 at full brake
DRAG = 0.3              # 1/s, slows the car in proportion to speed
TIMEOUT = 0.5           # s without commands before we brake on our own


class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')
        self.speed = 0.0
        self.cmd = CarlaEgoVehicleControl()
        self.cmd.brake = 1.0
        self.last_cmd = self.get_clock().now()

        self.create_subscription(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', self.on_cmd, 10)
        self.twist_pub = self.create_publisher(Twist, '/model/ego/cmd_vel', 10)
        self.speed_pub = self.create_publisher(Float32, '/carla/ego_vehicle/speedometer', 10)
        self.create_timer(DT, self.tick)

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd = self.get_clock().now()

    def tick(self):
        cmd = self.cmd
        if (self.get_clock().now() - self.last_cmd).nanoseconds > TIMEOUT * 1e9:
            cmd = CarlaEgoVehicleControl()
            cmd.brake = 1.0

        accel = ENGINE * cmd.throttle - DRAG * self.speed
        if self.speed > 0.0:
            accel -= BRAKES * cmd.brake
        self.speed = max(0.0, self.speed + accel * DT)

        wheel = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, cmd.steer * MAX_WHEEL_ANGLE))
        # CARLA: steer > 0 turns right.  ROS: angular.z > 0 turns left.  Hence the minus.
        turn_rate = -self.speed * math.tan(wheel) / WHEELBASE

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = turn_rate
        self.twist_pub.publish(twist)
        self.speed_pub.publish(Float32(data=self.speed))


def main():
    rclpy.init()
    node = VehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
