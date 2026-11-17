import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl


class DriveStraight(Node):
    def __init__(self):
        super().__init__('drive_straight')
        # A publisher sends messages of one type on one topic.
        self.pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        # A timer calls self.tick 20 times per second.
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        msg = CarlaEgoVehicleControl()
        msg.throttle = 0.4   # 0.0 to 1.0
        msg.steer = 0.0      # -1.0 (left) to 1.0 (right)
        msg.brake = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DriveStraight()
    try:
        rclpy.spin(node)   # keep running until Ctrl+C
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
