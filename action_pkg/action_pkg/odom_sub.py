import cv2

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from transforms3d.euler import quat2euler


class OdometrySubscriber(Node):

    def __init__(self):
        super().__init__("odom_subscriber")
        self.sub_period = 10

        self.subscription = self.create_subscription(
            Odometry,
            "diffbot/odom",
            self.listener_callback,
            self.sub_period
        )
        self.subscription

    def listener_callback(self, data):
        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x ,orientation.y, orientation.z, orientation.w]
        _, _, self._yaw = quat2euler(orientation_list)
        self.get_logger().info(f"Current Yaw Angle : {self._yaw}")

def main(args=None):
    rclpy.init(args=args)
    Odometry_subscriber = OdometrySubscriber()
    rclpy.spin(Odometry_subscriber)
    Odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()