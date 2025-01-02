import math
import time
import numpy as np

from custom_interfaces.action import Maze
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from action_pkg.image_sub import ImageSubscriber

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def is_yellowgreen(pixel):
        h, s, v = pixel

        lower_green = np.array([50, 100, 100])
        upper_green = np.array([70, 255, 255])

        if lower_green[0] <= h <= upper_green[0] and lower_green[1] <= s <= upper_green[1] and lower_green[2] <= v <= upper_green[2]:
            return True
        else:
            return False

direction_dict = {0: math.pi, 1: math.pi / 2, 2: 0.0000, 3: -1 * math.pi / 2}
direction_str_dict = {0: 'Up', 1: 'Right', 2: 'Down', 3: 'Left'}



class MazeActionServer(Node):

    def __init__(self):
        super().__init__('maze_action_server')

        self.yaw = 0.0
        self.forward_distance = 0.0

        self.twist_msg = Twist()
        self.loop_rate = self.create_rate(5, self.get_clock())

        self.laser_sub = self.create_subscription(
            LaserScan, 'diffbot/scan', self.laser_sub_cb, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, 'diffbot/odom', self.odom_sub_cb, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'diffbot/cmd_vel', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self._action_server = ActionServer(
            self, Maze, 'maze_action', self.execute_callback
        )
        self.get_logger().info('=== Maze Action Server Started ===')

        self.current_direction = 2

    def laser_sub_cb(self, data):
        self.forward_distance = data.ranges[360]

    def odom_sub_cb(self, data):
        orientation = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)

    def publish_callback(self):
        self.cmd_vel_pub.publish(self.twist_msg)

    def turn_robot(self, euler_angle):

        turn_offset = 100

        while abs(turn_offset) > 0.055:
            turn_offset = 0.4 * (euler_angle - self.yaw)
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = turn_offset
            self.cmd_vel_pub.publish(self.twist_msg)

        self.stop_robot()

    def parking_robot(self):

        while self.forward_distance > 1.0:
            self.twist_msg.linear.x = 0.5
            self.twist_msg.angular.x = 0.0

            self.cmd_vel_pub.publish(self.twist_msg)

        self.stop_robot()

    def stop_robot(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

        time.sleep(1)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback = Maze.Feedback()
        feedback.feedback_msg = ''
        image_sub_node = ImageSubscriber()

        while True:
            self.parking_robot()
            rclpy.spin_once(image_sub_node)

            center_pixel = image_sub_node.center_pixel
            if is_yellowgreen(center_pixel):
                goal_handle.succeed()
                self.get_logger().warn('=== Succeed ===')
                result = Maze.Result()
                result.success = True
                break

            direction = image_sub_node.detections

            if direction == 'left':
                self.current_direction -= 1
            elif direction == 'right':
                self.current_direction += 1

            self.turn_robot(direction_dict[self.current_direction])
            feedback.feedback_msg = f'Truning {direction}'
            goal_handle.publish_feedback(feedback)

            center_pixel = image_sub_node.center_pixel

        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        maze_action_server = MazeActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(maze_action_server)
        try:
            executor.spin()
        except KeyboardInterrupt:
            maze_action_server.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            maze_action_server._action_server.destroy()
            maze_action_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()