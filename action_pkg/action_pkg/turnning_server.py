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


direction_dict = {0: -1.576672, 1: 3.133561, 2: 1.578764, 3: -0.003896}
direction_str_dict = {0: 'Up', 1: 'Right', 2: 'Down', 3: 'Left'}

class TurnningServer(Node):

    def __init__(self):
        super().__init__('turnning_server')

        self.yaw = 0.0

        self.twist_msg = Twist()

        self.odom_sub = self.create_subscription(
            Odometry, 'diffbot/odom', self.odom_sub_cb, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'diffbot/cmd_vel', 10)

        timer_period = 10
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self._action_server = ActionServer(
            self, Maze, 'turnning_server', self.execute_callback
        )

        self.get_logger().info('=== Turnning Server Started ===')

    def odom_sub_cb(self, data):
        orientation = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)

    def publish_callback(self):
        self.cmd_vel_pub.publish(self.twist_msg)

    def turn_robot(self, euler_angle):
        self.get_logger().info(f'Robot Turns to {euler_angle}')

        turn_offset = 100

        while abs(turn_offset) > 0.087:
            turn_offset = 0.5 * (euler_angle - self.yaw)
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = turn_offset
            self.cmd_vel_pub.publish(self.twist_msg)
            # self.stop_robot()

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

        for _, val in enumerate(goal_handle.request.turning_sequence):
            self.get_logger().info(f'Current Cmd: {val}')

            feedback.feedback_msg = f'Truning {direction_str_dict[val]}'

            self.turn_robot(direction_dict[val])

            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()

        result = Maze.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    try:
        maze_action_server = TurnningServer()
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