from custom_interfaces.action import Maze
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class MazeActionClient(Node):

    def __init__(self):
        super().__init__('maze_action_client')
        self.action_client = ActionClient(self, Maze, 'maze_action')
        self.get_logger().info("=== Maze Action Client Started ===")

    def send_goal(self, go_sign):
        goal_msg = Maze.Goal()
        goal_msg.gogo = go_sign

        if self.action_client.wait_for_server(10) is False:
            self.get_logger().error('Server Not exists')
            return

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(f'Received feedback: {feedback.feedback_msg}')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().warn(f'Action Done !! Result: {result.success}')
        rclpy.shutdown()

def main(args=None):

    rclpy.init(args=args)
    maze_action_client = MazeActionClient()

    maze_action_client.get_logger().info('Please let me go : ')

    user_inputs = input()

    if 'go' in user_inputs:
        maze_action_client.get_logger().info('=== Sending Goal ===')
        maze_action_client.send_goal(user_inputs)
    else:
        maze_action_client.destroy_node()
        rclpy.shutdown()
    
if __name__ =='__main__':
    main()




"""def main(args=None):
    rclpy.init(args=args)

    maze_action_client = MazeActionClient()
    user_inputs = []

    try:
        maze_action_client.get_logger().info('Enter numbers [or stop] : ')

        while True:
            user_inputs.append(int(input()))
    except Exception:
        maze_action_client.get_logger().info(f'Your sequence list : {user_inputs}')
    maze_action_client.get_logger().info('=== Sending Goal ===')
    maze_action_client.send_goal(user_inputs)

    rclpy.spin(maze_action_client)
"""