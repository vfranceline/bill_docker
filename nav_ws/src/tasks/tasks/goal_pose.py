#!/usr/bin/ env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.task import Future

class NavigationActionClient(Node): 
    def __init__(self):
        super().__init__('navigation_action_client') 

        # Create the action client for 'NavigateToPose'
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create a future to handle completion
        self._goal_done_future = Future()
    
    def send_goal(self, pose: PoseStamped):
        # Create a goal
        goal_msg = NavigateToPose.Goal()

        # Set the target pose (the goal for navigation)
        goal_msg.pose = pose

        # Wait for the action server to be ready
        self._action_client.wait_for_server()

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Wait for the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_future_callback)

    def get_result_future_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

        # Mark the goal as done
        self._goal_done_future.set_result(True)
    
def main(args=None):
    rclpy.init(args=args)

    # Create the action client node
    navigation_client = NavigationActionClient() 

    # Define a target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    target_pose.pose.position.x = 2.15788
    target_pose.pose.position.y = -1.52611
    target_pose.pose.position.z = 0.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = -0.576843
    target_pose.pose.orientation.w = 0.816855

    # Send the goal
    navigation_client.send_goal(target_pose)

    # Spin the node until the goal is done
    rclpy.spin_until_future_complete(navigation_client, navigation_client._goal_done_future)

    # Shut down the node once done
    navigation_client.get_logger().info("Navigation goal has been processes. Shutting down...")
    rclpy.shutdown()

if __name__ == '__main__':
    main()