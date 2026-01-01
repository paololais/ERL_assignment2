#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from plansys2_executor.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.msg import Transition
import math


class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('move', 500)
        
        self.goal_sent_ = False
        self.progress_ = 0.0
        self.start_x_ = 0.0
        self.start_y_ = 0.0
        self.current_x_ = 0.0
        self.current_y_ = 0.0
        
        # Create odometry subscription
        self.odom_sub_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create separate node for Nav2 action client
        self.nav2_node_ = rclpy.create_node('move_action_nav2_client')
        self.nav2_client_ = ActionClient(
            self.nav2_node_,
            NavigateToPose,
            'navigate_to_pose'
        )
    
    def do_work(self):
        args = self.get_arguments()
        
        if len(args) < 3:
            self.get_logger().error('Not enough arguments for move action')
            self.finish(False, 0.0, 'Insufficient arguments')
            return
        
        wp_to_navigate = args[2]
        
        # Define waypoint coordinates
        if wp_to_navigate == 'bathroom':
            goal_x = 10.0
            goal_y = 5.0
        elif wp_to_navigate == 'bedroom':
            goal_x = 5.0
            goal_y = 6.0
        else:
            self.get_logger().error(f'Unknown waypoint: {wp_to_navigate}')
            self.finish(False, 0.0, 'Unknown waypoint')
            return
        
        if not self.goal_sent_:
            # Wait for action server
            if not self.nav2_client_.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn('NavigateToPose server not ready')
                return
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.orientation.w = 1.0
            
            # Create goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            
            # Send goal with result callback
            self.nav2_future_ = self.nav2_client_.send_goal_async(goal_msg)
            self.nav2_future_.add_done_callback(
                lambda future: self.goal_response_callback(future, wp_to_navigate)
            )
            
            self.goal_sent_ = True
            self.start_x_ = self.current_x_
            self.start_y_ = self.current_y_
        
        # Calculate progress
        total_dist = math.hypot(goal_x - self.start_x_, goal_y - self.start_y_)
        rem_dist = math.hypot(goal_x - self.current_x_, goal_y - self.current_y_)
        
        if total_dist > 0.0:
            self.progress_ = 1.0 - min(rem_dist / total_dist, 1.0)
        else:
            self.progress_ = 1.0
        
        self.send_feedback(self.progress_, f'Moving to {wp_to_navigate}')
        
        # Check if reached goal
        if rem_dist < 0.6:
            self.goal_sent_ = False
            self.progress_ = 1.0
            self.send_feedback(self.progress_, f'Moving to {wp_to_navigate}')
            self.get_logger().info(f'Reached waypoint: {wp_to_navigate}')
            self.finish(True, 1.0, 'Move completed')
        
        # Spin the nav2 node
        rclpy.spin_once(self.nav2_node_, timeout_sec=0)
    
    def goal_response_callback(self, future, wp_to_navigate):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {wp_to_navigate}')
            return
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.result_callback(future, wp_to_navigate)
        )
    
    def result_callback(self, future, wp_to_navigate):
        result = future.result()
        if result.status != 4:  # 4 = SUCCEEDED
            self.get_logger().error(f'Navigation failed: {wp_to_navigate}')
            self.finish(True, 1.0, 'Move failed')
    
    def odom_callback(self, msg):
        self.current_x_ = msg.pose.pose.position.x
        self.current_y_ = msg.pose.pose.position.y


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveAction()
    
    node.set_parameters([rclpy.parameter.Parameter('action_name', 
                                                    rclpy.Parameter.Type.STRING, 
                                                    'move')])
    node.trigger_transition(Transition.TRANSITION_CONFIGURE)
    node.trigger_transition(Transition.TRANSITION_ACTIVATE)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()