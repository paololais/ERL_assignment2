#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.action import ActionClient
from plansys2_executor.ActionExecutorClient import ActionExecutorClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.msg import Transition
from assignment2_interfaces.action import NavigateToWaypoint  # I tuoi action devono essere in un package interfaces
import math


class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__('move_to_waypoint', 500)
        
        self.goal_sent_ = False
        self.progress_ = 0.0
        self.start_x_ = 0.0
        self.start_y_ = 0.0
        self.current_x_ = 0.0
        self.current_y_ = 0.0
        
        # Waypoints dal progetto
        self.waypoints_ = {
            'wp1': {'x': -6.0, 'y': -6.0, 'theta': 0.0},
            'wp2': {'x': -6.0, 'y':  6.0, 'theta': 0.0},
            'wp3': {'x':  6.0, 'y': -6.0, 'theta': 0.0},
            'wp4': {'x':  6.0, 'y':  6.0, 'theta': 0.0}
        }
        
        self.goal_tolerance_ = 0.6
        
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
        
        self.get_logger().info('MoveAction initialized')
    
    def do_work(self):
        """Metodo chiamato da PlanSys2 per eseguire l'action"""
        args = self.get_arguments()
        
        # args = ['move_to_waypoint', 'robot1', 'wp1', 'wp2']
        # args[0] = action name
        # args[1] = robot
        # args[2] = from waypoint
        # args[3] = to waypoint
        
        if len(args) < 4:
            self.get_logger().error(f'Not enough arguments: {args}')
            self.finish(False, 0.0, 'Insufficient arguments')
            return
        
        wp_from = args[2]
        wp_to = args[3]
        
        self.get_logger().info(f'Moving from {wp_from} to {wp_to}')
        
        # Get target waypoint coordinates
        if wp_to not in self.waypoints_:
            self.get_logger().error(f'Unknown waypoint: {wp_to}')
            available = ', '.join(self.waypoints_.keys())
            self.finish(False, 0.0, f'Unknown waypoint. Available: {available}')
            return
        
        waypoint_data = self.waypoints_[wp_to]
        goal_x = waypoint_data['x']
        goal_y = waypoint_data['y']
        
        if not self.goal_sent_:
            # Wait for action server
            if not self.nav2_client_.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn('NavigateToPose server not ready')
                return
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            
            # Create goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            
            # Send goal
            self.get_logger().info(f'Sending navigation goal to ({goal_x}, {goal_y})')
            self.nav2_future_ = self.nav2_client_.send_goal_async(goal_msg)
            self.nav2_future_.add_done_callback(
                lambda future: self.goal_response_callback(future, wp_to)
            )
            
            self.goal_sent_ = True
            self.start_x_ = self.current_x_
            self.start_y_ = self.current_y_
        
        # Calculate progress
        total_dist = math.hypot(goal_x - self.start_x_, goal_y - self.start_y_)
        rem_dist = math.hypot(goal_x - self.current_x_, goal_y - self.current_y_)
        
        if total_dist > 0.0:
            self.progress_ = max(0.0, min(1.0 - (rem_dist / total_dist), 1.0))
        else:
            self.progress_ = 1.0
        
        self.send_feedback(self.progress_, f'Moving to {wp_to} ({self.progress_*100:.1f}%)')
        
        # Check if reached goal
        if rem_dist < self.goal_tolerance_:
            self.goal_sent_ = False
            self.progress_ = 1.0
            self.send_feedback(1.0, f'Reached {wp_to}')
            self.get_logger().info(f'✓ Reached waypoint: {wp_to}')
            self.finish(True, 1.0, f'Successfully moved to {wp_to}')
        
        # Spin the nav2 node
        rclpy.spin_once(self.nav2_node_, timeout_sec=0)
    
    def goal_response_callback(self, future, wp_to_navigate):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Navigation goal rejected for {wp_to_navigate}')
            self.finish(False, 0.0, 'Goal rejected')
            return
        
        self.get_logger().info(f'Navigation goal accepted for {wp_to_navigate}')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.result_callback(future, wp_to_navigate)
        )
    
    def result_callback(self, future, wp_to_navigate):
        result = future.result()
        status = result.status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Nav2 reports success for {wp_to_navigate}')
        else:
            self.get_logger().error(f'Nav2 failed with status {status} for {wp_to_navigate}')
    
    def odom_callback(self, msg):
        self.current_x_ = msg.pose.pose.position.x
        self.current_y_ = msg.pose.pose.position.y


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveAction()
    
    node.set_parameters([
        rclpy.parameter.Parameter(
            'action_name', 
            rclpy.Parameter.Type.STRING, 
            'move_to_waypoint'
        )
    ])
    
    node.trigger_transition(Transition.TRANSITION_CONFIGURE)
    node.trigger_transition(Transition.TRANSITION_ACTIVATE)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
