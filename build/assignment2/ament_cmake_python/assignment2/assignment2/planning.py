#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from plansys2_domain_expert.DomainExpertClient import DomainExpertClient
from plansys2_planner.PlannerClient import PlannerClient
from plansys2_problem_expert.ProblemExpertClient import ProblemExpertClient
from plansys2_executor.ExecutorClient import ExecutorClient
from plansys2_pddl_parser import Utils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


def print_plan(plan):
    """Stampa il piano in formato leggibile"""
    print("\n" + "="*50)
    print("PLAN GENERATED:")
    print("="*50)
    for idx, item in enumerate(plan.items, 1):
        print(f"{idx}. {item.action}")
        print(f"   Duration: {item.duration}s")
        print(f"   Start time: {item.time}s")
    print("="*50 + "\n")


class ArucoMissionController(Node):
    def __init__(self):
        super().__init__('aruco_mission_controller')
        
        # PlanSys2 clients
        self.domain_expert_ = None
        self.planner_client_ = None
        self.problem_expert_ = None
        self.executor_client_ = None
        
        # Mission state
        self.markers_map_ = {}  # {marker_id: waypoint}
        self.exploration_complete_ = False
        self.mission_phase_ = 'init'  # init, exploration, picture_taking, complete
        
        # ArUco detection
        self.bridge_ = CvBridge()
        self.aruco_dict_ = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params_ = cv2.aruco.DetectorParameters()
        
        # Camera subscription
        self.image_sub_ = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.current_image_ = None
        
        # Waypoints definition
        self.waypoints_ = ['wp1', 'wp2', 'wp3', 'wp4']
        
        self.get_logger().info('Aruco Mission Controller initialized')
    
    def init_plansys(self):
        """Inizializza i client per PlanSys2"""
        self.get_logger().info('Initializing PlanSys2 clients...')
        
        self.domain_expert_ = DomainExpertClient(self)
        self.planner_client_ = PlannerClient(self)
        self.problem_expert_ = ProblemExpertClient(self)
        self.executor_client_ = ExecutorClient(self)
        
        time.sleep(1)  # Aspetta che i servizi siano pronti
        self.get_logger().info('PlanSys2 clients ready!')
    
    def image_callback(self, msg):
        """Callback per processare immagini dalla camera"""
        try:
            self.current_image_ = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def detect_markers_in_image(self):
        """Rileva i marker ArUco nell'immagine corrente"""
        if self.current_image_ is None:
            return []
        
        gray = cv2.cvtColor(self.current_image_, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict_,
            parameters=self.aruco_params_
        )
        
        if ids is not None:
            return ids.flatten().tolist()
        return []
    
    def setup_exploration_problem(self):
        """Setup del problema PDDL per la fase di esplorazione"""
        self.get_logger().info('Setting up exploration problem...')
        
        # Clear previous problem
        self.problem_expert_.clear_knowledge()
        
        # Add instances
        self.problem_expert_.add_instance('robot', 'robot1')
        for wp in self.waypoints_:
            self.problem_expert_.add_instance('waypoint', wp)
        
        # Set initial state
        self.problem_expert_.add_predicate('(robot_at robot1 wp1)')
        
        # Set goal: visit all waypoints
        goal_str = '(and '
        for wp in self.waypoints_:
            goal_str += f'(waypoint_visited {wp}) '
        goal_str += ')'
        
        self.problem_expert_.set_goal(goal_str)
        
        self.get_logger().info('Exploration problem setup complete')
    
    def setup_picture_taking_problem(self):
        """Setup del problema PDDL per scattare foto ai marker in ordine"""
        self.get_logger().info('Setting up picture-taking problem...')
        
        # Clear previous problem
        self.problem_expert_.clear_knowledge()
        
        # Add instances
        self.problem_expert_.add_instance('robot', 'robot1')
        for wp in self.waypoints_:
            self.problem_expert_.add_instance('waypoint', wp)
        
        # Add markers found during exploration
        sorted_markers = sorted(self.markers_map_.keys())
        for marker_id in sorted_markers:
            marker_name = f'marker{marker_id}'
            self.problem_expert_.add_instance('marker', marker_name)
        
        # Set initial state
        current_wp = 'wp1'  # Assumiamo di essere tornati a wp1
        self.problem_expert_.add_predicate(f'(robot_at robot1 {current_wp})')
        
        # Add marker locations
        for marker_id, wp in self.markers_map_.items():
            marker_name = f'marker{marker_id}'
            self.problem_expert_.add_predicate(f'(marker_at {marker_name} {wp})')
            self.problem_expert_.add_predicate(f'(marker_detected {marker_name})')
        
        # All markers discovered
        self.problem_expert_.add_predicate('(all_markers_discovered)')
        
        # Set goal: take picture of all markers
        goal_str = '(and '
        for marker_id in sorted_markers:
            marker_name = f'marker{marker_id}'
            goal_str += f'(picture_taken {marker_name}) '
        goal_str += ')'
        
        self.problem_expert_.set_goal(goal_str)
        
        self.get_logger().info(f'Picture-taking problem setup complete for markers: {sorted_markers}')
    
    def generate_plan(self):
        """Genera il piano"""
        domain = self.domain_expert_.get_domain()
        problem = self.problem_expert_.get_problem()
        
        self.get_logger().info('Requesting plan from planner...')
        plan = self.planner_client_.get_plan(domain, problem)
        
        if plan is None:
            goal = self.problem_expert_.get_goal()
            goal_str = Utils.to_string(goal)
            self.get_logger().error(f"Could not find plan to reach goal: {goal_str}")
            return None
        
        print_plan(plan)
        return plan
    
    def execute_plan(self):
        """Esegue il piano usando l'Executor"""
        self.get_logger().info('Starting plan execution...')
        
        if not self.executor_client_.start_plan_execution():
            self.get_logger().error('Failed to start plan execution')
            return False
        
        # Monitor execution
        rate = self.create_rate(1)  # 1 Hz
        while self.executor_client_.execute_and_check_plan():
            rate.sleep()
            
            feedback = self.executor_client_.get_feedback()
            if feedback:
                self.get_logger().info(
                    f'Executing: {feedback.action} - '
                    f'Progress: {feedback.completion * 100:.1f}%'
                )
        
        result = self.executor_client_.get_result()
        
        if result.success:
            self.get_logger().info('✓ Plan execution completed successfully!')
            return True
        else:
            self.get_logger().error('✗ Plan execution failed')
            return False
    
    def run_exploration_phase(self):
        """Esegue la fase di esplorazione"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('PHASE 1: EXPLORATION - Visiting all waypoints')
        self.get_logger().info('='*60)
        
        self.mission_phase_ = 'exploration'
        
        # Setup and execute exploration
        self.setup_exploration_problem()
        plan = self.generate_plan()
        
        if plan is None:
            return False
        
        success = self.execute_plan()
        
        if success:
            # Simula la detection dei marker (in realtà dovresti farlo durante l'esecuzione)
            self.get_logger().info('\nDetecting markers at each waypoint...')
            time.sleep(2)
            
            # SIMULAZIONE: Assegna marker ai waypoints
            # In realtà questo dovrebbe essere fatto dalle action detect_marker
            self.markers_map_ = {
                11: 'wp1',  # Marker ID 11 at waypoint 1
                12: 'wp2',  # Marker ID 12 at waypoint 2
                13: 'wp3',  # Marker ID 13 at waypoint 3
                14: 'wp4'   # Marker ID 14 at waypoint 4
            }
            
            self.get_logger().info(f'\nMarkers discovered: {self.markers_map_}')
            self.exploration_complete_ = True
        
        return success
    
    def run_picture_taking_phase(self):
        """Esegue la fase di scatto foto in ordine di ID"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('PHASE 2: PICTURE TAKING - Visiting markers in order')
        self.get_logger().info('='*60)
        
        self.mission_phase_ = 'picture_taking'
        
        # Setup and execute picture taking
        self.setup_picture_taking_problem()
        plan = self.generate_plan()
        
        if plan is None:
            return False
        
        success = self.execute_plan()
        
        if success:
            self.mission_phase_ = 'complete'
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('✓✓✓ MISSION COMPLETED SUCCESSFULLY! ✓✓✓')
            self.get_logger().info('='*60)
        
        return success
    
    def run_mission(self):
        """Esegue l'intera missione"""
        self.get_logger().info('\n' + '🚀 '*20)
        self.get_logger().info('STARTING ARUCO MARKER MISSION')
        self.get_logger().info('🚀 '*20)
        
        # Initialize PlanSys2
        self.init_plansys()
        
        # Phase 1: Exploration
        if not self.run_exploration_phase():
            self.get_logger().error('Exploration phase failed!')
            return
        
        time.sleep(2)  # Pausa tra le fasi
        
        # Phase 2: Picture taking
        if not self.run_picture_taking_phase():
            self.get_logger().error('Picture taking phase failed!')
            return
        
        self.get_logger().info('\nAll markers photographed in order!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = ArucoMissionController()
    
    # Use MultiThreadedExecutor for handling callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    # Run mission in a separate thread
    import threading
    mission_thread = threading.Thread(target=controller.run_mission)
    mission_thread.start()
    
    # Spin to handle callbacks
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
