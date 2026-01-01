#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from plansys2_domain_expert.DomainExpertClient import DomainExpertClient
from plansys2_planner.PlannerClient import PlannerClient
from plansys2_problem_expert.ProblemExpertClient import ProblemExpertClient
from plansys2_pddl_parser import Utils


def print_plan(plan):
    """Stampa il piano in formato leggibile"""
    print("Plan:")
    for item in plan.items:
        print(f"  Action: {item.action}")
        print(f"  Duration: {item.duration}")
        print(f"  Time: {item.time}")


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.domain_expert_ = None
        self.planner_client_ = None
        self.problem_expert_ = None
    
    def init(self):
        """Inizializza i client per PlanSys2"""
        self.domain_expert_ = DomainExpertClient()
        self.planner_client_ = PlannerClient()
        self.problem_expert_ = ProblemExpertClient()
    
    def plan(self):
        """Genera e stampa il piano"""
        # Ottieni dominio e problema
        domain = self.domain_expert_.get_domain()
        problem = self.problem_expert_.get_problem()
        
        # Richiedi il piano
        plan = self.planner_client_.get_plan(domain, problem)
        
        if plan is None:
            # Se il piano non è stato trovato
            goal = self.problem_expert_.get_goal()
            goal_str = Utils.to_string(goal)
            print(f"Could not find plan to reach goal {goal_str}")
        else:
            # Stampa il piano trovato
            print_plan(plan)


def main(args=None):
    rclpy.init(args=args)
    
    node = Controller()
    node.init()
    node.plan()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()