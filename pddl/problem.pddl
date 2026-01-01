(define (problem aruco_mission_problem)
  (:domain aruco_mission)
  
  (:objects
    robot1 - robot
    wp1 wp2 wp3 wp4 - waypoint
    marker1 marker2 marker3 marker4 - marker
  )
  
  (:init
    (robot_at robot1 wp1)
    
    ;; Waypoint coordinates (for your action nodes)
    ;; wp1: (-6.0, -6.0)
    ;; wp2: (-6.0, 6.0)
    ;; wp3: (6.0, -6.0)
    ;; wp4: (6.0, 6.0)
    
    ;; We don't know which marker is at which waypoint initially
    ;; This will be discovered during exploration
  )
  
  (:goal
    (and
      (waypoint_visited wp1)
      (waypoint_visited wp2)
      (waypoint_visited wp3)
      (waypoint_visited wp4)
      (forall (?m - marker)
        (implies (marker_detected ?m) (picture_taken ?m))
      )
    )
  )
)
