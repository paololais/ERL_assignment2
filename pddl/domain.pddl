(define (domain aruco_mission)
  (:requirements :strips :typing :durative-actions :fluents)
  
  (:types
    waypoint marker robot
  )
  
  (:predicates
    (robot_at ?r - robot ?w - waypoint)
    (marker_at ?m - marker ?w - waypoint)
    (waypoint_visited ?w - waypoint)
    (marker_detected ?m - marker)
    (picture_taken ?m - marker)
    (all_markers_discovered)
    (mission_complete)
  )
  
  (:functions
    (marker_id ?m - marker)
  )
  
  ;; Action 1: Navigate to waypoint for exploration
  (:durative-action move_to_waypoint
    :parameters (?r - robot ?from ?to - waypoint)
    :duration (= ?duration 15)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (not (= ?from ?to)))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (waypoint_visited ?to))
    )
  )
  
  ;; Action 2: Detect marker at current waypoint
  (:durative-action detect_marker
    :parameters (?r - robot ?m - marker ?w - waypoint)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?w))
      (at start (marker_at ?m ?w))
      (at start (not (marker_detected ?m)))
    )
    :effect (and
      (at end (marker_detected ?m))
    )
  )
  
  ;; Action 3: Take picture of marker (must be in order by ID)
  (:durative-action take_picture
    :parameters (?r - robot ?m - marker ?w - waypoint)
    :duration (= ?duration 3)
    :condition (and
      (at start (robot_at ?r ?w))
      (at start (marker_at ?m ?w))
      (at start (marker_detected ?m))
      (at start (not (picture_taken ?m)))
      (at start (all_markers_discovered))
    )
    :effect (and
      (at end (picture_taken ?m))
    )
  )
)
