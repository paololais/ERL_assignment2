(define (domain assignment2_domain)
    (:requirements :durative-actions :equality :strips :typing)

    (:types
        robot
        waypoint
        marker
    )

    (:predicates
        (robot_at ?r - robot ?wp - waypoint)
        (marker_at ?m - marker ?wp - waypoint)
        (visited ?wp - waypoint)
        (marker_detected ?m - marker)
        (marker_processed ?m - marker)
        (exploration_done)
        (precedes ?m1 ?m2 - marker)
        (is_first ?m - marker)
    )

    ;; 1. NAVIGAZIONE "LIBERA"
    ;; Rimuovendo (connected ?from ?to), permettiamo il salto diretto
    (:durative-action navigate
        :parameters (?r - robot ?from ?to - waypoint)
        :duration (= ?duration 10)
        :condition (and
            (at start (robot_at ?r ?from))
            (at start (not (= ?from ?to)))
        )
        :effect (and
            (at start (not (robot_at ?r ?from)))
            (at end (robot_at ?r ?to))
        )
    )

    ;; 2. DETECT
    (:durative-action detect_marker
        :parameters (?r - robot ?m - marker ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (over all (robot_at ?r ?wp))
            (at start (marker_at ?m ?wp))
        )
        :effect (and
            (at end (marker_detected ?m))
            (at end (visited ?wp))
        )
    )

    ;; FINALIZE 
    (:durative-action finalize_exploration
        :parameters (?m1 ?m2 ?m3 ?m4 - marker)
        :duration (= ?duration 1)
        :condition (and
            (at start (marker_detected ?m1))
            (at start (marker_detected ?m2))
            (at start (marker_detected ?m3))
            (at start (marker_detected ?m4))
            
            (at start (not (= ?m1 ?m2))) (at start (not (= ?m1 ?m3))) (at start (not (= ?m1 ?m4)))
            (at start (not (= ?m2 ?m3))) (at start (not (= ?m2 ?m4)))
            (at start (not (= ?m3 ?m4)))
        )
        :effect (and
            (at end (exploration_done)) ;; da ora puo process
        )
    )

    ;;  PROCESS FIRST
    (:durative-action process_first
        :parameters (?r - robot ?m1 ?m2 ?m3 ?m4 - marker ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (at start (marker_detected ?m1))
            (at start (marker_detected ?m2))
            (at start (marker_detected ?m3))
            (at start (marker_detected ?m4))
            (over all (robot_at ?r ?wp))
            (over all (marker_at ?m1 ?wp))
            (at start (is_first ?m1))
        )
        :effect (and
            (at end (marker_processed ?m1))
        )
    )

    ;; PROCESS NEXT
    (:durative-action process_next
        :parameters (?r - robot ?m_prev ?m_curr - marker ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (over all (robot_at ?r ?wp))
            (over all (marker_at ?m_curr ?wp))
            (at start (marker_detected ?m_curr))
            (at start (precedes ?m_prev ?m_curr))
            (at start (marker_processed ?m_prev))
        )
        :effect (and
            (at end (marker_processed ?m_curr))
        )
    )
)