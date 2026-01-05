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
        (marker_detected ?m - marker)
        (marker_processed ?m - marker)
        (is_first ?m - marker)
        (precedes ?m1 ?m2 - marker)
    )

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

    (:durative-action detect_marker
        :parameters (?r - robot ?m - marker ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (over all (robot_at ?r ?wp))
            (at start (marker_at ?m ?wp))
        )
        :effect (and
            (at end (marker_detected ?m))
        )
    )

    (:durative-action process_first
        :parameters (?r - robot ?m - marker ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (at start (marker_detected ?m))
            (over all (robot_at ?r ?wp))
            (over all (marker_at ?m ?wp))
            (at start (is_first ?m))
        )
        :effect (and
            (at end (marker_processed ?m))
        )
    )
)