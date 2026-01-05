(define (problem assignment2_task)
    (:domain assignment2_domain)

    (:objects
        r1 - robot
        wp_1 wp_3 start_pos - waypoint
        marker_1 - marker
    )

    (:init
        (robot_at r1 start_pos)
        (marker_at marker_1 wp_3)
    )

    (:goal 
        (and
            (marker_detected marker_1)
        )
    )
)