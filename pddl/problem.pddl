(define (problem assignment2_task)
    (:domain assignment2_domain)

    (:objects
        r1 - robot
        start_pos wp_1 wp_2 wp_3 wp_4 - waypoint
        marker_1 marker_2 marker_3 marker_4 - marker
    )

    (:init
        (robot_at r1 start_pos)

        ;; HINTS Posizioni Marker
        (marker_at marker_1 wp_1)
        (marker_at marker_2 wp_2)
        (marker_at marker_3 wp_3)
        (marker_at marker_4 wp_4)

        ;; SEQUENZA FASE 2
        (is_first marker_1)
        (precedes marker_1 marker_2)
        (precedes marker_2 marker_3)
        (precedes marker_3 marker_4)
        (is_last marker_4)
    )

    (:goal (and
        (marker_processed marker_1)
        (marker_processed marker_2)
        (marker_processed marker_3)
        (marker_processed marker_4)
        (robot_at r1 start_pos)
    ))
)