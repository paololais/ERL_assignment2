(define (problem assignment2_task)
    (:domain assignment2_domain)

    (:objects
        r1 - robot
        wp_1 wp_2 wp_3 wp_4 start_pos - waypoint
        marker_1 marker_2 marker_3 marker_4 - marker
    )

    (:init
        ;; Posizione Robot
        (robot_at r1 start_pos) ;; Assumiamo parta da uno dei waypoint o aggiungi start_pos

        ;; HINTS (Dove sono i marker secondo l'assignment [cite: 15])
        (marker_at marker_1 wp_1)
        (marker_at marker_2 wp_2)
        (marker_at marker_3 wp_3)
        (marker_at marker_4 wp_4)
    )

    ;; GOAL FASE 1: Trova tutto.
    ;; Non chiediamo ancora di processarli, perché non sappiamo l'ordine.
    (:goal (and
        (marker_detected marker_1)
        (marker_detected marker_2)
        (marker_detected marker_3)
        (marker_detected marker_4)
    ))
)