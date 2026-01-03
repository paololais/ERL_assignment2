(define (domain assignment2_domain)
    (:requirements :strips :typing :durative-actions :equality)

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
        (exploration_done)
        (processing_enabled ?m - marker)
        (precedes ?m1 ?m2 - marker)
        (is_first ?m - marker)
        (is_last ?m - marker)
    )

    ;; 1. NAVIGAZIONE "LIBERA"
    ;; Rimuovendo (connected ?from ?to), permettiamo il salto diretto
    (:durative-action navigate
        :parameters (?r - robot ?from ?to - waypoint)
        :duration (= ?duration 10)
        :condition (and
            (at start (robot_at ?r ?from))
            ;; Ho rimosso il vincolo connected: ora può andare ovunque
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
        )
    )

    ;; 3. FINALIZE (Con Anti-Cheat per evitare alias)
    (:durative-action finalize_detection_phase
        :parameters (?m1 ?m2 ?m3 ?m4 - marker)
        :duration (= ?duration 1)
        :condition (and
            (at start (marker_detected ?m1))
            (at start (marker_detected ?m2))
            (at start (marker_detected ?m3))
            (at start (marker_detected ?m4))
            
            ;; Constraints per assicurare che siano 4 marker diversi
            (at start (not (= ?m1 ?m2)))
            (at start (not (= ?m1 ?m3)))
            (at start (not (= ?m1 ?m4)))
            (at start (not (= ?m2 ?m3)))
            (at start (not (= ?m2 ?m4)))
            (at start (not (= ?m3 ?m4)))

            (at start (is_first ?m1)) 
        )
        :effect (and
            (at end (exploration_done))
            (at end (processing_enabled ?m1)) 
        )
    )

    ;; 4. PROCESS IMAGE
    (:durative-action process_image_sequence
        :parameters (?r - robot ?m_curr ?m_next - marker ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (at start (exploration_done))
            (over all (robot_at ?r ?wp))
            (over all (marker_at ?m_curr ?wp))
            (at start (processing_enabled ?m_curr))
            (at start (precedes ?m_curr ?m_next))
        )
        :effect (and
            (at end (marker_processed ?m_curr))
            (at end (not (processing_enabled ?m_curr)))
            (at end (processing_enabled ?m_next))
        )
    )

    ;; 5. PROCESS LAST IMAGE
    (:durative-action process_last_image
        :parameters (?r - robot ?m - marker ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (at start (exploration_done))
            (over all (robot_at ?r ?wp))
            (over all (marker_at ?m ?wp))
            (at start (processing_enabled ?m))
            (at start (is_last ?m))
        )
        :effect (and
            (at end (marker_processed ?m))
            (at end (not (processing_enabled ?m)))
        )
    )
)