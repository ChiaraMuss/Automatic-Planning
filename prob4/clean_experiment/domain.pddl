;Header and description

(define (domain domain_name)

;remove requirements that are not needed
(:requirements :strips :typing :durative-actions :numeric-fluents)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    medical_unit inventory generic_location - location
    patient
    terrestrial_robot arial_robot - robot
    delivery_robot accompany_robot - terrestrial_robot
    )


(:predicates ;todo: define predicates here
        (connected ?x - location ?y - location)
        (robot_at ?r - robot ?l - location)

        ;; patient management
        (free_to_accompany ?r - accompany_robot)
        (accompanying_pat ?p - patient ?ar - accompany_robot)
        (patient_at ?p - patient ?l - location)

    )


(:functions ;todo: define numeric functions here
        (speed ?r - robot)
        (distance ?location1 - location ?location2 - location)
        (expected_patient_interaction_time)
    )

    (:durative-action move
        :parameters (?r - terrestrial_robot ?from - location ?to - location)
        :duration (= ?duration (/ (distance ?from ?to) (speed ?r)))

        :condition (and
            (at start(robot_at ?r ?from))
            (over all (connected ?from ?to))
        )

        :effect (and
            (at start(not (robot_at ?r ?from)))
            (at end(robot_at ?r ?to))
        )
    )

    ;(:durative-action take_patient
    ;    :parameters (?ar - accompany_robot ?w - location ?p - patient)
    ;    :duration (= ?duration expected_patient_interaction_time)
    ;    :condition (and
    ;        (at start (robot_at ?ar ?w))
    ;        (at start (free_to_accompany ?ar))
    ;        (at start (patient_at ?p ?w))
    ;    )
    ;    :effect (and
    ;        (at end (not (patient_at ?p ?w)))
    ;        (at end (not (free_to_accompany ?ar)))
    ;        (at end (accompanying_pat ?p ?ar))
    ;    )
    ;)
;
    ;(:durative-action deliver_patient
    ;    :parameters (?ar - accompany_robot ?m - medical_unit ?p - patient)
    ;    :duration (= ?duration expected_patient_interaction_time)
    ;    :condition (and
    ;        (at start (robot_at ?ar ?m))
    ;        (at start (accompanying_pat ?p ?ar))
    ;    )
    ;    :effect (and
    ;        (at start (not (accompanying_pat ?p ?ar)))
    ;        (at start (free_to_accompany ?ar))
    ;        (at start (patient_at ?p ?m))
    ;    )
    ;)

)