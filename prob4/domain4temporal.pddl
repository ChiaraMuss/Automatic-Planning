;Assumptions:
; - distances in decimeters and durations in seconds

(define (domain temporal_healthcare)

  ;remove requirements that are not needed
  (:requirements :strips :typing :durative-actions :numeric-fluents)

  (:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle

    medical_unit inventory generic_location - location
    content terrestrial_robot arial_robot - robot
    delivery_robot accompany_robot - terrestrial_robot
    drone - arial_robot
    carrier
    box
    patient
  )

  ; un-comment following line if constants are needed
  ;(:constants )

  (:predicates ;todo: define predicates here
    (robot_at ?r - robot ?l - location)
    (contains ?b - box ?c - content)
    (carrier_load ?c - carrier ?b - box)
    (robot_has_carrier ?r - robot ?c - carrier) ;; The non delivery robot must be distinguished by the problem
    (connected ?l1 - location ?l2 - location)
    (has_drone_port ?l - location)
    (at_box ?b - box ?l - location)

    ;; patient management
    (free_to_accompany ?r - accompany_robot)
    (accompanying_pat ?p - patient ?ar - accompany_robot)
    (patient_at ?p - patient ?l - location)
  )

  (:functions ;todo: define numeric functions here
    (carrier_capacity ?c - carrier)
    (carrier_used ?c - carrier)
    (med_unit_inventory_of ?m - medical_unit ?c - content)
    (warehouse_inventory_of ?i - location ?c - content)
    (speed ?r - robot)
    (distance ?location1 - location ?location2 - location)
    (arial_distance ?location1 - location ?location2 - location)
    (expected_patient_interaction_time)
    (loading_time)
    (unloading_time)

  )

  ;define actions here

  ;;----------------------- TERRESTRIAL ROBOTS
  ;; Moving a robot
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
  (:durative-action load_to_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - inventory)
    :duration (= ?duration loading_time)
    :condition (and
      (at start (and
          (robot_has_carrier ?r ?c)
          (< (carrier_used ?c) (carrier_capacity ?c))
        )
      )
      (over all
        (and
          (robot_at ?r ?l)
          (at_box ?b ?l)
        )
      )
    )
    :effect (and
      (at end (and
          (not (at_box ?b ?l))
          (carrier_load ?c ?b)
          (increase (carrier_used ?c) 1))

      )
    )
  )

  (:durative-action unload_from_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - location)
      :duration (= ?duration 1)
      :condition (and 
          (at start (and
          (carrier_load ?c ?b)
          (robot_has_carrier ?r ?c)
        ))
          (over all (and
          (robot_at ?r ?l) (robot_has_carrier ?r ?c)
        ))

      )
      :effect (and 
          (at end (and
          (at_box ?b ?l)
          (not (carrier_load ?c ?b))
          (decrease (carrier_used ?c) 1)
        ))
      )
  )
  

  ;;---------------------------------------------- PATIENTS MANAGEMENT
  ;; Patient transfer 
  (:durative-action take_patient
    :parameters (?ar - accompany_robot ?w - location ?p - patient)
    :duration (= ?duration expected_patient_interaction_time)
    :condition (and
      (over all (robot_at ?ar ?w)) ; to force the robot to stay put during the action
      (at start (free_to_accompany ?ar))
      (over all (patient_at ?p ?w)) ; to force the robot to stay put during the action
    )
    :effect (and
      (at start (not (patient_at ?p ?w)))
      (at end (not (free_to_accompany ?ar)))
      (at end (accompanying_pat ?p ?ar))
    )
  )
  (:durative-action deliver_patient
    :parameters (?ar - accompany_robot ?m - medical_unit ?p - patient)
    :duration (= ?duration expected_patient_interaction_time)
    :condition (and
      (over all (robot_at ?ar ?m))
      (at start (accompanying_pat ?p ?ar))
    )
    :effect (and
      (at end (not (accompanying_pat ?p ?ar)))
      (at end (free_to_accompany ?ar))
      (at end (patient_at ?p ?m))
    )
  )
)