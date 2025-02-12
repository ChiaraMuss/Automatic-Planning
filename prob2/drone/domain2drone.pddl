(define (domain healthcare)
  (:requirements :strips :typing :fluents)

  (:types
    medical_unit inventory - location
    scissor band_aid syringes - content
    terrestrial_robot arial_robot - robot
    delivery_robot accompany_robot - terrestrial_robot
    drone - arial_robot
    carrier - box
    patient
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (contains ?b - box ?c - content)
    (carrier_load ?c - carrier ?b - box)
    (carrier_at ?c - carrier ?l - location)
    (robot_has_carrier ?r - robot ?c - carrier) ;; The non delivery robot must be distinguished by the problem
    (connected ?l1 - location ?l2 - location)
    (has_drone_port ?l - location)
    (at_box ?b - box ?l - location)

    ;; patient management
    (free_to_accompany ?r - accompany_robot)
    (accompanying_pat ?p - patient ?ar - accompany_robot)
    (patient_at ?p - patient ?l - location)
  )

  (:functions
    (carrier_capacity ?c - carrier)
    (carrier_used ?c - carrier)
    (med_unit_inventory_of ?m - medical_unit ?c - content)
    (warehouse_inventory_of ?i - location ?c - content)
  )
;;----------------------- TERRESTRIAL ROBOTS
  ;; Moving a robot
  (:action move
    :parameters (?r - terrestrial_robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  ;;-------------------------------------------DELIVERY ROBOT
  ;; Load a box into the carrier of the robot
  (:action load_to_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - inventory)
    :precondition (and (robot_at ?r ?l) (robot_has_carrier ?r ?c) (carrier_at ?c ?l) (at_box ?b ?l)
      (< (carrier_used ?c) (carrier_capacity ?c)))
    :effect (and (not (at_box ?b ?l)) (carrier_load ?c ?b) (increase (carrier_used ?c) 1))
  )

  ;; Unload a box from the carrier
  (:action unload_from_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - location)
    :precondition (and (robot_at ?r ?l) (robot_has_carrier ?r ?c) (carrier_load ?c ?b))
    :effect (and (at_box ?b ?l) (not (carrier_load ?c ?b)) (decrease (carrier_used ?c) 1))
  )


  ;; Unload content at a medical unit
  (:action unload_content
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content)
    :precondition (and (robot_at ?r ?m) (at_box ?b ?m) (contains ?b ?c))
    :effect (and (not (contains ?b ?c)) (increase (med_unit_inventory_of ?m ?c) 1))
  )

  ;;------------------------------------------------ DRONE CAPABILITIES
  ;; Load a box into a drone
  (:action load_to_drone_carrier
    :parameters (?d - drone ?b - box ?c - carrier ?l - location)
    :precondition (and (robot_at ?d ?l) (robot_has_carrier ?d ?c) (carrier_at ?c ?l) (at_box ?b ?l)
      (< (carrier_used ?c) (carrier_capacity ?c)))
  
    :effect (and (not (at_box ?b ?l)) (carrier_load ?c ?b) (increase (carrier_used ?c) 1))
  )

  ;; Move a drone
  (:action move_drone
    :parameters (?d - drone ?c - carrier ?from - location ?to - location)
    :precondition (and (robot_at ?d ?from) (carrier_at ?c ?from) (robot_has_carrier ?d ?c) (has_drone_port ?from) (has_drone_port ?to))
    :effect (and (not (carrier_at ?c ?from)) (carrier_at ?c ?to)
      (not (robot_at ?d ?from)) (robot_at ?d ?to))
  )

  (:action unload_from_drone_carrier
    :parameters (?d - drone ?c - carrier ?b - box ?l - location)
    :precondition (and (robot_at ?d ?l) (robot_has_carrier ?d ?c) (carrier_load ?c ?b))
    :effect (and (at_box ?b ?l) (not (carrier_load ?c ?b)) (decrease (carrier_used ?c) 1))
  )

  ;; TODO ha senso che un drone svuoti un pacco?
  ;;---------------------------------------------- PATIENTS MANAGEMENT
  ;; Patient transfer
  (:action take_patient
    :parameters (?ar - accompany_robot ?w - location ?p - patient)
    :precondition (and (robot_at ?ar ?w) (free_to_accompany ?ar) (patient_at ?p ?w))
    :effect (and (not (patient_at ?p ?w)) (not (free_to_accompany ?ar)) (accompanying_pat ?p ?ar))
  )

  (:action deliver_patient
    :parameters (?ar - accompany_robot ?m - medical_unit ?p - patient)
    :precondition (and (robot_at ?ar ?m) (accompanying_pat ?p ?ar))
    :effect (and (not (accompanying_pat ?p ?ar)) (free_to_accompany ?ar) (patient_at ?p ?m))
  )
)
