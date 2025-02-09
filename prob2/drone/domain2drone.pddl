(define (domain healthcare)
  (:requirements :strips :typing :fluents)

  (:types
    medical_unit inventory - location
    scissor band_aid syringes - content
    delivery_robot accompany_robot drone - robot
    carrier - box
    patient
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (contains ?b - box ?c - content)
    (carrier_load ?c - carrier ?b - box)
    (carrier_at ?c - carrier ?l - location)
    (robot_has_carrier ?r - delivery_robot ?c - carrier)
    (drone_carrying ?d - drone ?b - box)
    (connected ?l1 - location ?l2 - location)
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

  ;; Moving a robot
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

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

  ;; Move robot along with its carrier
  (:action move_with_carrier
    :parameters (?r - delivery_robot ?c - carrier ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (carrier_at ?c ?from) (robot_has_carrier ?r ?c) (connected ?from ?to))
    :effect (and (not (carrier_at ?c ?from)) (carrier_at ?c ?to)
      (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ;; Load a box into a drone
  (:action load_to_drone
    :parameters (?d - drone ?b - box ?l - location)
    :precondition (and (robot_at ?d ?l) (at_box ?b ?l))
    :effect (and (not (at_box ?b ?l)) (drone_carrying ?d ?b)))

  ;; Move a drone
  (:action move_drone
    :parameters (?d - drone ?from - location ?to - location)
    :precondition (and (robot_at ?d ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?d ?from)) (robot_at ?d ?to)))

  ;; Unload a box from a drone
  (:action unload_from_drone
    :parameters (?d - drone ?b - box ?l - location)
    :precondition (and (robot_at ?d ?l) (drone_carrying ?d ?b))
    :effect (and (at_box ?b ?l) (not (drone_carrying ?d ?b))))

  ;; Unload content at a medical unit
  (:action unload_content
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content)
    :precondition (and (robot_at ?r ?m) (at_box ?b ?m) (contains ?b ?c))
    :effect (and (not (contains ?b ?c)) (increase (med_unit_inventory_of ?m ?c) 1))
  )

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
