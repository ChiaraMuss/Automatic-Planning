;All modifications made between the prob 4 files and these ones is marked with ##

;Assumptions:

(define (domain temporal_healthcare_simplified)

  (:requirements :typing :durative-actions)

  (:types
    ;## Removed all the type hierarchy
    medical_unit inventory location delivery_robot accompany_robot drone content carrier box patient capacity-number
  )

  (:predicates ;## Thanks to the deletion of the type hierarchy we had to add all of the permutations of robot and location that we need to maintain compatibility with prob4 problems
    (delivery_robot_at_location ?r - delivery_robot ?l - location)
    (delivery_robot_at_medical_unit ?r - delivery_robot ?l - medical_unit)
    (delivery_robot_at_inventory ?r - delivery_robot ?l - inventory)

    (accompany_robot_at_location ?r - accompany_robot ?l - location)
    (accompany_robot_at_medical_unit ?r - accompany_robot ?l - medical_unit)

    (drone_at ?r - drone ?l - inventory) ;we suppose that heliports are also inventory locations

    (contains ?b - box ?c - content)
    (carrier_load ?c - carrier ?b - box)

    (delivery_robot_has_carrier ?r - delivery_robot ?c - carrier)
    (drone_has_carrier ?r - drone ?c - carrier)

    (connection_between_locations ?l1 - location ?l2 - location) ;no connection between med_unit and heliports
    (connection_location_med_unit ?l1 - location ?l2 - medical_unit)
    (connection_location_inventory ?l1 - location ?l2 - inventory)

    (has_drone_port ?l - inventory)
    (at_box_inventory ?b - box ?l - inventory);## Distinction between types of box locations
    (at_box_medical_unit ?b - box ?l - medical_unit)

    ;; patient management
    (free_to_accompany ?r - accompany_robot)
    (accompanying_pat ?p - patient ?ar - accompany_robot)
    (patient_at_location ?p - patient ?l - location)
    (patient_at_med_unit ?p - patient ?l - medical_unit) ;## adding distinction in location for the patient

    (carrier_capacity ?c - carrier ?cap - capacity-number) ;## defining the capacities as predicates to avoid using fluents
    (capacity-predecessor ?s1 - capacity-number ?s2 - capacity-number)
    (med_unit_inventory_of ?m - medical_unit ?c - content ?n - capacity-number)

  )

  ;; Removed all fluents

  ;;----------------------- TERRESTRIAL ROBOTS
  ;; Moving a robot

  ;;;;;---------------------------------Delivery Robot
  (:durative-action move_between_location_delivery_robot
    :parameters (?r - delivery_robot ?from - location ?to - location)
    :duration (= ?duration 40)
    :condition (and
      (at start (delivery_robot_at_location ?r ?from))
      (at start (connection_between_locations ?from ?to))
    )
    :effect (and
      (at start (not (delivery_robot_at_location ?r ?from)))
      (at end (delivery_robot_at_location ?r ?to))
    )
  )
  ;## adding distinction between movement of different terrestrial robots in and out of specific types of locations

  (:durative-action move_in_inventory_delivery_robot
    :parameters (?r - delivery_robot ?from - location ?to - inventory)
    :duration (= ?duration 40)
    :condition (and
      (at start (delivery_robot_at_location ?r ?from))
      (at start (connection_location_inventory ?from ?to))
    )
    :effect (and
      (at start (not (delivery_robot_at_location ?r ?from)))
      (at end (delivery_robot_at_inventory ?r ?to))
    )
  )

  (:durative-action move_out_inventory_delivery_robot
    :parameters (?r - delivery_robot ?from - inventory ?to - location)
    :duration (= ?duration 40)
    :condition (and
      (at start (delivery_robot_at_inventory ?r ?from))
      (at start (connection_location_inventory ?to ?from))
    )
    :effect (and
      (at start (not (delivery_robot_at_inventory ?r ?from)))
      (at end (delivery_robot_at_location ?r ?to))
    )
  )

  (:durative-action move_in_med_unit_delivery_robot
    :parameters (?r - delivery_robot ?from - location ?to - medical_unit)
    :duration (= ?duration 40)
    :condition (and
      (at start (delivery_robot_at_location ?r ?from))
      (at start (connection_location_med_unit ?from ?to))
    )
    :effect (and
      (at start (not (delivery_robot_at_location ?r ?from)))
      (at end (delivery_robot_at_medical_unit ?r ?to))
    )
  )

  (:durative-action move_out_med_unit_delivery_robot
    :parameters (?r - delivery_robot ?from - medical_unit ?to - location)
    :duration (= ?duration 40)
    :condition (and
      (at start (delivery_robot_at_medical_unit ?r ?from))
      (at start (connection_location_med_unit ?to ?from))
    )
    :effect (and
      (at start (not (delivery_robot_at_medical_unit ?r ?from)))
      (at end (delivery_robot_at_location ?r ?to))
    )
  )

  ;;;;;--------------------------------- Accompanying Robot
  ;## adding distinction between movement of different accompanying robot in and out of specific types of locations.
  ;## movement in and out of inventory location are excluded for accompanying robot since they must not transport boxes

  (:durative-action move_between_location_accompany_robot
    :parameters (?r - accompany_robot ?from - location ?to - location)
    :duration (= ?duration 40)
    :condition (and
      (at start (accompany_robot_at_location ?r ?from))
      (at start (connection_between_locations ?from ?to))
    )
    :effect (and
      (at start (not (accompany_robot_at_location ?r ?from)))
      (at end (accompany_robot_at_location ?r ?to))
    )
  )

  (:durative-action move_in_med_unit_accompany_robot
    :parameters (?r - accompany_robot ?from - location ?to - medical_unit)
    :duration (= ?duration 40)
    :condition (and
      (at start (accompany_robot_at_location ?r ?from))
      (at start (connection_location_med_unit ?from ?to))
    )
    :effect (and
      (at start (not (accompany_robot_at_location ?r ?from)))
      (at end (accompany_robot_at_medical_unit ?r ?to))
    )
  )

  (:durative-action move_out_med_unit_accompany_robot
    :parameters (?r - accompany_robot ?from - medical_unit ?to - location)
    :duration (= ?duration 40)
    :condition (and
      (at start (accompany_robot_at_medical_unit ?r ?from))
      (at start (connection_location_med_unit ?to ?from))
    )
    :effect (and
      (at start (not (accompany_robot_at_medical_unit ?r ?from)))
      (at end (accompany_robot_at_location ?r ?to))
    )
  )

  ;;;------------ CARRIER Management
  ;## Modifying the action to use the capacity predicates
  (:durative-action load_to_robot_carrier_inventory
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - inventory ?n1 ?n2 - capacity-number)
    :duration (= ?duration 3)
    :condition (and
      (at start (delivery_robot_has_carrier ?r ?c))
      (at start (capacity-predecessor ?n1 ?n2))
      (at start (carrier_capacity ?c ?n2))
      (over all (delivery_robot_at_inventory ?r ?l))
      (over all (at_box_inventory ?b ?l))
    )
    :effect (and
      (at start (not (carrier_capacity ?c ?n2)))
      (at end (carrier_capacity ?c ?n1))

      (at end (carrier_load ?c ?b))
      (at end (not (at_box_inventory ?b ?l)))
    )
  )

  ;## Two different unloads, one for the inventory location and one for the medical unit location

  (:durative-action unload_from_robot_carrier_inventory
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - inventory ?n1 ?n2 - capacity-number)
    :duration (= ?duration 1)
    :condition (and
      (at start (carrier_load ?c ?b))
      (at start (delivery_robot_has_carrier ?r ?c))
      (at start (capacity-predecessor ?n1 ?n2))
      (at start (carrier_capacity ?c ?n1))
      (over all (delivery_robot_at_inventory ?r ?l))

    )
    :effect (and
      (at start (not (carrier_load ?c ?b)))
      (at start (not (carrier_capacity ?c ?n1)))
      (at start (carrier_capacity ?c ?n2))

      (at end (at_box_inventory ?b ?l))
    )
  )

  (:durative-action unload_from_robot_carrier_medical_unit
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - medical_unit ?n1 ?n2 - capacity-number)
    :duration (= ?duration 1)
    :condition (and
      (at start (carrier_load ?c ?b))
      (at start (delivery_robot_has_carrier ?r ?c))
      (at start (capacity-predecessor ?n1 ?n2))
      (at start (carrier_capacity ?c ?n1))
      (over all (delivery_robot_at_medical_unit ?r ?l))
    )

    :effect (and
      (at start (not (carrier_load ?c ?b)))
      (at end (not (carrier_capacity ?c ?n1)))
      (at end (carrier_capacity ?c ?n2))

      (at end (at_box_medical_unit ?b ?l))
    )
  )

  (:durative-action unload_content ;## changed only the use of the capacity predicates
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content ?n1 ?n2 - capacity-number)
    :duration (= ?duration 20)
    :condition (and
      (at start (contains ?b ?c)) ; Ensure the box contains the specific content
      (at start (capacity-predecessor ?n1 ?n2))
      (at start (med_unit_inventory_of ?m ?c ?n1))

      (over all (delivery_robot_at_medical_unit ?r ?m))
      (over all (at_box_medical_unit ?b ?m))
    )
    :effect (and
      (at start (not (contains ?b ?c))) ; Remove content from the box
      (at end (not(med_unit_inventory_of ?m ?c ?n1)))
      (at end (med_unit_inventory_of ?m ?c ?n2)) ; Add content to the medical unit's inventory
    )
  )

  ;;------------------------------------------------ DRONE CAPABILITIES

  (:durative-action load_to_drone_carrier ;## changed only the use of the capacity predicates
    :parameters (?d - drone ?b - box ?c - carrier ?l - inventory ?n1 ?n2 - capacity-number)
    :duration (= ?duration 2)
    :condition (and
      (at start (drone_has_carrier ?d ?c))
      (at start (capacity-predecessor ?n1 ?n2))
      (at start (carrier_capacity ?c ?n2))
      (over all (drone_at ?d ?l))
      (over all (at_box_inventory ?b ?l))
    )
    :effect (and
      (at start (not (carrier_capacity ?c ?n2)))
      (at start (carrier_capacity ?c ?n1))

      (at end (carrier_load ?c ?b))
      (at end (not (at_box_inventory ?b ?l)))
    )
  )

  (:durative-action unload_from_drone_carrier ;## changed only the use of the capacity predicates
    :parameters (?d - drone ?c - carrier ?b - box ?l - inventory)
    :duration (= ?duration 4)
    :condition (and
      (over all
        (drone_at ?d ?l)
      )
      (at start (drone_has_carrier ?d ?c))
      (over all (carrier_load ?c ?b))
    )
    :effect (and
      (at end (not (carrier_load ?c ?b)))
      (at end (at_box_inventory ?b ?l))
    )
  )

  (:durative-action move_drone
    :parameters (?d - drone ?c - carrier ?from - inventory ?to - inventory)
    :duration (= ?duration 10)
    :condition (and
      (at start
        (drone_at ?d ?from)
      )
      (over all (drone_has_carrier ?d ?c))
      (over all (has_drone_port ?from))
      (over all (has_drone_port ?to))
    )
    :effect (and
      (at start (not (drone_at ?d ?from)))
      (at end (drone_at ?d ?to))
    )
  )

  ;;---------------------------------------------- PATIENTS MANAGEMENT
  ;; Patient transfer 
  (:durative-action take_patient ;##changed the predicates because of the new types 
    :parameters (?ar - accompany_robot ?w - location ?p - patient)
    :duration (= ?duration 20)
    :condition (and
      (at start (free_to_accompany ?ar))
      (at start (patient_at_location ?p ?w))
      (over all (accompany_robot_at_location ?ar ?w)) ; to force the robot to stay put during the action

    )
    :effect (and
      (at start (not (patient_at_location ?p ?w)))
      (at start (not (free_to_accompany ?ar)))
      (at end (accompanying_pat ?p ?ar))
    )
  )

  (:durative-action deliver_patient ;##changed the predicates because of the new types 
    :parameters (?ar - accompany_robot ?m - medical_unit ?p - patient)
    :duration (= ?duration 20)
    :condition (and
      (at start (accompanying_pat ?p ?ar))
      (over all (accompany_robot_at_medical_unit ?ar ?m))

    )
    :effect (and
      (at end (not (accompanying_pat ?p ?ar)))
      (at end (free_to_accompany ?ar))
      (at end (patient_at_med_unit ?p ?m))
    )
  )
)