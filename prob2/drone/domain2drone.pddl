;; Define the healthcare logistics domain
(define (domain healthcare)
  (:requirements :strips :typing :fluents)  ;; Requires STRIPS, typing, and fluents for numeric operations

  ;; Define types used in the domain
  (:types
    medical_unit inventory - location  ;; Medical units and inventory locations as subtypes of location
    scissor band_aid syringes - content  ;; Different types of medical supplies
    terrestrial_robot arial_robot - robot  ;; Two categories of robots: terrestrial and aerial
    delivery_robot accompany_robot - terrestrial_robot  ;; Terrestrial robots: delivery and accompanying robots
    drone - arial_robot  ;; Aerial robot type: drone
    carrier - box  ;; A carrier is a type of box used for transporting goods
    patient  ;; Represents a patient who needs to be transported
  )

  ;; Define predicates for system states
  (:predicates
    (robot_at ?r - robot ?l - location)  ;; A robot is at a specific location
    (contains ?b - box ?c - content)  ;; A box contains a specific type of content
    (carrier_load ?c - carrier ?b - box)  ;; A carrier is loaded with a specific box
    (carrier_at ?c - carrier ?l - location)  ;; A carrier is at a specific location
    (robot_has_carrier ?r - robot ?c - carrier)  ;; A robot is assigned a carrier
    (connected ?l1 - location ?l2 - location)  ;; Two locations are connected
    (has_drone_port ?l - location)  ;; A location has a drone port
    (at_box ?b - box ?l - location)  ;; A box is at a specific location

    ;; Patient management predicates
    (free_to_accompany ?r - accompany_robot)  ;; An accompanying robot is free to transport a patient
    (accompanying_pat ?p - patient ?ar - accompany_robot)  ;; A patient is currently being accompanied by a robot
    (patient_at ?p - patient ?l - location)  ;; A patient is at a specific location
  )

  ;; Define functions for numeric tracking
  (:functions
    (carrier_capacity ?c - carrier)  ;; Maximum capacity of a carrier
    (carrier_used ?c - carrier)  ;; Current usage of a carrier
    (med_unit_inventory_of ?m - medical_unit ?c - content)  ;; Inventory of a medical unit for a given supply
    (warehouse_inventory_of ?i - location ?c - content)  ;; Inventory of a warehouse for a given supply
  )

  ;; ----------------------- TERRESTRIAL ROBOTS -----------------------

  ;; Action: Move a terrestrial robot between connected locations
  (:action move
    :parameters (?r - terrestrial_robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))  ;; Robot must be at the source location and locations must be connected
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))  ;; Update the robot's location
  )

  ;; ----------------------- DELIVERY ROBOT -----------------------

  ;; Action: Load a box into a delivery robot's carrier
  (:action load_to_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - inventory)
    :precondition (and 
      (robot_at ?r ?l)  ;; The robot must be at the inventory location
      (robot_has_carrier ?r ?c)  ;; The robot must have a carrier assigned
      (carrier_at ?c ?l)  ;; The carrier must be at the inventory location
      (at_box ?b ?l)  ;; The box must be at the inventory location
      (< (carrier_used ?c) (carrier_capacity ?c))  ;; The carrier must have available capacity
    )
    :effect (and 
      (not (at_box ?b ?l))  ;; The box is no longer at the inventory location
      (carrier_load ?c ?b)  ;; The box is now loaded into the carrier
      (increase (carrier_used ?c) 1)  ;; The carrier's used capacity increases
    )
  )

  ;; Action: Unload a box from a delivery robot's carrier
  (:action unload_from_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - location)
    :precondition (and 
      (robot_at ?r ?l)  ;; The robot must be at the location
      (robot_has_carrier ?r ?c)  ;; The robot must have a carrier assigned
      (carrier_load ?c ?b)  ;; The box must be loaded in the carrier
    )
    :effect (and 
      (at_box ?b ?l)  ;; The box is placed at the location
      (not (carrier_load ?c ?b))  ;; The box is no longer in the carrier
      (decrease (carrier_used ?c) 1)  ;; The carrier's used capacity decreases
    )
  )

  ;; Action: Deliver a box's content to a medical unit
  (:action unload_content
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content)
    :precondition (and 
      (robot_at ?r ?m)  ;; The robot must be at the medical unit
      (at_box ?b ?m)  ;; The box must be at the medical unit
      (contains ?b ?c)  ;; The box must contain the required content
    )
    :effect (and 
      (not (contains ?b ?c))  ;; The box no longer contains the content
      (increase (med_unit_inventory_of ?m ?c) 1)  ;; The medical unit's inventory of that content increases
    )
  )

  ;; ----------------------- DRONE CAPABILITIES -----------------------

  ;; Action: Load a box into a drone's carrier
  (:action load_to_drone_carrier
    :parameters (?d - drone ?b - box ?c - carrier ?l - location)
    :precondition (and 
      (robot_at ?d ?l) 
      (robot_has_carrier ?d ?c) 
      (carrier_at ?c ?l) 
      (at_box ?b ?l)
      (< (carrier_used ?c) (carrier_capacity ?c))
    )
    :effect (and 
      (not (at_box ?b ?l)) 
      (carrier_load ?c ?b) 
      (increase (carrier_used ?c) 1))
  )

  ;; Action: Move a drone between locations with drone ports
  (:action move_drone
    :parameters (?d - drone ?c - carrier ?from - location ?to - location)
    :precondition (and 
      (robot_at ?d ?from) 
      (carrier_at ?c ?from) 
      (robot_has_carrier ?d ?c) 
      (has_drone_port ?from) 
      (has_drone_port ?to))
    :effect (and 
      (not (carrier_at ?c ?from)) (carrier_at ?c ?to) 
      (not (robot_at ?d ?from)) (robot_at ?d ?to))
  )

  ;; Action: Unload a box from a drone's carrier
  (:action unload_from_drone_carrier
    :parameters (?d - drone ?c - carrier ?b - box ?l - location)
    :precondition (and 
      (robot_at ?d ?l) 
      (robot_has_carrier ?d ?c) 
      (carrier_load ?c ?b))
    :effect (and 
      (at_box ?b ?l) 
      (not (carrier_load ?c ?b)) 
      (decrease (carrier_used ?c) 1))
  )

  ;; ----------------------- PATIENT MANAGEMENT -----------------------

  ;; Action: Pick up a patient
  (:action take_patient
    :parameters (?ar - accompany_robot ?w - location ?p - patient)
    :precondition (and (robot_at ?ar ?w) (free_to_accompany ?ar) (patient_at ?p ?w))
    :effect (and (not (patient_at ?p ?w)) (not (free_to_accompany ?ar)) (accompanying_pat ?p ?ar))
  )

  ;; Action: Deliver a patient to a medical unit
  (:action deliver_patient
    :parameters (?ar - accompany_robot ?m - medical_unit ?p - patient)
    :precondition (and (robot_at ?ar ?m) (accompanying_pat ?p ?ar))
    :effect (and (not (accompanying_pat ?p ?ar)) (free_to_accompany ?ar) (patient_at ?p ?m))
  )
)

