;Assumptions:
; - distances in decimeters and durations in seconds

;; Define the domain for a healthcare logistics system using temporal planning
(define (domain temporal_healthcare)

  ;; Declare the required features for this domain
  (:requirements 
    :strips              ;; Basic STRIPS-like planning (preconditions & effects)
    :typing              ;; Use of typed objects (e.g., robot, location)
    :durative-actions    ;; Supports actions that take time to complete
    :numeric-fluents     ;; Allows numeric variables (e.g., for distances, capacities)
  )

  ;; Declare object types and their hierarchical relationships
  (:types
    medical_unit inventory generic_location - location  ;; Locations where robots and supplies can be
    content terrestrial_robot aerial_robot - robot      ;; Robots are either ground-based or aerial
    delivery_robot accompany_robot - terrestrial_robot  ;; Ground robots: delivery & patient transport
    drone - aerial_robot                                ;; Drones are aerial robots
    carrier box patient                                 ;; Carriers transport multiple boxes
  )

    (:predicates 
    ;; Robot movement and connectivity
    (robot_at ?r - robot ?l - location)          ;; The robot ?r is at location ?l
    (connected ?l1 - location ?l2 - location)    ;; Two locations ?l1 and ?l2 are connected
    (has_drone_port ?l - location)               ;; Location ?l has a drone landing/takeoff port

    ;; Supply management
    (contains ?b - box ?c - content)             ;; Box ?b contains medical supply ?c
    (carrier_load ?c - carrier ?b - box)         ;; Carrier ?c is loaded with box ?b
    (robot_has_carrier ?r - robot ?c - carrier)  ;; Robot ?r is assigned carrier ?c
    (at_box ?b - box ?l - location)              ;; Box ?b is located at ?l

    ;; Patient management
    (free_to_accompany ?r - accompany_robot)     ;; Robot ?r is available to transport a patient
    (accompanying_pat ?p - patient ?ar - accompany_robot) ;; Patient ?p is being transported by ?ar
    (patient_at ?p - patient ?l - location)      ;; Patient ?p is currently at location ?l
  )

  (:functions 
    (carrier_capacity ?c - carrier)               ;; Maximum capacity of carrier ?c
    (carrier_used ?c - carrier)                   ;; How much of the carrier’s capacity is used
    (med_unit_inventory_of ?m - medical_unit ?c - content)  ;; Inventory at medical units
    (warehouse_inventory_of ?i - location ?c - content)  ;; Inventory at warehouse
    (speed ?r - robot)                            ;; Speed of the robot
    (distance ?location1 - location ?location2 - location)  ;; Distance between two locations
    (arial_distance ?location1 - location ?location2 - location)  ;; Aerial distance for drones
    (expected_patient_interaction_time)           ;; Estimated patient interaction time
    (loading_time)                                ;; Time required to load a box
    (unloading_time)                              ;; Time required to unload a box
    (content_unload_time)                         ;; Time required to unload content at a medical unit
  )


  ;  ;;----------------------- TERRESTRIAL ROBOTS
  ;; Moving a robot
  (:durative-action move
    :parameters (?r - terrestrial_robot ?from - location ?to - location)
    :duration (= ?duration (/ (distance ?from ?to) (speed ?r))) ;; Speed-based duration
    :condition (and
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )

  (  ;; Loading a box into a carrier
  (:durative-action load_to_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - inventory)
    :duration (= ?duration loading_time)
    :condition (and
      (at start (robot_has_carrier ?r ?c))
      (at start (< (carrier_used ?c) (carrier_capacity ?c)))
      (over all (robot_at ?r ?l))
      (over all (at_box ?b ?l))
    )
    :effect (and
      (at start (increase (carrier_used ?c) 1))
      (at end (carrier_load ?c ?b))
      (at end (not (at_box ?b ?l)))
    )
  )

  ;; Unloading a box from a carrier
  (:durative-action unload_from_robot_carrier
    :parameters (?r - delivery_robot ?b - box ?c - carrier ?l - location)
    :duration (= ?duration unloading_time)
    :condition (and
      (at start (carrier_load ?c ?b))
      (over all (robot_at ?r ?l))
      (over all (robot_has_carrier ?r ?c))
    )
    :effect (and
      (at start (not (carrier_load ?c ?b)))
      (at end (decrease (carrier_used ?c) 1))
      (at end (at_box ?b ?l))
    )
  )

   ;; Unloading medical content from a box into a medical unit's inventory
  (:durative-action unload_content
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content)
    :duration (= ?duration content_unload_time)
    :condition (and
      (at start (contains ?b ?c)) ; Ensure the box contains the specific content

      (over all (robot_at ?r ?m))
      (over all (at_box ?b ?m))
      (over all (>= (med_unit_inventory_of ?m ?c) 0))
    )
    :effect (and
      (at start (not (contains ?b ?c))) ; Remove content from the box
      (at end (increase (med_unit_inventory_of ?m ?c) 1)) ; Add content to the medical unit's inventory
    )
  )

  ;;------------------------------------------------ DRONE CAPABILITIES
  ;; Loading a box into a drone's carrier
  (:durative-action load_to_drone_carrier
    :parameters (?d - drone ?b - box ?c - carrier ?l - location)
    :duration (= ?duration loading_time)
    :condition (and
      (over all (robot_at ?d ?l))
      (over all (robot_has_carrier ?d ?c))
      (over all (at_box ?b ?l))
      (over all (< (carrier_used ?c) (carrier_capacity ?c)))

    )
    :effect (and
      (at end (not (at_box ?b ?l)))
      (at end (carrier_load ?c ?b))
      (at end (increase (carrier_used ?c) 1))

    )
  )
  ;; Unloading a box from a drone's carrier to a location
  (:durative-action unload_from_drone_carrier
    :parameters (?d - drone ?c - carrier ?b - box ?l - location)
    :duration (= ?duration unloading_time)
    :condition (and
      (over all
        (robot_at ?d ?l)
      )
      (over all (robot_has_carrier ?d ?c))
      (over all (carrier_load ?c ?b))
    )
    :effect (and
      (at end (not (carrier_load ?c ?b)))
      (at end (at_box ?b ?l))
    )
  )

 ;; Moving a drone between two locations using aerial transport
  (:durative-action move_drone
    :parameters (?d - drone ?c - carrier ?from - location ?to - location)
    :duration (= ?duration (/ (arial_distance ?from ?to) (speed ?d)))
    :condition (and
      (at start
        (robot_at ?d ?from)
      )
      (over all (robot_has_carrier ?d ?c))
      (over all (has_drone_port ?from))
      (over all (has_drone_port ?to))
    )
    :effect (and
      (at start
        (not (robot_at ?d ?from))
      )
      (at end
        (robot_at ?d ?to)
      )
    )
  )

  ;;---------------------------------------------- PATIENTS MANAGEMENT
  ;;  Accompanying robot picks up a patient for transport
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
  ;; Delivering a patient to a medical unit
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
