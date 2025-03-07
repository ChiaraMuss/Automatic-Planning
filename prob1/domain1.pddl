;; Define the domain for a healthcare logistics problem
(define (domain healthcare)
  (:requirements :strips :typing)

  ;; Define types used in the domain
  (:types 
    location  ;; General type for locations
    medical_unit - location  ;; A medical unit is a specific type of location
    box  ;; Represents a container for supplies
    content  ;; Represents the type of medical supply
    delivery_robot accompany_robot - robot  ;; Two types of robots: delivery and accompanying
    patient  ;; Represents a patient who may need to be transported
  )

  ;; Define predicates to describe the state of the system
  (:predicates
    (robot_at ?r - robot ?l - location)  ;; A robot is at a specific location
    (medical_supplied ?m - medical_unit ?c - content)  ;; A medical unit has received a supply
    (needs_supply ?m - medical_unit ?c - content)  ;; A medical unit requires a supply
    (contains ?b - box ?c - content)  ;; A box contains a specific type of content
    (robot_carries ?r - robot ?b - box)  ;; A robot is carrying a box
    (at_box ?b - box ?l - location)  ;; A box is at a specific location
    (connected ?l1 - location ?l2 - location)  ;; Two locations are connected
    (hands_free ?r - delivery_robot)  ;; A delivery robot is not carrying a box
    ;; Patient management predicates
    (free_to_accompany ?r - accompany_robot)  ;; An accompanying robot is available to assist a patient
    (accompanying_pat ?p - patient ?ar - accompany_robot)  ;; A patient is being accompanied by a robot
    (patient_at ?p - patient ?l - location)  ;; A patient is at a specific location
  )

  ;; Action: Move a robot from one location to another
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))  ;; The robot must be at the source location and the locations must be connected
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))  ;; The robot's location is updated
  )

  ;; Action: Load a box onto a delivery robot
  (:action load_box
    :parameters (?r - delivery_robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (at_box ?b ?l) (hands_free ?r))  ;; The robot must be at the location, the box must be there, and the robot must have free hands
    :effect (and (not (at_box ?b ?l)) (not (hands_free ?r)) (robot_carries ?r ?b))  ;; The robot now carries the box, and its hands are no longer free
  )

  ;; Action: Unload a box from a delivery robot
  (:action unload_box
    :parameters (?r - delivery_robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (robot_carries ?r ?b))  ;; The robot must be at the location and carrying the box
    :effect (and (not (robot_carries ?r ?b)) (hands_free ?r) (at_box ?b ?l))  ;; The box is placed at the location, and the robot's hands are free
  )

  ;; Action: Deliver the contents of a box to a medical unit if the unit needs it
  (:action unload_content
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content)
    :precondition (and 
      (robot_at ?r ?m)  ;; The robot must be at the medical unit
      (at_box ?b ?m)  ;; The box must be at the medical unit
      (contains ?b ?c)  ;; The box must contain the required content
      (needs_supply ?m ?c)  ;; The medical unit must need the supply
    )
    :effect (and 
      (not (contains ?b ?c))  ;; The box no longer contains the content
      (medical_supplied ?m ?c)  ;; The medical unit has received the supply
    )
  )

  ;; Action: Pick up a patient for transport to a medical unit
  ;; The patient's position is inferred from the accompanying robot
  (:action take_patient
    :parameters (?ar - accompany_robot ?w - location ?p - patient)
    :precondition (and (robot_at ?ar ?w) (free_to_accompany ?ar) (patient_at ?p ?w))  ;; The robot must be at the same location as the patient and be available
    :effect (and (not (patient_at ?p ?w)) (not (free_to_accompany ?ar)) (accompanying_pat ?p ?ar))  ;; The patient is now being accompanied by the robot
  )

  ;; Action: Deliver a patient to a medical unit
  (:action deliver_patient
    :parameters (?ar - accompany_robot ?m - medical_unit ?p - patient)
    :precondition (and (robot_at ?ar ?m) (accompanying_pat ?p ?ar))  ;; The robot must be at the medical unit and currently accompanying the patient
    :effect (and (not (accompanying_pat ?p ?ar)) (free_to_accompany ?ar) (patient_at ?p ?m))  ;; The patient is now at the medical unit, and the robot is free
  )
)
