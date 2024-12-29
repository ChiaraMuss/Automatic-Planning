(define (domain healthcare)
  (:requirements :strips :typing)

  (:types 
    location 
    medical_unit - location  ; medical_unit is a subtype of location
    box 
    content 
    robot accompany_robot - robot  ; Define a new type for the robot that can accompany patients
    carrier - box                  ; Carrier is a box type that holds boxes
    patient)

  (:predicates
    (robot_at ?r - robot ?l - location)       ; Robot is at a location
    (patient_at ?p - patient ?l - location)   ; Patient is at a location
    (has ?m - medical_unit ?c - content)      ; Medical unit has a specific content
    (empty ?b - box)                          ; Box is empty
    (contains ?b - box ?c - content)          ; Box contains content
    (robot_carries ?r - robot ?b - box)       ; Robot carries box
    (accompanying ?r - accompany_robot ?p - patient)    ; Robot accompanies patient
    (connected ?l1 - location ?l2 - location) ; Locations are connected
    (at_box ?b - box ?l - location)           ; Box is at a location
    (at_content ?c - content ?l - location)  ; Content is at a location
    (at_location ?m - medical_unit ?l - location)  ; Medical unit is at a location
    (on_carrier ?r - robot ?b - box)          ; Box is on the robot's carrier
    (carrier_capacity ?r - robot ?cap - number)   ; Robot's carrier capacity (max boxes)
    (carrier_full ?r - robot)                  ; Predicate to track if carrier is full
  )

  ;; Move robot between locations
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ;; Load a box onto the robot's carrier
  (:action load_box
    :parameters (?r - robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) 
                       (at_box ?b ?l) 
                       (empty ?b) 
                       (not (on_carrier ?r ?b)) 
                       (not (carrier_full ?r)))  ; Ensure the carrier is not full
    :effect (and (not (empty ?b)) (on_carrier ?r ?b) (robot_carries ?r ?b))
  )

  ;; Unload a box from the robot's carrier to a location
  (:action unload_box
    :parameters (?r - robot ?b - box ?l - location ?m - medical_unit ?c - content)
    :precondition (and (robot_at ?r ?l) (on_carrier ?r ?b) (at_location ?m ?l) (contains ?b ?c))
    :effect (and (not (on_carrier ?r ?b)) (at_box ?b ?l) (has ?m ?c))  ; Medical unit gets the content
  )

  ;; Fill box with content
  (:action fill_box
    :parameters (?r - robot ?b - box ?c - content ?l - location)
    :precondition (and (robot_at ?r ?l) (empty ?b) (at_content ?c ?l))  ; Ensure content is at the location
    :effect (and (not (empty ?b)) (contains ?b ?c))
  )

  ;; Empty box into a medical unit
  (:action empty_box
    :parameters (?r - robot ?b - box ?m - medical_unit ?l - location ?c - content)
    :precondition (and (robot_carries ?r ?b) (robot_at ?r ?l) (at_location ?m ?l) (contains ?b ?c))  ; Ensure content is in the box
    :effect (and (not (contains ?b ?c)) (has ?m ?c))  ; Medical unit now has the content from the box
  )

  ;; Pick up an empty box
  (:action pick_up_box
    :parameters (?r - robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (at_box ?b ?l) (empty ?b)) ; Box and robot must be at the same location
    :effect (robot_carries ?r ?b)
  )

  ;; Deliver the box to the medical unit
  (:action deliver_box
    :parameters (?r - robot ?b - box ?m - medical_unit ?l - location ?c - content)
    :precondition (and (robot_carries ?r ?b) (robot_at ?r ?l) (at_location ?m ?l) (contains ?b ?c))  ; Ensure content is in the box
    :effect (and (empty ?b) (has ?m ?c))  ; After delivery, medical unit has the content
  )

  ;; Accompany patient to the medical unit (allowing multiple location traversal)
  (:action accompany_patient
    :parameters (?r - accompany_robot ?p - patient ?from - location ?to - location ?m - medical_unit)
    :precondition (and (robot_at ?r ?from) (patient_at ?p ?from) (connected ?from ?to) (at_location ?m ?to))
    :effect (and (patient_at ?p ?to) (accompanying ?r ?p) (not (patient_at ?p ?from)))
  )
)

