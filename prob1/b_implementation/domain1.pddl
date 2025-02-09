;Header and description
;Assumptions:
;- An object MUST be in a medical_unit or in a inventory location
;- There could be multiple waiting rooms (There could be used for example when moving a patient from a medical unit to another)
;- If NOT with accompanied from a robot a patient MUST be in a waiting_room
;- A standard delivery robot can deliver only one box at a time
;- There is no distinctions between paths that can take accompanying robot and paths that only delivery robots  can take (delivery tunnels to connect facilities)
;- A box even if technically could be taken from any location it should be delivered or taken only from / to a medical_unit or inventory location  (it's not a bug, it'a a feature)
;- For how the problem is defined, it implies that medical_units are not locations "Each medical unit is at a specific location."
;- All patients are equal, no disabled patient that needs specialized care from a robot or human intervention
(define (domain healthcare_vm)

    ;remove requirements that are not needed
    (:requirements :strips :typing :conditional-effects)

    (:types
        ;if in need we could expand the patient definition to include disabled patients that has to be accompanied by specialized robots
        robot box location content patient medical_unit - objects
        delivery_robot accompanying - robot
        scissor band_aid syringes - content 
        inventory waiting_room  floor - location
    )

    (:predicates ;todo: define predicates here
        (connected ?l1 - location ?l2 - location)
        (at_box ?b - box ?l - location)
        (robot_carries ?r - delivery_robot ?b - box) ; Robot carries box
        (empty_box ?b - box)
        (empty_handed ?dr - delivery_robot)
        (free_to_accompany ?ar - accompanying)
        (accompanying_pat ?p - patient ?ar - accompanying)
        (patient_at ?p - patient ?w - location)
        (robot_at ?r - robot ?l - location)
        (boxed_in ?o - content ?b - box)
        (obj_at ?o - content ?l - location)
        (med_unit_has ?m - medical_unit ?c - content)
        (med_unit_at ?m - medical_unit ?l - location)
        (box_at_med_unit ?b - box ?m - medical_unit)

        ;        (boxed_in_at_loc ?o - boxable ?l - location ?b - box)
    )

    ;; Move robot between locations
    (:action move
        :parameters (?r - robot ?from - location ?to - location)
        :precondition (and (robot_at ?r ?from) (connected ?from ?to))
        :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
    )
    ;;Patient position can be derived from the accompanying robot (makes more sense since is easier to track a robot than the patient in real life)
    (:action take_patient
        :parameters (?ar - accompanying ?w - waiting_room ?p - patient)
        :precondition (and (robot_at ?ar ?w) (free_to_accompany ?ar) (patient_at ?p ?w))
        :effect (and (not(patient_at ?p ?w)) (not(free_to_accompany ?ar)) (accompanying_pat ?p ?ar))
    )
    ;; Could be changed to move patient only between waiting rooms 
    (:action deliver_patient
        :parameters (?ar - accompanying ?l - location ?m - medical_unit ?p - patient)
        :precondition (and (robot_at ?ar ?m) (med_unit_at ?m ?l) (accompanying_pat ?p ?ar))
        :effect (and (not (accompanying_pat ?p ?ar)) (free_to_accompany ?ar) (patient_at ?p ?m))
    )


    ;;The fill_box should be only done at a med_unit or in an inventory location. Even if not forced by this function it should be implied by the delivery action and problem init
    ;fill a box with a content, if the box is empty and the content to add in the box, the box and the robotic agent are at the same location;
    (:action fill_box
        :parameters (?b - box ?o - content ?l - location ?d - delivery_robot)
        :precondition (and (at_box ?b ?l) (obj_at ?o ?l) (robot_at ?d ?l) (empty_handed ?d) (empty_box ?b))
        :effect (and (not (obj_at ?o ?l) ) (not(empty_box ?b)) (boxed_in ?b ?o))
    )

    ;; Empty box into a medical unit
    ;empty a box by leaving the content to the current location and given medical unit, causing the medical unit to then have the content;
    (:action empty_box
        :parameters (?r - delivery_robot ?b - box ?l - location ?m - medical_unit ?c - content)
        :precondition (and (robot_carries ?r ?b) (robot_at ?r ?l) (med_unit_at ?m ?l) (boxed_in ?b ?c)) ; Ensure content is in the box
        :effect (and (not (boxed_in ?b ?c)) (med_unit_has ?m ?c) (obj_at ?c ?l) (empty_box ?b)) ; Medical unit now has the content from the box
    )

    ;; Pick up an empty box
    (:action pick_up_box
        :parameters (?r - delivery_robot ?b - box ?l - location)
        :precondition (and (robot_at ?r ?l) (empty_handed ?r) (at_box ?b ?l) (empty_box ?b)) ; Box and robot must be at the same location
        :effect (and (not (empty_handed ?r))(robot_carries ?r ?b))
    )

    ;; Deliver the box to the medical unit
    (:action deliver_box
        :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?l - location )
        :precondition (and (robot_carries ?r ?b) (robot_at ?r ?l) (med_unit_at ?m ?l))
        :effect (and (not(robot_carries ?r ?b)) (empty_handed ?r) (box_at_med_unit ?b ?m) ) ; After delivery, medical unit has the box
    )

)