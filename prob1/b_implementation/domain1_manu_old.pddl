;Header and description
;Assumptions:
;- An object MUST be in a medical_unit or in a inventory location
;- There could be multiple waiting rooms (There could be used for example when moving a patient from a medical unit to another)
;- If NOT with accompanied from a robot a patient MUST be in a waiting_room
;- A standard delivery robot can deliver only one box at a time
;- There is no distinctions between paths that can take accompanying robot and paths that only delivery robots  can take (delivery tunnels to connect facilities)
;- A box even if technically could be taken from any location it should be delivered or taken only from / to a medical_unit or inventory location  (it's not a bug, it'a a feature)
(define (domain domain_name)

    ;remove requirements that are not needed
    (:requirements :strips :typing :conditional-effects :negative-preconditions)

    (:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
        robot box location boxable patient - objects
        delivery accompanying disable_transporter - robot
        scissor band_aid syringes - boxable
        inventory waiting_room medical_unit floor - location
    )

    ; un-comment following line if constants are needed
    ;(:constants )

    (:predicates ;todo: define predicates here
        (connected ?l1 - location ?l2 - location)
        (box_at ?b - box ?l - location)
        (has_box ?b - box ?dr - delivery)
        (box_empty ?b - box)
        (empty_handed ?dr - delivery)
        (free_to_accompany ?ar - accompanying)
        (accompanying_pat ?p - patient ?ar - accompanying)
        (patient_in ?p - patient ?w - waiting_room)
        (patient_at_med_unit ?p - patient ?m - medical_unit)
        (robot_at ?r - robot ?l - location)
        (boxed_in ?o - boxable ?b - box)
        (obj_at ?o - boxable ?l - location)
        (boxed_in_at_loc ?o - boxable ?l - location ?b - box)
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
        :precondition (and (robot_at ?ar ?w) (free_to_accompany ?ar) (patient_in ?p ?w))
        :effect (and (not(patient_in ?p ?w)) (not(free_to_accompany ?ar)) (accompanying_pat ?p ?ar))
    )
    ;; Could be changed to move patient only between waiting rooms 
    (:action deliver_patient
        :parameters (?ar - accompanying ?m - medical_unit ?p - patient)
        :precondition (and (robot_at ?ar ?m) (accompanying_pat ?p ?ar))
        :effect (and (not (accompanying_pat ?p ?ar)) (free_to_accompany ?ar) (patient_at_med_unit ?p ?m))
    )

    ;; distinction between take/deliver empty and non empty box to change the boxed_in_at_loc predicate

    ;;The fill_box should be only done at a med_unit or in an inventory location. Even if not forced by this function it should be implied by the delivery action and problem init
    (:action fill_box
        :parameters (?b - box ?o - boxable ?l - location ?d - delivery)
        :precondition (and (box_at ?b ?l) (obj_at ?o ?l) (robot_at ?d ?l) (empty_handed ?d) (box_empty ?b))
        :effect (and (not (obj_at ?o ?l)) (not(box_empty ?b)) (boxed_in ?o ?b))
    )

)