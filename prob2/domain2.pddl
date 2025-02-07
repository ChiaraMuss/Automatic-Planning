(define (domain healthcare)
  (:requirements :strips :typing :fluents)

  (:types 
    location 
    medical_unit - location
    box 
    content 
    robot 
    accompany_robot - robot
    carrier - box
    patient)

  (:predicates
    (robot_at ?r - robot ?l - location)
    (patient_at ?p - patient ?l - location)
    (has ?m - medical_unit ?c - content)
    (empty ?b - box)
    (contains ?b - box ?c - content)
    (robot_carries ?r - robot ?b - box)
    (carrier_load ?c - carrier ?b - box)
    (carrier_at ?c - carrier ?l - location)
    (robot_carries_carrier ?r - robot ?c - carrier)
    (connected ?l1 - location ?l2 - location)
    (at_box ?b - box ?l - location)
  )

  (:functions
    (carrier_capacity ?c - carrier)  ; Capacità massima del carrier
    (carrier_used ?c - carrier)      ; Numero di scatole attualmente nel carrier
  )

  ;; Spostare un robot
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ;; Caricare una scatola nel carrier
  (:action load_to_carrier
    :parameters (?r - robot ?c - carrier ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (carrier_at ?c ?l) (at_box ?b ?l)
                       (< (carrier_used ?c) (carrier_capacity ?c)))
    :effect (and (not (at_box ?b ?l)) (carrier_load ?c ?b)
                 (increase (carrier_used ?c) 1))
  )

  ;; Scaricare una scatola dal carrier
  (:action unload_from_carrier
    :parameters (?r - robot ?c - carrier ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (carrier_at ?c ?l) (carrier_load ?c ?b))
    :effect (and (at_box ?b ?l) (not (carrier_load ?c ?b))
                 (decrease (carrier_used ?c) 1))
  )

  ;; Muovere un carrier con il robot
  (:action move_carrier
    :parameters (?r - robot ?c - carrier ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (carrier_at ?c ?from) (connected ?from ?to))
    :effect (and (not (carrier_at ?c ?from)) (carrier_at ?c ?to)
                 (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ;; Scaricare il contenuto da una scatola in un'unità medica
  (:action unload_content
    :parameters (?r - robot ?b - box ?m - medical_unit ?c - content ?l - location)
    :precondition (and (robot_at ?r ?l) (at_box ?b ?l) (contains ?b ?c))
    :effect (and (not (contains ?b ?c)) (has ?m ?c))
  )
)


