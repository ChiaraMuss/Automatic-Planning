(define (domain healthcare)
  (:requirements :strips :typing)

  (:types 
    location 
    medical_unit - location
    box 
    content 
    robot 
    patient)

  (:predicates
    (robot_at ?r - robot ?l - location)
    (patient_at ?p - patient ?l - location)
    (medical_supplied ?m - medical_unit)  ;; Un'unità medica ha ricevuto almeno una fornitura
    (contains ?b - box ?c - content)
    (robot_carries ?r - robot ?b - box)
    (at_box ?b - box ?l - location)
    (connected ?l1 - location ?l2 - location)
  )

  ;; Spostamento di un robot tra location collegate
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ;; Caricare una scatola sul robot
  (:action load_box
    :parameters (?r - robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (at_box ?b ?l))
    :effect (and (not (at_box ?b ?l)) (robot_carries ?r ?b))
  )

  ;; Scaricare una scatola dal robot
  (:action unload_box
    :parameters (?r - robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (robot_carries ?r ?b))
    :effect (and (not (robot_carries ?r ?b)) (at_box ?b ?l))
  )

  ;; Consegnare contenuto a una medical unit
  (:action unload_content
    :parameters (?r - robot ?b - box ?m - medical_unit ?c - content ?l - location)
    :precondition (and (robot_at ?r ?l) (at_box ?b ?l) (contains ?b ?c))
    :effect (and (not (contains ?b ?c)) (medical_supplied ?m)))
  
  ;; Spostare un paziente a un'unità medica
  (:action move_patient
    :parameters (?r - robot ?p - patient ?from - location ?to - medical_unit)
    :precondition (and (robot_at ?r ?from) (patient_at ?p ?from) (connected ?from ?to))
    :effect (and (not (patient_at ?p ?from)) (patient_at ?p ?to)))
)
