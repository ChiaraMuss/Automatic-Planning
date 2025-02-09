(define (domain healthcare)
  (:requirements :strips :typing)

  (:types 
    location 
    medical_unit - location
    box 
    content
    delivery_robot accompany_robot - robot ;; Due tipi di robot
    patient
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (medical_supplied ?m - medical_unit ?c - content)  ;; Una medical unit ha ricevuto almeno un tipo di fornitura
    (needs_supply ?m - medical_unit ?c - content) ;; Indica che l'unità ha bisogno di una fornitura
    (contains ?b - box ?c - content)
    (robot_carries ?r - robot ?b - box)
    (at_box ?b - box ?l - location)
    (connected ?l1 - location ?l2 - location)
    (hands_free ?r - delivery_robot)
    ;; patient management
    (free_to_accompany ?r - accompany_robot)
    (accompanying_pat ?p - patient ?ar - accompany_robot)
    (patient_at ?p - patient ?l - location)
  )

  ;; Spostamento di un robot tra location collegate
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ;; Caricare una scatola sul robot
  (:action load_box
    :parameters (?r - delivery_robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (at_box ?b ?l) (hands_free ?r))
    :effect (and (not (at_box ?b ?l)) (not (hands_free ?r)) (robot_carries ?r ?b))
  )

  ;; Scaricare una scatola dal robot
  (:action unload_box
    :parameters (?r - delivery_robot ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (robot_carries ?r ?b) )
    :effect (and (not (robot_carries ?r ?b)) (hands_free ?r) (at_box ?b ?l))
  )

  ;; Consegnare contenuto a una medical unit **solo se questa ne ha bisogno**
  (:action unload_content
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content)
    :precondition (and 
      (robot_at ?r ?m) 
      (at_box ?b ?m) 
      (contains ?b ?c)
      (needs_supply ?m ?c) ;; Verifica che l'unità abbia bisogno di questa fornitura
    )
    :effect (and 
      (not (contains ?b ?c)) 
      (medical_supplied ?m ?c) ;; Conferma che l'unità ha ricevuto il tipo di contenuto
    )
  )

  ;; Spostare un paziente a un'unità medica
  ;;Patient position can be derived from the accompanying robot (makes more sense since is easier to track a robot than the patient in real life)
  (:action take_patient
    :parameters (?ar - accompany_robot ?w - location ?p - patient)
    :precondition (and (robot_at ?ar ?w) (free_to_accompany ?ar) (patient_at ?p ?w))
    :effect (and (not(patient_at ?p ?w)) (not(free_to_accompany ?ar)) (accompanying_pat ?p ?ar))
  )
  ;; Could be changed to move patient only between waiting rooms 
  (:action deliver_patient
    :parameters (?ar - accompany_robot ?m - medical_unit ?p - patient)
    :precondition (and (robot_at ?ar ?m) (accompanying_pat ?p ?ar))
    :effect (and (not (accompanying_pat ?p ?ar)) (free_to_accompany ?ar) (patient_at ?p ?m))
  )
)

