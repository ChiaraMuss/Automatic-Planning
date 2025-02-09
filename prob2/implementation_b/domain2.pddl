(define (domain healthcare)
  (:requirements :strips :typing :fluents)

  (:types
    medical_unit inventory - location
    scissor band_aid syringes - content
    delivery_robot accompany_robot - robot
    carrier - box
    patient
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    ;(has ?m - medical_unit ?c - content)
    ;(empty ?b - box)
    (contains ?b - box ?c - content)
    (carrier_load ?c - carrier ?b - box)
    (carrier_at ?c - carrier ?l - location)
    ;(robot_carries_carrier ?r - robot ?c - carrier)
    (connected ?l1 - location ?l2 - location)
    (at_box ?b - box ?l - location)

    ;; patient stuff
    (free_to_accompany ?r - accompany_robot)
    (accompanying_pat ?p - patient ?ar - accompany_robot)
    (patient_at ?p - patient ?l - location)

  )

  (:functions
    (carrier_capacity ?c - carrier) ; Capacità massima del carrier
    (carrier_used ?c - carrier) ; Numero di scatole attualmente nel carrier
    (med_unit_inventory_of ?m - medical_unit ?c - content)
    (warehouse_inventory_of ?i - location ?c - content) ;;TODO aggiungere riempi scatola

  )
  ;; Spostare un robot
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )

  ;; Caricare una scatola nel carrier
  (:action load_to_carrier
    :parameters (?r - delivery_robot ?c - carrier ?b - box ?l - inventory)
    :precondition (and (robot_at ?r ?l) (carrier_at ?c ?l) (at_box ?b ?l)
      (< (carrier_used ?c) (carrier_capacity ?c))
      )
    :effect (and (not (at_box ?b ?l)) (carrier_load ?c ?b)
      (increase (carrier_used ?c) 1))
  )

  ;; Scaricare una scatola dal carrier
  (:action unload_from_carrier ;;in caso di necessitá i robot si possono passare il carrier a metá strada
    :parameters (?r - delivery_robot ?c - carrier ?b - box ?l - location)
    :precondition (and (robot_at ?r ?l) (carrier_at ?c ?l) (carrier_load ?c ?b))
    :effect (and (at_box ?b ?l) (not (carrier_load ?c ?b))
      (decrease (carrier_used ?c) 1))
  )

  ;; Muovere un carrier con il robot
  ;(:action connect_to_carrier
  ;  :parameters (?r - robot ?c - carrier ?l - location)
  ;  :precondition (and (robot_at ?r ?l) (carrier_at ?c ?l))
  ;  :effect (and (not (carrier_at ?c ?l))
  ;    (robot_carries_carrier ?r ?c))
  ;)
;
  ;; Muovere un carrier con il robot
  ;(:action disconnect_from_carrier
  ;  :parameters (?r - robot ?c - carrier ?l - location)
  ;  :precondition (and (robot_at ?r ?l) (robot_carries_carrier ?r ?c))
  ;  :effect (and (not (robot_carries_carrier ?r ?c))
  ;  (carrier_at ?c ?l))
  ;)

  ;; Muovere un carrier con il robot
  (:action move_carrier
    :parameters (?r - delivery_robot ?c - carrier ?from - location ?to - location)
    :precondition (and (robot_at ?r ?from) (carrier_at ?c ?from) (connected ?from ?to))
    :effect (and (not (carrier_at ?c ?from)) (carrier_at ?c ?to)
      (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  ;; Scaricare il contenuto da una scatola in un'unità medica
  (:action unload_content
    :parameters (?r - delivery_robot ?b - box ?m - medical_unit ?c - content)
    :precondition (and (robot_at ?r ?m) (at_box ?b ?m) (contains ?b ?c))
    :effect (and (not (contains ?b ?c)) (increase (med_unit_inventory_of ?m ?c) 1)) ;(has ?m ?c)
  )

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