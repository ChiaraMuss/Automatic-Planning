(define (problem healthcare-problem)
  (:domain healthcare)

  (:objects
    central_warehouse entrance location1 location2 - location
    scalpel tongue_depressor aspirin - content
    box1 box2 box3 box4 box5 - box
    robot1 robot2 robot3 - robot
    accompany_robot1 - accompany_robot
    carrier1 carrier2 carrier3 - carrier
    patient1 patient2 - patient
    medical_unit1 medical_unit2 medical_unit3 - medical_unit
  )

  (:init
    ; Posizioni iniziali dei robot e dei pazienti
    (robot_at robot1 central_warehouse)
    (robot_at robot2 central_warehouse)
    (robot_at robot3 central_warehouse)
    (robot_at accompany_robot1 entrance)
    (patient_at patient1 entrance)
    (patient_at patient2 entrance)

    ; Posizione iniziale delle scatole
    (at_box box1 central_warehouse)
    (at_box box2 central_warehouse)
    (at_box box3 central_warehouse)
    (at_box box4 central_warehouse)
    (at_box box5 central_warehouse)

    ; Posizione iniziale dei carrier
    (carrier_at carrier1 central_warehouse)
    (carrier_at carrier2 central_warehouse)
    (carrier_at carrier3 central_warehouse)

    ; Definizione della capacità dei carrier
    (= (carrier_capacity carrier1) 3)
    (= (carrier_capacity carrier2) 2)
    (= (carrier_capacity carrier3) 4)

    (= (carrier_used carrier1) 0)
    (= (carrier_used carrier2) 0)
    (= (carrier_used carrier3) 0)

    ; Contenuti iniziali nelle scatole
    (contains box1 scalpel)
    (contains box2 tongue_depressor)
    (contains box3 aspirin)
    (contains box4 scalpel)
    (contains box5 aspirin)

    ; Connessioni tra le location
    (connected central_warehouse location1)
    (connected location1 location2)
    (connected location2 entrance)
  )

  (:goal
    (and
      (has medical_unit1 scalpel)
      (has medical_unit1 tongue_depressor)
      (has medical_unit2 aspirin)
      (has medical_unit3 scalpel)
      (has medical_unit3 aspirin)
    )
  )
)
