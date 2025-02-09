(define (problem healthcare-problem)
  (:domain healthcare)

  (:objects
    central_warehouse entrance location1 location2 location3 - location
    scalpel tongue_depressor aspirin - content
    box1 box2 box3 box4 box5 - box
    delivery_robot - robot
    accompany_robot - robot
    patient1 patient2 - patient
    medical_unit1 medical_unit2 medical_unit3 - medical_unit
  )

  (:init
    ;; Posizione iniziale dei robot
    (robot_at delivery_robot central_warehouse)
    (robot_at accompany_robot entrance)

    ;; Posizione iniziale dei pazienti
    (patient_at patient1 entrance)
    (patient_at patient2 entrance)

    ;; Posizione iniziale delle scatole
    (at_box box1 central_warehouse)
    (at_box box2 central_warehouse)
    (at_box box3 central_warehouse)
    (at_box box4 central_warehouse)
    (at_box box5 central_warehouse)

    ;; Contenuti delle scatole
    (contains box1 scalpel)
    (contains box2 tongue_depressor)
    (contains box3 aspirin)
    (contains box4 scalpel)
    (contains box5 aspirin)

    ;; Connessioni bidirezionali tra le location
    (connected central_warehouse location1)
    (connected location1 central_warehouse)

    (connected location1 location2)
    (connected location2 location1)

    (connected location2 entrance)
    (connected entrance location2)

    (connected entrance location1)
    (connected location1 entrance)

    (connected location1 medical_unit1)
    (connected medical_unit1 location1)

    (connected location2 medical_unit2)
    (connected medical_unit2 location2)

    (connected location3 medical_unit3)
    (connected medical_unit3 location3)

    (connected location2 location3)
    (connected location3 location2)

    (connected entrance medical_unit1)
    (connected entrance medical_unit2)
    (connected entrance medical_unit3)
  )

  (:goal
    (and
      (medical_supplied medical_unit1)
      (medical_supplied medical_unit2)
      (medical_supplied medical_unit3)

      (patient_at patient1 medical_unit1)
      (patient_at patient2 medical_unit2)
    )
  )
)
