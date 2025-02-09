(define (problem healthcare-problem)
  (:domain healthcare)

  (:objects
    central_warehouse entrance location1 location2 location3 - location
    scalpel tongue_depressor aspirin - content
    box1 box2 box3 box4 box5 - box
    delivery_robot1 - delivery_robot
    accompany_robot1 - accompany_robot
    patient1 patient2 - patient
    medical_unit1 medical_unit2 medical_unit3 - medical_unit
  )

  (:init
    ;; Posizione iniziale dei robot
    (robot_at delivery_robot1 central_warehouse)
    (robot_at accompany_robot1 entrance)
    (free_to_accompany accompany_robot1)
    (hands_free delivery_robot1)

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

    ;; Definiamo i bisogni delle medical unit
    (needs_supply medical_unit1 scalpel) ;; Medical unit 1 ha bisogno di scalpel
    (needs_supply medical_unit1 aspirin) ;; Medical unit 1 ha bisogno di aspirin
    ;; medical_unit2 non ha bisogno di alcuna fornitura, quindi non è definito qui
    (needs_supply medical_unit3 aspirin) ;; Medical unit 3 ha bisogno di aspirin

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

    ;(connected entrance medical_unit1)
    ;(connected entrance medical_unit2)
    ;(connected entrance medical_unit3)
  )

  (:goal
    (and
      ;; Le medical unit devono ricevere solo la fornitura che necessitano
      (medical_supplied medical_unit1 scalpel)
      (medical_supplied medical_unit1 aspirin)
      (medical_supplied medical_unit3 aspirin)

      ;; I pazienti devono raggiungere le unità mediche
      (patient_at patient1 medical_unit1)
      (patient_at patient2 medical_unit2)
    )
  )
)

