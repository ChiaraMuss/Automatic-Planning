(define (problem healthcare_problem)
  (:domain healthcare)

  (:objects
    central_warehouse - inventory
    entrance location1 location2 - location
    scalpel tongue_depressor aspirin - content
    box1 box2 box3 box4 box5 box6 box7 box8 box9 box10 - box
    delivery_robot1 - delivery_robot
    accompany_robot1 - accompany_robot
    carrier1 carrier2 carrier3 - carrier
    rocco ciro - patient
    medical_unit1 medical_unit2 medical_unit3 - medical_unit
  )

  (:init
    ; Posizioni iniziali dei robot e dei pazienti
    (robot_at delivery_robot1 central_warehouse)
    ;(robot_at robot2 central_warehouse)
    ;(robot_at robot3 central_warehouse)
    (robot_at accompany_robot1 entrance)
    (patient_at rocco entrance)
    (patient_at ciro entrance)

    ;;Accompanying robot status
    (free_to_accompany accompany_robot1)
    ; Posizione iniziale delle scatole
    (at_box box1 central_warehouse)
    (at_box box2 central_warehouse)
    (at_box box3 central_warehouse)
    (at_box box4 central_warehouse)
    (at_box box5 central_warehouse)
    (at_box box6 central_warehouse)
    (at_box box7 central_warehouse)
    (at_box box8 central_warehouse)
    (at_box box9 central_warehouse)
    (at_box box10 central_warehouse)

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

    ;;Inventario
    (= (med_unit_inventory_of medical_unit1 aspirin) 0 )
    (= (med_unit_inventory_of medical_unit1 scalpel) 0)
    (= (med_unit_inventory_of medical_unit1 tongue_depressor) 0)

    (= (med_unit_inventory_of medical_unit2 aspirin) 0)
    (= (med_unit_inventory_of medical_unit2 scalpel) 0)
    (= (med_unit_inventory_of medical_unit2 tongue_depressor) 0)

    ; Contenuti iniziali nelle scatole
    (contains box1 aspirin)
    (contains box2 aspirin)
    (contains box3 aspirin)
    (contains box4 aspirin)
    (contains box5 aspirin)
    (contains box6 scalpel)
    (contains box7 scalpel)
    (contains box8 tongue_depressor)
    (contains box9 tongue_depressor)
    (contains box10 tongue_depressor)
    ; Connessioni tra le location
    (connected central_warehouse location1)
    (connected location1 central_warehouse )
    (connected location1 location2)
    (connected location2 location1)

    (connected location1 medical_unit2)
    (connected medical_unit2 location1)

    (connected location2 medical_unit1)
    (connected medical_unit1 location2)
    (connected entrance location2 )

    (connected location2 entrance)
  )

  (:goal
    (and
      (= 1 (med_unit_inventory_of medical_unit1 aspirin))
      (= 2 (med_unit_inventory_of medical_unit2 tongue_depressor))

      (patient_at rocco medical_unit1)

    )
  )
)
