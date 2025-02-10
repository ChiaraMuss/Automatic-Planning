(define (problem healthcare_problem)
  (:domain healthcare)

  (:objects
    central_warehouse heliport_alpha - inventory
    entrance sector_a sector_b sector_c - location
    scalpel tongue_depressor aspirin - content
    box1 box2 box3 box4 box5 box6 box7 box8 box9 box10 - box
    delivery_robot1 delivery_robot2 - delivery_robot
    accompany_robot1 - accompany_robot
    drone1 drone2 - drone
    carrier1 carrier2 carrier3 carrier4 - carrier
    rocco ciro - patient
    cardiology dentistry radiology neuro_surgery day_hospital - medical_unit
  )

  (:init
    ; Initial positions of robots, drones, and patients
    (robot_at delivery_robot1 central_warehouse)
    (robot_at delivery_robot2 central_warehouse)
    (robot_at accompany_robot1 entrance)
    (robot_at drone1 central_warehouse)
    (robot_at drone2 central_warehouse)
    (patient_at rocco entrance)
    (patient_at ciro entrance)
    
    ;; Accompanying robot status
    (free_to_accompany accompany_robot1)
    
    ; Initial positions of boxes
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

    ; Initial carrier positions and assignments
    (carrier_at carrier1 central_warehouse)
    (carrier_at carrier2 central_warehouse)
    (carrier_at carrier3 central_warehouse)
    (carrier_at carrier4 central_warehouse)
    (robot_has_carrier delivery_robot1 carrier1)
    (robot_has_carrier delivery_robot2 carrier2)
    (robot_has_carrier drone1 carrier3)

    ; Carrier capacities
    (= (carrier_capacity carrier1) 3)
    (= (carrier_capacity carrier2) 2)
    (= (carrier_capacity carrier3) 2)
    (= (carrier_capacity carrier4) 5)

    (= (carrier_used carrier1) 0)
    (= (carrier_used carrier2) 0)
    (= (carrier_used carrier3) 0)
    (= (carrier_used carrier4) 0)

    ;; Inventory
    (= (med_unit_inventory_of day_hospital aspirin) 0 )
    (= (med_unit_inventory_of day_hospital scalpel) 0)
    (= (med_unit_inventory_of day_hospital tongue_depressor) 0)

    (= (med_unit_inventory_of neuro_surgery aspirin) 0)
    (= (med_unit_inventory_of neuro_surgery scalpel) 0)
    (= (med_unit_inventory_of neuro_surgery tongue_depressor) 0)

    (= (med_unit_inventory_of radiology aspirin) 0)
    (= (med_unit_inventory_of radiology scalpel) 0)
    (= (med_unit_inventory_of radiology tongue_depressor) 0)

    (= (med_unit_inventory_of cardiology aspirin) 0)
    (= (med_unit_inventory_of cardiology scalpel) 0)
    (= (med_unit_inventory_of cardiology tongue_depressor) 0)

    (= (med_unit_inventory_of dentistry aspirin) 0)
    (= (med_unit_inventory_of dentistry scalpel) 0)
    (= (med_unit_inventory_of dentistry tongue_depressor) 0)

    ; Box contents
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
    
    ; Terrestrial connections
    (connected central_warehouse sector_a)
    (connected sector_a central_warehouse)

    (connected sector_a sector_b)
    (connected sector_b sector_a)

    (connected sector_b sector_c)
    (connected sector_c sector_b)
    (connected day_hospital sector_c)
    (connected sector_c day_hospital)

    (connected entrance sector_c)
    (connected sector_c entrance)
    (connected sector_c heliport_alpha)
    (connected heliport_alpha sector_c)

    (connected neuro_surgery sector_b)
    (connected sector_b neuro_surgery)
    (connected heliport_alpha neuro_surgery)
    (connected neuro_surgery heliport_alpha)

    
    (connected radiology sector_b)
    (connected sector_b radiology)
    (connected radiology heliport_alpha)
    (connected heliport_alpha radiology)

    (connected cardiology sector_a)
    (connected sector_a cardiology)

    (connected dentistry sector_a)
    (connected sector_a dentistry)

    ; Drone port available
    (has_drone_port heliport_alpha)
    (has_drone_port central_warehouse)

  )

  (:goal
    (and
      (= 3 (med_unit_inventory_of day_hospital aspirin))
      ;(= 0 (med_unit_inventory_of day_hospital scalpel))
      ;(= 0 (med_unit_inventory_of day_hospital tongue_depressor))
;
      ;(= 0 (med_unit_inventory_of radiology aspirin))
      ;(= 0 (med_unit_inventory_of radiology scalpel))
      ;(= 0 (med_unit_inventory_of radiology tongue_depressor))
;
      ;(= 0 (med_unit_inventory_of cardiology aspirin))
      ;(= 0 (med_unit_inventory_of cardiology scalpel))
      (= 1 (med_unit_inventory_of cardiology tongue_depressor))
;
      ;(= 0 (med_unit_inventory_of neuro_surgery aspirin))
      (= 2 (med_unit_inventory_of neuro_surgery scalpel))
      ;(= 0 (med_unit_inventory_of neuro_surgery tongue_depressor))

      (= 2 (med_unit_inventory_of dentistry aspirin))
      ;(= 0 (med_unit_inventory_of dentistry scalpel))
      (= 2 (med_unit_inventory_of dentistry tongue_depressor))

      (patient_at rocco cardiology)
      (patient_at ciro day_hospital)
    )
  )
)

