;; Define the healthcare logistics problem
(define (problem healthcare_problem)
  (:domain healthcare)  ;; The problem belongs to the "healthcare" domain

  ;; Define objects used in the problem
  (:objects
    central_warehouse heliport_alpha - inventory  ;; Inventory locations for supplies and drones
    entrance sector_a sector_b sector_c - location  ;; Various locations in the facility
    scalpel tongue_depressor aspirin - content  ;; Different types of medical supplies
    box1 box2 box3 box4 box5 box6 box7 box8 box9 box10 - box  ;; Boxes containing medical supplies
    delivery_robot1 delivery_robot2 - delivery_robot  ;; Delivery robots used for transporting supplies
    accompany_robot1 - accompany_robot  ;; Accompanying robot for patient transport
    drone1 drone2 - drone  ;; Drones used for airborne transport
    carrier1 carrier2 carrier3 carrier4 - carrier  ;; Carriers used for transporting multiple boxes
    rocco ciro - patient  ;; Patients who need to be transported to medical units
    cardiology dentistry radiology neuro_surgery day_hospital - medical_unit  ;; Medical units receiving supplies and patients
  )

  ;; Define the initial state of the environment
  (:init
    ;; Initial positions of robots, drones, and patients
    (robot_at delivery_robot1 central_warehouse)  ;; Delivery robot 1 starts at the warehouse
    (robot_at delivery_robot2 central_warehouse)  ;; Delivery robot 2 starts at the warehouse
    (robot_at accompany_robot1 entrance)  ;; The accompanying robot starts at the entrance
    (robot_at drone1 central_warehouse)  ;; Drone 1 starts at the warehouse
    (robot_at drone2 central_warehouse)  ;; Drone 2 starts at the warehouse
    (patient_at rocco entrance)  ;; Patient Rocco is at the entrance
    (patient_at ciro entrance)  ;; Patient Ciro is at the entrance
    
    ;; Accompanying robot status
    (free_to_accompany accompany_robot1)  ;; The accompanying robot is initially available
    
    ;; Initial positions of boxes
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

    ;; Initial positions of carriers and their assignments
    (carrier_at carrier1 central_warehouse)
    (carrier_at carrier2 central_warehouse)
    (carrier_at carrier3 central_warehouse)
    (carrier_at carrier4 central_warehouse)

    (robot_has_carrier delivery_robot1 carrier1)  ;; Delivery robot 1 has carrier 1
    (robot_has_carrier delivery_robot2 carrier2)  ;; Delivery robot 2 has carrier 2
    (robot_has_carrier drone1 carrier3)  ;; Drone 1 has carrier 3

    ;; Carrier capacities
    (= (carrier_capacity carrier1) 3)  ;; Carrier 1 can hold 3 boxes
    (= (carrier_capacity carrier2) 2)  ;; Carrier 2 can hold 2 boxes
    (= (carrier_capacity carrier3) 2)  ;; Carrier 3 can hold 2 boxes
    (= (carrier_capacity carrier4) 5)  ;; Carrier 4 can hold 5 boxes

    ;; Initial carrier usage (all are empty at the start)
    (= (carrier_used carrier1) 0)
    (= (carrier_used carrier2) 0)
    (= (carrier_used carrier3) 0)
    (= (carrier_used carrier4) 0)

    ;; Initial inventory at medical units (all empty)
    (= (med_unit_inventory_of day_hospital aspirin) 0)
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

    ;; Define the contents of each box
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
    
    ;; Define bidirectional connections between locations
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

    ;; Locations with drone ports
    (has_drone_port heliport_alpha)
    (has_drone_port central_warehouse)
  )

  ;; Define the goal conditions
  (:goal
    (and
      ;; Required inventory levels at medical units
      (= 3 (med_unit_inventory_of day_hospital aspirin))  ;; Day hospital needs 3 aspirin
      ;(= 0 (med_unit_inventory_of day_hospital scalpel))  
      ;(= 0 (med_unit_inventory_of day_hospital tongue_depressor))

      ;(= 0 (med_unit_inventory_of radiology aspirin))
      ;(= 0 (med_unit_inventory_of radiology scalpel))
      ;(= 0 (med_unit_inventory_of radiology tongue_depressor))

      ;(= 0 (med_unit_inventory_of cardiology aspirin))
      ;(= 0 (med_unit_inventory_of cardiology scalpel))
      (= 1 (med_unit_inventory_of cardiology tongue_depressor))  ;; Cardiology needs 1 tongue depressor

      ;(= 0 (med_unit_inventory_of neuro_surgery aspirin))
      (= 2 (med_unit_inventory_of neuro_surgery scalpel))  ;; Neurosurgery needs 2 scalpels
      ;(= 0 (med_unit_inventory_of neuro_surgery tongue_depressor))

      (= 2 (med_unit_inventory_of dentistry aspirin))  ;; Dentistry needs 2 aspirin
      ;(= 0 (med_unit_inventory_of dentistry scalpel))
      (= 2 (med_unit_inventory_of dentistry tongue_depressor))  ;; Dentistry needs 2 tongue depressors

      ;; Patients must reach their designated medical units
      (patient_at rocco cardiology)  ;; Patient Rocco must arrive at cardiology
      (patient_at ciro day_hospital)  ;; Patient Ciro must arrive at day hospital
    )
  )
)

