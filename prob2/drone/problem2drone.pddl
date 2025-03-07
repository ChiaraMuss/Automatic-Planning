;; Define the healthcare logistics problem
(define (problem healthcare_problem)
  (:domain healthcare)  ;; The problem belongs to the "healthcare" domain

  ;; Define objects used in the problem
  (:objects
    central_warehouse - inventory  ;; The central warehouse where supplies are stored
    entrance location1 location2 - location  ;; Various locations in the environment
    scalpel tongue_depressor aspirin - content  ;; Different types of medical supplies
    box1 box2 box3 box4 box5 box6 box7 box8 box9 box10 - box  ;; Boxes containing medical supplies
    delivery_robot1 delivery_robot2 - delivery_robot  ;; Delivery robots used for transporting supplies
    accompany_robot1 - accompany_robot  ;; Accompanying robot for patient transport
    drone1 drone2 - drone  ;; Drones used for airborne transport
    carrier1 carrier2 carrier3 carrier4 - carrier  ;; Carriers used for transporting multiple boxes
    rocco ciro - patient  ;; Patients who need to be transported to medical units
    medical_unit1 medical_unit2 medical_unit3 - medical_unit  ;; Medical units receiving supplies and patients
  )

  ;; Define the initial state of the environment
  (:init
    ;; Initial positions of robots, drones, and patients
    (robot_at delivery_robot1 central_warehouse)  ;; Delivery robot 1 starts at the warehouse
    ;(robot_at delivery_robot2 central_warehouse)  ;; Delivery robot 2 is commented out
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

    ;; Initial inventory at medical units
    (= (med_unit_inventory_of medical_unit1 aspirin) 0)
    (= (med_unit_inventory_of medical_unit1 scalpel) 0)
    (= (med_unit_inventory_of medical_unit1 tongue_depressor) 0)

    (= (med_unit_inventory_of medical_unit2 aspirin) 0)
    (= (med_unit_inventory_of medical_unit2 scalpel) 0)
    (= (med_unit_inventory_of medical_unit2 tongue_depressor) 0)

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
    (connected central_warehouse location1)
    (connected location1 central_warehouse)

    (connected location1 location2)
    (connected location2 location1)

    (connected location1 medical_unit2)
    (connected medical_unit2 location1)

    (connected location2 medical_unit1)
    (connected medical_unit1 location2)
    
    (connected entrance location2)
    (connected location2 entrance)
    
    ;; Locations with drone ports
    (has_drone_port medical_unit1)
    (has_drone_port medical_unit2)
    (has_drone_port central_warehouse)
  )

  ;; Define the goal conditions
  (:goal
    (and
      ;; Required inventory levels at medical units
      (= 3 (med_unit_inventory_of medical_unit1 aspirin))  ;; Medical unit 1 needs 3 aspirin
      (= 2 (med_unit_inventory_of medical_unit1 scalpel))  ;; Medical unit 1 needs 2 scalpels
      (= 0 (med_unit_inventory_of medical_unit1 tongue_depressor))  ;; No tongue depressors needed
      
      (= 1 (med_unit_inventory_of medical_unit2 aspirin))  ;; Medical unit 2 needs 1 aspirin
      (= 0 (med_unit_inventory_of medical_unit2 scalpel))  ;; No scalpels needed
      (= 3 (med_unit_inventory_of medical_unit2 tongue_depressor))  ;; Medical unit 2 needs 3 tongue depressors

      ;; Patients must reach their designated medical units
      (patient_at rocco medical_unit1)  ;; Patient Rocco must arrive at medical unit 1
      (patient_at ciro medical_unit2)  ;; Patient Ciro must arrive at medical unit 2
    )
  )
)

