;; Define the healthcare logistics problem instance using temporal planning
(define (problem healthcare_problem)
    (:domain temporal_healthcare)  ;; This problem is part of the "temporal_healthcare" domain

    ;; Define all objects used in the problem
    (:objects
        ;; Inventory storage locations where medical supplies are initially stored
        central_warehouse heliport_alpha - inventory  

        ;; General hospital locations (corridors, sectors)
        entrance sector_a sector_b sector_c - generic_location

        ;; Medical supplies (content to be delivered)
        scalpel tongue_depressor aspirin - content  

        ;; Boxes containing medical supplies
        box1 box2 box3 box4 box5 box6 box7 box8 box9 box10 - box  

        ;; Delivery robots for transporting medical supplies
        delivery_robot1 delivery_robot2 - delivery_robot  

        ;; Accompanying robot for transporting patients
        accompany_robot1 - accompany_robot  

        ;; Drones for airborne transport of supplies
        drone1 drone2 - drone  

        ;; Carriers that can hold multiple boxes for transport
        carrier1 carrier2 carrier3 carrier4 - carrier  

        ;; Patients who need transportation
        rocco ciro - patient  

        ;; Medical units where supplies and patients must be delivered
        cardiology dentistry radiology neuro_surgery day_hospital - medical_unit  
    )

    (:init
        ; - distances in meters and durations in seconds

        (=(speed delivery_robot1) 2.0)
        (=(speed delivery_robot2) 2.0)
        (=(speed accompany_robot1) 1.1); average walking speed is 1,3 m/s since we are in a hospital with older people we made it slower 
        (=(speed drone1) 14); around 50km/h
        (=(speed drone2) 14); around 50km/h

        (=(expected_patient_interaction_time) 30)
        (=(loading_time) 10)
        (=(unloading_time) 5)
        (=(content_unload_time) 5)
        ;; Initial positions of robots, drones, and patients
        (robot_at delivery_robot1 central_warehouse)
        (robot_at delivery_robot2 central_warehouse)
        (robot_at accompany_robot1 entrance)
        (robot_at drone1 central_warehouse)
        (robot_at drone2 central_warehouse)
        (patient_at rocco entrance)
        (patient_at ciro entrance)
        ;
        ;;; Accompanying robot status
        (free_to_accompany accompany_robot1)
        ;
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
        ;
        ;; Initial carrier positions and assignments
        (robot_has_carrier delivery_robot1 carrier1)
        (robot_has_carrier delivery_robot2 carrier2)
        (robot_has_carrier drone1 carrier3)
        ;
        ;; Carrier capacities
        (= (carrier_capacity carrier1) 1)
        (= (carrier_capacity carrier2) 1)
        (= (carrier_capacity carrier3) 2)
        (= (carrier_capacity carrier4) 5)

        (= (carrier_used carrier1) 0)
        (= (carrier_used carrier2) 0)
        (= (carrier_used carrier3) 0)
        (= (carrier_used carrier4) 0)
        ;
        ;;; Inventory
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

        ;; Box contents
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
        ;
        ;;; Terrestrial connections
        (connected central_warehouse sector_a) ;41m
        (=(distance central_warehouse sector_a) 41)
        (connected sector_a central_warehouse)
        (=(distance sector_a central_warehouse) 41)

        (connected sector_a sector_b)
        (=(distance sector_a sector_b) 68)
        (connected sector_b sector_a);68m
        (=(distance sector_b sector_a) 68)

        (connected sector_b sector_c)
        (=(distance sector_b sector_c) 79)
        (connected sector_c sector_b);79m
        (=(distance sector_c sector_b) 79)

        (connected day_hospital sector_c)
        (=(distance day_hospital sector_c) 10)
        (connected sector_c day_hospital)
        (=(distance sector_c day_hospital) 10)

        (connected entrance sector_c)
        (=(distance entrance sector_c) 27)
        (connected sector_c entrance);27m
        (=(distance sector_c entrance) 27)

        (connected sector_c heliport_alpha)
        (=(distance sector_c heliport_alpha) 25)
        (connected heliport_alpha sector_c);25m
        (=(distance heliport_alpha sector_c) 25)

        (connected neuro_surgery sector_b)
        (=(distance neuro_surgery sector_b) 10)
        (connected sector_b neuro_surgery)
        (=(distance sector_b neuro_surgery) 10)

        (connected heliport_alpha neuro_surgery)
        (=(distance heliport_alpha neuro_surgery) 10)
        (connected neuro_surgery heliport_alpha)
        (=(distance neuro_surgery heliport_alpha) 10)

        (connected radiology sector_b)
        (=(distance radiology sector_b) 10)
        (connected sector_b radiology)
        (=(distance sector_b radiology) 10)
        (connected radiology heliport_alpha)
        (=(distance radiology heliport_alpha) 10)
        (connected heliport_alpha radiology)
        (=(distance heliport_alpha radiology) 10)

        (connected cardiology sector_a)
        (=(distance cardiology sector_a) 10)
        (connected sector_a cardiology)
        (=(distance sector_a cardiology) 10)
        (connected dentistry sector_a)
        (=(distance dentistry sector_a) 10)
        (connected sector_a dentistry)
        (=(distance sector_a dentistry) 10)

        ; Drone port available
        (has_drone_port heliport_alpha);129m
        (has_drone_port central_warehouse)
        (=(arial_distance heliport_alpha central_warehouse) 129)
        (=(arial_distance central_warehouse heliport_alpha) 129)

    )

    ;; Define the goal state for successful logistics execution

    ;; Ensure specific medical supplies are delivered to each medical unit
    ;; - Day Hospital should have 3 aspirin, but no scalpels or tongue depressors
    ;; - Cardiology requires 1 tongue depressor, but no aspirin or scalpels
    ;; - Radiology does not require any supplies
    ;; - Dentistry needs 2 aspirin and 2 tongue depressors, but no scalpels
    ;; - Neuro Surgery requires 2 scalpels, but no aspirin or tongue depressors

    ;; Ensure patients reach their designated medical units
    ;; - Patient "Rocco" should be at the Cardiology unit
    ;; - Patient "Ciro" should be at the Day Hospital

    (:goal
        (and
            ;(robot_at accompany_robot1 sector_a)
            (= 3 (med_unit_inventory_of day_hospital aspirin))
            (= (med_unit_inventory_of day_hospital scalpel) 0)
            (= (med_unit_inventory_of day_hospital tongue_depressor) 0)

            (= 1 (med_unit_inventory_of cardiology tongue_depressor))
            (= (med_unit_inventory_of cardiology aspirin) 0)
            (= (med_unit_inventory_of cardiology scalpel) 0)

            (= (med_unit_inventory_of radiology aspirin) 0)
            (= (med_unit_inventory_of radiology scalpel) 0)
            (= (med_unit_inventory_of radiology tongue_depressor) 0)

            (= 2 (med_unit_inventory_of dentistry aspirin))
            (= 2 (med_unit_inventory_of dentistry tongue_depressor))
            (= (med_unit_inventory_of dentistry scalpel) 0)
            
            (= 2 (med_unit_inventory_of neuro_surgery scalpel))
            (= (med_unit_inventory_of neuro_surgery tongue_depressor) 0)
            (= (med_unit_inventory_of neuro_surgery aspirin) 0)

            ;(= (med_unit_inventory_of day_hospital aspirin) 0)
            ;(= (med_unit_inventory_of day_hospital scalpel) 0)
            ;(= (med_unit_inventory_of day_hospital tongue_depressor) 0)
;
            ;(= (med_unit_inventory_of cardiology tongue_depressor)0 )
            ;(= (med_unit_inventory_of cardiology aspirin) 0)
            ;(= (med_unit_inventory_of cardiology scalpel) 0)
;
            ;(= (med_unit_inventory_of radiology aspirin) 0)
            ;(= (med_unit_inventory_of radiology scalpel) 0)
            ;(= (med_unit_inventory_of radiology tongue_depressor) 0)
;
            ;(= (med_unit_inventory_of dentistry aspirin) 0)
            ;(= (med_unit_inventory_of dentistry tongue_depressor) 0)
            ;(= (med_unit_inventory_of dentistry scalpel) 0)
;
            ;(= 0 (med_unit_inventory_of neuro_surgery scalpel))
            ;(= (med_unit_inventory_of neuro_surgery tongue_depressor) 0)
            ;(= (med_unit_inventory_of neuro_surgery aspirin) 0)

            (patient_at rocco cardiology)
            (patient_at ciro day_hospital)
        )
    )
)
