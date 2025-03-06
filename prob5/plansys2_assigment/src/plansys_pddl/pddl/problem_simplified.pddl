(define (problem healthcare_problem)
    (:domain temporal_healthcare_simplified)

    (:objects
        central_warehouse heliport_alpha - inventory
        entrance sector_a sector_b sector_c - location
        scalpel tongue_depressor aspirin  - content
        box1 box2 box3 box4 box5 box6 box7 box8 box9 box10 - box
        delivery_robot1 delivery_robot2 - delivery_robot
        accompany_robot1 - accompany_robot
        drone1 drone2 - drone
        carrier1 carrier2 carrier3 carrier4 - carrier
        rocco ciro - patient
        cardiology dentistry radiology neuro_surgery day_hospital - medical_unit
        capacity0 capacity1 capacity2 capacity3 capacity4 capacity5 capacity6 capacity7 - capacity-number

    )

    (:init
        
        ;##added predecession predicates
        (capacity-predecessor capacity0 capacity1)
        (capacity-predecessor capacity1 capacity2)
        (capacity-predecessor capacity2 capacity3)
        (capacity-predecessor capacity3 capacity4)
        (capacity-predecessor capacity4 capacity5)
        (capacity-predecessor capacity5 capacity6)
        (capacity-predecessor capacity6 capacity7)

        ;; Initial positions of robots, drones, and patients
        (delivery_robot_at_inventory delivery_robot1 central_warehouse)
        (delivery_robot_at_inventory delivery_robot2 central_warehouse)
        (accompany_robot_at_location accompany_robot1 entrance)
        (drone_at drone1 central_warehouse)
        (drone_at drone2 central_warehouse)
        (patient_at_location rocco entrance)
        (patient_at_location ciro entrance)
        ;
        ;;; Accompanying robot status
        (free_to_accompany accompany_robot1)
        ;
        ;; Initial positions of boxes
        (at_box_inventory box1 central_warehouse)
        (at_box_inventory box2 central_warehouse)
        (at_box_inventory box3 central_warehouse)
        (at_box_inventory box4 central_warehouse)
        (at_box_inventory box5 central_warehouse)
        (at_box_inventory box6 central_warehouse)
        (at_box_inventory box7 central_warehouse)
        (at_box_inventory box8 central_warehouse)
        (at_box_inventory box9 central_warehouse)
        (at_box_inventory box10 central_warehouse)
        ;
        ;; Initial carrier positions and assignments
        (delivery_robot_has_carrier delivery_robot1 carrier1)
        (delivery_robot_has_carrier delivery_robot2 carrier2)
        (drone_has_carrier drone1 carrier3)
        ;
        ;; Carrier capacities
        (carrier_capacity carrier1 capacity3)
        (carrier_capacity carrier2 capacity2)
        (carrier_capacity carrier3 capacity2)
        (carrier_capacity carrier4 capacity5)

        ;;; Inventory
        (med_unit_inventory_of day_hospital aspirin capacity0)
        (med_unit_inventory_of day_hospital scalpel capacity0)
        (med_unit_inventory_of day_hospital tongue_depressor capacity0)
        ;;
        (med_unit_inventory_of neuro_surgery aspirin capacity0)
        (med_unit_inventory_of neuro_surgery scalpel capacity0)
        (med_unit_inventory_of neuro_surgery tongue_depressor capacity0)
        ;;
        (med_unit_inventory_of radiology aspirin capacity0)
        (med_unit_inventory_of radiology scalpel capacity0)
        (med_unit_inventory_of radiology tongue_depressor capacity0)
        ;;
        (med_unit_inventory_of cardiology aspirin capacity0)
        (med_unit_inventory_of cardiology scalpel capacity0)
        (med_unit_inventory_of cardiology tongue_depressor capacity0)
        ;;
        (med_unit_inventory_of dentistry aspirin capacity0)
        (med_unit_inventory_of dentistry scalpel capacity0)
        (med_unit_inventory_of dentistry tongue_depressor capacity0)

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
        (connection_location_inventory sector_a central_warehouse) ;41m
        
        (connection_between_locations sector_a sector_b)
        (connection_between_locations sector_b sector_a)
        
        (connection_between_locations sector_b sector_c)
        (connection_between_locations sector_c sector_b);79m
        

        (connection_location_med_unit sector_c day_hospital)
        
        (connection_between_locations entrance sector_c)
        (connection_between_locations sector_c entrance);27m
        
        (connection_location_inventory sector_c heliport_alpha)
        
        (connection_location_med_unit sector_b neuro_surgery)
        ;## Removed connection between medunits and heliport


        (connection_location_med_unit sector_b radiology)

        (connection_location_med_unit sector_a cardiology)
        (connection_location_med_unit sector_a dentistry)
        
        ; Drone port available
        (has_drone_port heliport_alpha);129m
        (has_drone_port central_warehouse)
        
    )
    (:goal
        (and
            (med_unit_inventory_of day_hospital aspirin capacity3)
            (med_unit_inventory_of cardiology tongue_depressor capacity1)
            (med_unit_inventory_of dentistry aspirin capacity2)
            (med_unit_inventory_of dentistry tongue_depressor capacity2)
            (med_unit_inventory_of neuro_surgery scalpel capacity2)
            (patient_at_med_unit rocco cardiology)
            (patient_at_med_unit ciro day_hospital)
        )
    )
)
