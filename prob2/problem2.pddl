(define (problem healthcare-problem)
  (:domain healthcare)

  (:objects
    central_warehouse entrance location1 location2 - location
    scalpel tongue_depressor aspirin - content
    box1 box2 box3 - box
    robot1 robot2 - robot
    accompany_robot1 - accompany_robot
    carrier1 - carrier  ; The carrier for the robot
    patient1 patient2 - patient
    medical_unit1 medical_unit2 - medical_unit  ; Separate objects for medical units
    capacity3 capacity2 - number  ; Define capacities for the robots
  )

  (:init
    ; Initial positions of robots and patients
    (robot_at robot1 central_warehouse)
    (robot_at robot2 entrance)
    (robot_at accompany_robot1 entrance)
    (patient_at patient1 entrance)    ; Patient 1 is at the entrance (location)
    (patient_at patient2 entrance)    ; Patient 2 is at the entrance (location)

    ; Initial positions of boxes and contents
    (at_box box1 central_warehouse)
    (at_box box2 central_warehouse)
    (at_box box3 central_warehouse)
    (at_content scalpel central_warehouse)
    (at_content tongue_depressor central_warehouse)
    (at_content aspirin central_warehouse)

    ; Initial carrier capacities for the robots
    (carrier_capacity robot1 capacity3)  ; Robot1 can carry up to 3 boxes
    (carrier_capacity robot2 capacity2)  ; Robot2 can carry up to 2 boxes

    ; Initial box statuses (none of the boxes are on the carriers initially)
    (not (on_carrier robot1 box1))  ; Box1 is not on the carrier initially
    (not (on_carrier robot1 box2))  ; Box2 is not on the carrier initially
    (not (on_carrier robot2 box3))  ; Box3 is not on the carrier initially

    ; Connections between locations
    (connected central_warehouse entrance)
    (connected entrance location1)
    (connected location1 location2)

    ; Associate medical units with locations
    (at_location medical_unit1 location1)  ; Medical unit 1 is located at location1
    (at_location medical_unit2 location2)  ; Medical unit 2 is located at location2

    ; Medical units require specific contents
    (has medical_unit1 scalpel)
    (has medical_unit2 aspirin)
  )

  (:goal
    (and
      (has medical_unit1 scalpel)       ; Medical unit 1 has scalpel
      (has medical_unit2 aspirin)      ; Medical unit 2 has aspirin
      (patient_at patient1 location1)  ; Patient 1 reaches location1 (associated with medical_unit1)
      (patient_at patient2 location2)  ; Patient 2 reaches location2 (associated with medical_unit2)
    )
  )
)