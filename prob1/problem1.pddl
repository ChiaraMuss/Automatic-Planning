;; Define the healthcare logistics problem
(define (problem healthcare-problem)
  (:domain healthcare)  ;; The problem belongs to the "healthcare" domain

  ;; Define objects used in the problem
  (:objects
    central_warehouse entrance location1 location2 location3 - location  ;; Locations in the environment
    scalpel tongue_depressor aspirin - content  ;; Different types of medical supplies
    box1 box2 box3 box4 box5 - box  ;; Boxes that contain medical supplies
    delivery_robot1 - delivery_robot  ;; Delivery robot responsible for transporting supplies
    accompany_robot1 - accompany_robot  ;; Accompanying robot responsible for patient transport
    patient1 patient2 - patient  ;; Patients who need to be transported
    medical_unit1 medical_unit2 medical_unit3 - medical_unit  ;; Medical units where patients and supplies need to be delivered
  )

  ;; Define the initial state of the environment
  (:init
    ;; Initial positions of the robots
    (robot_at delivery_robot1 central_warehouse)  ;; The delivery robot starts at the central warehouse
    (robot_at accompany_robot1 entrance)  ;; The accompanying robot starts at the entrance
    (free_to_accompany accompany_robot1)  ;; The accompanying robot is initially free
    (hands_free delivery_robot1)  ;; The delivery robot is initially not carrying anything

    ;; Initial positions of the patients
    (patient_at patient1 entrance)  ;; Patient 1 starts at the entrance
    (patient_at patient2 entrance)  ;; Patient 2 starts at the entrance

    ;; Initial positions of the boxes
    (at_box box1 central_warehouse)
    (at_box box2 central_warehouse)
    (at_box box3 central_warehouse)
    (at_box box4 central_warehouse)
    (at_box box5 central_warehouse)

    ;; Contents of the boxes
    (contains box1 scalpel)  ;; Box 1 contains a scalpel
    (contains box2 tongue_depressor)  ;; Box 2 contains a tongue depressor
    (contains box3 aspirin)  ;; Box 3 contains aspirin
    (contains box4 scalpel)  ;; Box 4 contains a scalpel
    (contains box5 aspirin)  ;; Box 5 contains aspirin

    ;; Define the medical unit supply requirements
    (needs_supply medical_unit1 scalpel)  ;; Medical unit 1 requires a scalpel
    (needs_supply medical_unit1 aspirin)  ;; Medical unit 1 requires aspirin
    ;; Medical unit 2 does not require any supplies, so it is not included here
    (needs_supply medical_unit3 aspirin)  ;; Medical unit 3 requires aspirin

    ;; Define bidirectional connections between locations
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

    ;; Uncomment these lines if direct connections from the entrance to medical units are needed
    ;(connected entrance medical_unit1)
    ;(connected entrance medical_unit2)
    ;(connected entrance medical_unit3)
  )

  ;; Define the goal conditions
  (:goal
    (and
      ;; Medical units must receive only the supplies they need
      (medical_supplied medical_unit1 scalpel)
      (medical_supplied medical_unit1 aspirin)
      (medical_supplied medical_unit3 aspirin)

      ;; Patients must be transported to the appropriate medical units
      (patient_at patient1 medical_unit1)
      (patient_at patient2 medical_unit2)
    )
  )
)

