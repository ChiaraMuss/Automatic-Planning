(define (problem problem_1) (:domain healthcare_vm)
    (:objects
        central_warehouse - inventory
        entrance - waiting_room
        padiglione_a padiglione_b padiglione_c - location
        box1 box2 - box
        robot1 robot2 - delivery_robot
        accompany_robot1 accompany_robot2 - accompany_robot ; New robot type for accompanying patients
        carlo franco - patient
        pronto_soccorso ambulatorio neurologia odontoiatria - medical_unit ; Separate objects for medical units
        scalpel1 scalpel2 - scalpel
        aspirin1 aspirin2 - aspirin
        tongue_depressor1 tongue_depressor2 - tongue_depressor
    )

    (:init
        (connected entrance padiglione_c)
        (connected padiglione_c entrance)
        (connected padiglione_a padiglione_b)
        (connected padiglione_b padiglione_a)
        (connected padiglione_c padiglione_b)
        (connected padiglione_b padiglione_c)

        (med_unit_at pronto_soccorso padiglione_c)
        (med_unit_at ambulatorio padiglione_c)
        (med_unit_at neurologia padiglione_b)
        (med_unit_at odontoiatria padiglione_a)

        ;Initially all boxes are located at a single location that we may call the central warehouse.
        ;All the contents to load in the boxes are initially located at the central warehouse.
        ; single robotic agent allowed to carry boxes is located at the central warehouse to deliver boxes.
        ; single robotic agent allowed to accompany patients is initially located at the entrance.
    )

(:goal (and
    ;certain medical units have certain supplies;
    ;some medical units might not need supply;
    ;some medical units might need several supplies;
    ; some patients need to reach some medical unit.
        ))
)
