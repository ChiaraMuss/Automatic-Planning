(define (problem healthcare_problem)
    (:domain temporal_healthcare)

    (:objects
        entrance sector_a sector_b sector_c central_warehouse - generic_location
        accompany_robot1 - accompany_robot
        rocco ciro - patient

    )

    (:init
        (robot_at accompany_robot1 central_warehouse)

        ; - distances in meters and durations in seconds
        (=(speed accompany_robot1) 1.1)
        (connected central_warehouse sector_a) ;41m
        (=(distance central_warehouse sector_a) 1)
        (connected sector_a central_warehouse)
        (=(distance sector_a central_warehouse) 1)

    )
    (:goal
        (and
            (robot_at accompany_robot1 sector_a)
        )
    )
)