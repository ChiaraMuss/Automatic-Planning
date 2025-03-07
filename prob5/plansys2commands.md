docker exec -it [container-id] bash # connect new terminal to already running docker

colcon build --packages-select <name-of-pkg> --symlink-install # rebuild single package 

"source install/setup.zsh" or "source install/setup.sh" #make the terminal aware of the compiled packages depending on the terminal type

## How to run

In terminal 1:

``` shell
ros2 launch plansys_pddl healthcare_plansys_launch.py
```

In terminal 2:

``` shell
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal
```

From inside plansys2_terminal for running the problem_commands

``` shell
source src/plansys_pddl/pddl/problem_commands
```

The goal should be setted with

``` shell
set goal (and (med_unit_inventory_of day_hospital aspirin capacity3)(med_unit_inventory_of cardiology tongue_depressor capacity1)(med_unit_inventory_of dentistry aspirin capacity2)(med_unit_inventory_of dentistry tongue_depressor capacity2)(med_unit_inventory_of neuro_surgery scalpel capacity2)(patient_at_med_unit rocco cardiology)(patient_at_med_unit ciro day_hospital))
```
