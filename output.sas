begin_version
3
end_version
begin_metric
0
end_metric
4
begin_variable
var0
-1
4
Atom robot_at(accompany_robot2, central_warehouse)
Atom robot_at(accompany_robot2, entrance)
Atom robot_at(accompany_robot2, location1)
Atom robot_at(accompany_robot2, location2)
end_variable
begin_variable
var1
-1
3
Atom robot_at(accompany_robot1, entrance)
Atom robot_at(accompany_robot1, location1)
Atom robot_at(accompany_robot1, location2)
end_variable
begin_variable
var2
-1
3
Atom patient_at(patient2, entrance)
Atom patient_at(patient2, location1)
Atom patient_at(patient2, location2)
end_variable
begin_variable
var3
-1
3
Atom patient_at(patient1, entrance)
Atom patient_at(patient1, location1)
Atom patient_at(patient1, location2)
end_variable
0
begin_state
0
0
0
0
end_state
begin_goal
2
2 2
3 1
end_goal
13
begin_operator
accompany_patient accompany_robot1 patient1 entrance location1 medical_unit1
1
1 0
1
0 3 0 1
1
end_operator
begin_operator
accompany_patient accompany_robot1 patient1 location1 location2 medical_unit2
1
1 1
1
0 3 1 2
1
end_operator
begin_operator
accompany_patient accompany_robot1 patient2 entrance location1 medical_unit1
1
1 0
1
0 2 0 1
1
end_operator
begin_operator
accompany_patient accompany_robot1 patient2 location1 location2 medical_unit2
1
1 1
1
0 2 1 2
1
end_operator
begin_operator
accompany_patient accompany_robot2 patient1 entrance location1 medical_unit1
1
0 1
1
0 3 0 1
1
end_operator
begin_operator
accompany_patient accompany_robot2 patient1 location1 location2 medical_unit2
1
0 2
1
0 3 1 2
1
end_operator
begin_operator
accompany_patient accompany_robot2 patient2 entrance location1 medical_unit1
1
0 1
1
0 2 0 1
1
end_operator
begin_operator
accompany_patient accompany_robot2 patient2 location1 location2 medical_unit2
1
0 2
1
0 2 1 2
1
end_operator
begin_operator
move accompany_robot1 entrance location1
0
1
0 1 0 1
1
end_operator
begin_operator
move accompany_robot1 location1 location2
0
1
0 1 1 2
1
end_operator
begin_operator
move accompany_robot2 central_warehouse entrance
0
1
0 0 0 1
1
end_operator
begin_operator
move accompany_robot2 entrance location1
0
1
0 0 1 2
1
end_operator
begin_operator
move accompany_robot2 location1 location2
0
1
0 0 2 3
1
end_operator
0
