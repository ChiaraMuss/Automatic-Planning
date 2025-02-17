cmake_minimum_required(VERSION 3.8)
project(plansys_pddl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(check_obstacles_node src/check_obstacles_node.cpp)
ament_target_dependencies(check_obstacles_node ${dependencies})

#add_executable(charge_action_node src/charge_action_node.cpp)
#ament_target_dependencies(charge_action_node ${dependencies})

#add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
#ament_target_dependencies(ask_charge_action_node ${dependencies})

add_executable(deliver_patient_node src/deliver_patient_node.cpp)
ament_target_dependencies(deliver_patient_node ${dependencies})

add_executable(load_to_drone_carrier_node src/load_to_drone_carrier_node.cpp)
ament_target_dependencies(load_to_drone_carrier_node ${dependencies})

add_executable(load_to_robot_carrier_node src/load_to_robot_carrier_node.cpp)
ament_target_dependencies(load_to_robot_carrier_node ${dependencies})

add_executable(move_drone_node src/move_drone_node.cpp)
ament_target_dependencies(move_drone_node ${dependencies})

add_executable(take_patient_node src/take_patient_node.cpp)
ament_target_dependencies(take_patient_node ${dependencies})

add_executable(unload_content_node src/unload_content_node.cpp)
ament_target_dependencies(unload_content_node ${dependencies})

add_executable(unload_from_drone_carrier_node src/unload_from_drone_carrier_node.cpp)
ament_target_dependencies(unload_from_drone_carrier_node ${dependencies})

add_executable(unload_from_robot_carrier_node src/unload_from_robot_carrier_node.cpp)
ament_target_dependencies(unload_from_robot_carrier_node ${dependencies})


install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  #charge_action_node
  #ask_charge_action_node
  check_obstacles_node

  deliver_patient_node
  load_to_drone_carrier_node
  load_to_robot_carrier_node
  move_drone_node
  take_patient_node
  unload_content_node
  unload_from_drone_carrier_node
  unload_from_robot_carrier_node

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
