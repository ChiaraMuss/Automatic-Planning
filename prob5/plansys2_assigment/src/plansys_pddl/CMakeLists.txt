cmake_minimum_required(VERSION 3.8)
project(plansys_pddl)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Define dependencies
set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

# Create executables based on the action names, NOT the .cpp filenames
add_executable(move_out_med_unit_delivery_robot src/MoveOutMedUnitDeliveryRobot.cpp)
ament_target_dependencies(move_out_med_unit_delivery_robot ${dependencies})

add_executable(deliver_patient src/DeliverPatient.cpp)
ament_target_dependencies(deliver_patient ${dependencies})

add_executable(load_to_drone_carrier src/LoadToDroneCarrier.cpp)
ament_target_dependencies(load_to_drone_carrier ${dependencies})

add_executable(load_to_robot_carrier_inventory src/LoadToRobotCarrierInventory.cpp)
ament_target_dependencies(load_to_robot_carrier_inventory ${dependencies})

add_executable(move_between_location_accompany_robot src/MoveBetweenLocationAccompanyRobot.cpp)
ament_target_dependencies(move_between_location_accompany_robot ${dependencies})

add_executable(move_between_location_delivery_robot src/MoveBetweenLocationDeliveryRobot.cpp)
ament_target_dependencies(move_between_location_delivery_robot ${dependencies})

add_executable(move_drone src/MoveDrone.cpp)
ament_target_dependencies(move_drone ${dependencies})

add_executable(move_in_inventory_delivery_robot src/MoveInInventoryDeliveryRobot.cpp)
ament_target_dependencies(move_in_inventory_delivery_robot ${dependencies})

add_executable(move_in_med_unit_accompany_robot src/MoveInMedUnitAccompanyRobot.cpp)
ament_target_dependencies(move_in_med_unit_accompany_robot ${dependencies})

add_executable(move_in_med_unit_delivery_robot src/MoveInMedUnitDeliveryRobot.cpp)
ament_target_dependencies(move_in_med_unit_delivery_robot ${dependencies})

add_executable(move_out_inventory_delivery_robot src/MoveOutInventoryDeliveryRobot.cpp)
ament_target_dependencies(move_out_inventory_delivery_robot ${dependencies})

add_executable(move_out_med_unit_accompany_robot src/MoveOutMedUnitAccompanyRobot.cpp)
ament_target_dependencies(move_out_med_unit_accompany_robot ${dependencies})

add_executable(take_patient src/TakePatient.cpp)
ament_target_dependencies(take_patient ${dependencies})

add_executable(unload_content src/UnloadContent.cpp)
ament_target_dependencies(unload_content ${dependencies})

add_executable(unload_from_drone_carrier src/UnloadFromDroneCarrier.cpp)
ament_target_dependencies(unload_from_drone_carrier ${dependencies})

add_executable(unload_from_robot_carrier_inventory src/UnloadFromRobotCarrierInventory.cpp)
ament_target_dependencies(unload_from_robot_carrier_inventory ${dependencies})

add_executable(unload_from_robot_carrier_medical_unit src/UnloadFromRobotCarrierMedicalUnit.cpp)
ament_target_dependencies(unload_from_robot_carrier_medical_unit ${dependencies})

add_executable(check_obstacles_node src/check_obstacles_node.cpp)
ament_target_dependencies(check_obstacles_node ${dependencies})

# Install configuration and PDDL files
install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

# Install executables using the action names, NOT the .cpp filenames
install(TARGETS
  move_out_med_unit_delivery_robot
  deliver_patient
  load_to_drone_carrier
  load_to_robot_carrier_inventory
  move_between_location_accompany_robot
  move_between_location_delivery_robot
  move_drone
  move_in_inventory_delivery_robot
  move_in_med_unit_accompany_robot
  move_in_med_unit_delivery_robot
  move_out_inventory_delivery_robot
  move_out_med_unit_accompany_robot
  take_patient
  unload_content
  unload_from_drone_carrier
  unload_from_robot_carrier_inventory
  unload_from_robot_carrier_medical_unit
  check_obstacles_node

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Enable testing section (if BUILD_TESTING is active)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

# Export dependencies and finalize the package
ament_export_dependencies(${dependencies})
ament_package()

