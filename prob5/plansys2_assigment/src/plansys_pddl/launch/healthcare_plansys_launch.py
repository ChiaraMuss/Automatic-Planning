# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory('plansys_pddl')
    namespace = LaunchConfiguration('namespace')

    # Declare namespace argument
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )

    # Include PlanSys2 bringup launch
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': package_dir + '/pddl/domainsimplified.pddl',
            'namespace': namespace
        }.items()
    )

    # ----------------------------------------------- ROBOT ACTIONS
    check_obstacles_cmd = Node(
        package='plansys_pddl',
        executable='check_obstacles_node',
        name='check_obstacles_node',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_between_location_delivery_cmd = Node(
        package='plansys_pddl',
        executable='move_between_location_delivery_robot',
        name='move_between_location_delivery_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_between_location_accompany_cmd = Node(
        package='plansys_pddl',
        executable='move_between_location_accompany_robot',
        name='move_between_location_accompany_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_in_inventory_delivery_cmd = Node(
        package='plansys_pddl',
        executable='move_in_inventory_delivery_robot',
        name='move_in_inventory_delivery_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_in_med_unit_delivery_cmd = Node(
        package='plansys_pddl',
        executable='move_in_med_unit_delivery_robot',
        name='move_in_med_unit_delivery_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_in_med_unit_accompany_cmd = Node(
        package='plansys_pddl',
        executable='move_in_med_unit_accompany_robot',
        name='move_in_med_unit_accompany_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_out_med_unit_delivery_cmd = Node(
        package='plansys_pddl',
        executable='move_out_med_unit_delivery_robot',
        name='move_out_med_unit_delivery_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_out_inventory_delivery_cmd = Node(
        package='plansys_pddl',
        executable='move_out_inventory_delivery_robot',
        name='move_out_inventory_delivery_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_out_med_unit_accompany_cmd = Node(
        package='plansys_pddl',
        executable='move_out_med_unit_accompany_robot',
        name='move_out_med_unit_accompany_robot',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    # ----------------------------------------------- CARRIER MANAGEMENT
    load_to_carrier_cmd = Node(
        package='plansys_pddl',
        executable='load_to_robot_carrier_inventory',
        name='load_to_robot_carrier_inventory',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    unload_from_carrier_cmd = Node(
        package='plansys_pddl',
        executable='unload_from_robot_carrier_inventory',
        name='unload_from_robot_carrier_inventory',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    unload_from_carrier_medical_cmd = Node(
        package='plansys_pddl',
        executable='unload_from_robot_carrier_medical_unit',
        name='unload_from_robot_carrier_medical_unit',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    unload_content_cmd = Node(
        package='plansys_pddl',
        executable='unload_content',
        name='unload_content',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    # ----------------------------------------------- DRONE CAPABILITIES
    load_to_drone_cmd = Node(
        package='plansys_pddl',
        executable='load_to_drone_carrier',
        name='load_to_drone_carrier',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    unload_from_drone_cmd = Node(
        package='plansys_pddl',
        executable='unload_from_drone_carrier',
        name='unload_from_drone_carrier',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    move_drone_cmd = Node(
        package='plansys_pddl',
        executable='move_drone',
        name='move_drone',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    # ---------------------------------------------- PATIENT MANAGEMENT
    take_patient_cmd = Node(
        package='plansys_pddl',
        executable='take_patient',
        name='take_patient',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    deliver_patient_cmd = Node(
        package='plansys_pddl',
        executable='deliver_patient',
        name='deliver_patient',
        namespace=namespace,
        output='screen',
        parameters=[]
    )

    # ----------------------------------------------- LAUNCH DESCRIPTION
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_namespace_cmd)

    # Add core PlanSys2 launch
    ld.add_action(plansys2_cmd)

    # Add robot movement nodes
    ld.add_action(check_obstacles_cmd)
    ld.add_action(move_between_location_delivery_cmd)
    ld.add_action(move_between_location_accompany_cmd)
    ld.add_action(move_in_inventory_delivery_cmd)
    ld.add_action(move_in_med_unit_delivery_cmd)
    ld.add_action(move_in_med_unit_accompany_cmd)
    ld.add_action(move_out_med_unit_delivery_cmd)
    ld.add_action(move_out_inventory_delivery_cmd)
    ld.add_action(move_out_med_unit_accompany_cmd)

    # Add carrier nodes
    ld.add_action(load_to_carrier_cmd)
    ld.add_action(unload_from_carrier_cmd)
    ld.add_action(unload_from_carrier_medical_cmd)
    ld.add_action(unload_content_cmd)

    # Add drone nodes
    ld.add_action(load_to_drone_cmd)
    ld.add_action(unload_from_drone_cmd)
    ld.add_action(move_drone_cmd)

    # Add patient management nodes
    ld.add_action(take_patient_cmd)
    ld.add_action(deliver_patient_cmd)

    return ld

