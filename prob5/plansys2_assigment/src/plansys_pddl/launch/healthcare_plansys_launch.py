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
    # Get the launch directory
    example_dir = get_package_share_directory('plansys_pddl')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain4temporalnew.pddl',
          'namespace': namespace
          }.items())

    """     # Specify the actions
        move_cmd = Node(
            package='plansys_pddl',
            executable='move_action_node',
            name='move_action_node',
            namespace=namespace,
            output='screen',
            parameters=[])

        charge_cmd = Node(
            package='plansys_pddl',
            executable='charge_action_node',
            name='charge_action_node',
            namespace=namespace,
            output='screen',
            parameters=[])

        ask_charge_cmd = Node(
            package='plansys_pddl',
            executable='ask_charge_action_node',
            name='ask_charge_action_node',
            namespace=namespace,
            output='screen',
            parameters=[])   # Create the launch description and populate
        """
    check_obstacles_cmd = Node(
        package='plansys_pddl',
        executable='check_obstacles_node',
        name='check_obstacles_node',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
            
    move_cmd = Node(
        package='plansys_pddl',
        executable='move_action_node',
        name='move_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    load_to_carrier_cmd = Node(
        package='plansys_pddl',
        executable='load_to_robot_carrier_node',
        name='load_to_robot_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    unload_from_carrier_cmd = Node(
        package='plansys_pddl',
        executable='unload_from_robot_carrier_node',
        name='unload_from_robot_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    unload_content_cmd = Node(
        package='plansys_pddl',
        executable='unload_content_node',
        name='unload_content_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    # ----------------------------------------------- DRONE CAPABILITIES
    load_to_drone_cmd = Node(
        package='plansys_pddl',
        executable='load_to_drone_carrier_node',
        name='load_to_drone_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    unload_from_drone_cmd = Node(
        package='plansys_pddl',
        executable='unload_from_drone_carrier_node',
        name='unload_from_drone_carrier_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    
    move_drone_cmd = Node(
        package='plansys_pddl',
        executable='move_drone_node',
        name='move_drone_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    # ---------------------------------------------- PATIENTS MANAGEMENT
    take_patient_cmd = Node(
        package='plansys_pddl',
        executable='take_patient_node',
        name='take_patient_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    deliver_patient_cmd = Node(
        package='plansys_pddl',
        executable='deliver_patient_node',
        name='deliver_patient_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(move_cmd)
    ld.add_action(check_obstacles_cmd)


    ld.add_action(load_to_carrier_cmd)
    ld.add_action(unload_from_carrier_cmd)
    ld.add_action(unload_content_cmd)
    ld.add_action(load_to_drone_cmd)
    ld.add_action(unload_from_drone_cmd)
    ld.add_action(move_drone_cmd)
    ld.add_action(take_patient_cmd)
    ld.add_action(deliver_patient_cmd)
    
    return ld
