#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Minimal robot description with mock hardware for 3 omni wheels
    # IMPORTANT: Joint names MUST match the wheel_names in your controller config
    robot_description_content = \
"""<?xml version="1.0"?>
<robot name="test_omni_robot">
  <ros2_control name="OmniSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="fake_sensor_commands">false</param>
      <param name="state_following_offset">0.0</param>
    </hardware>
    
    <!-- Match these names with wheel_names in controller config -->
    <joint name="wheel_1_joint">
      <command_interface name="velocity">
        <param name="min">-100</param>
        <param name="max">100</param>
      </command_interface>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    
    <joint name="wheel_2_joint">
      <command_interface name="velocity">
        <param name="min">-100</param>
        <param name="max">100</param>
      </command_interface>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    
    <joint name="wheel_3_joint">
      <command_interface name="velocity">
        <param name="min">-100</param>
        <param name="max">100</param>
      </command_interface>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
"""

    # Get controller config file path (correct filename!)
    pkg_share = get_package_share_directory('omni_wheel_controller')
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')

    # Controller manager node with robot description and controller config
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config
        ],
        output='screen',
        emulate_tty=True,
    )

    # Spawn the omni wheel controller after a short delay
    spawn_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_wheel_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay spawner to let controller_manager initialize
    delayed_spawn = TimerAction(
        period=3.0,  # Increased delay to ensure controller_manager is ready
        actions=[spawn_controller]
    )

    return LaunchDescription([
        controller_manager_node,
        delayed_spawn,
    ])