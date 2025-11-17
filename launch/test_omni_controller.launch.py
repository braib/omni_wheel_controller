#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Minimal robot description with mock hardware for 3 omni wheels
    robot_description_content = """<?xml version="1.0"?>
<robot name="test_omni_robot">
  <ros2_control name="OmniSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="fake_sensor_commands">false</param>
      <param name="state_following_offset">0.0</param>
    </hardware>
    
    <joint name="wheel1">
      <command_interface name="velocity">
        <param name="min">-50</param>
        <param name="max">50</param>
      </command_interface>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    
    <joint name="wheel2">
      <command_interface name="velocity">
        <param name="min">-50</param>
        <param name="max">50</param>
      </command_interface>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    
    <joint name="wheel3">
      <command_interface name="velocity">
        <param name="min">-50</param>
        <param name="max">50</param>
      </command_interface>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
"""
    
    # Get controller config file path
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
        arguments=['omni_wheel_controller'],
        output='screen',
    )
    
    # Delay spawner to let controller_manager initialize
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_controller]
    )
    
    return LaunchDescription([
        controller_manager_node,
        delayed_spawn,
    ])