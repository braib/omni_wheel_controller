# Omni Wheel Controller

ROS 2 Humble controller for 3-wheel omni-directional robots.

## Features

- Subscribes to `/cmd_vel` (geometry_msgs/Twist)
- Computes inverse kinematics for 3 omni wheels
- Publishes odometry (`/odom`) and TF transform (`odom` → `base_link`)
- Configurable wheel angles and parameters
- Command timeout watchdog

## Building

```bash
cd ~/ros2_ws/src
# Clone or copy the omni_wheel_controller package here
cd ~/ros2_ws
colcon build --packages-select omni_wheel_controller
source install/setup.bash
```

## Configuration

Edit `config/controller.yaml` to set:
- Wheel joint names
- Wheel radius and center-to-wheel distance
- Wheel angles (base_rotation_deg, wheel_spacing_deg)
- Command timeout
- Odometry covariances

## Usage

### With Controller Manager

```bash
# Load and activate the controller
ros2 control load_controller omni_wheel_controller
ros2 control set_controller_state omni_wheel_controller active

# Or using spawner
ros2 run controller_manager spawner omni_wheel_controller --controller-manager /controller_manager
```

### Send Commands

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Strafe right
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### View Odometry

```bash
ros2 topic echo /odom
```

### View TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

## Kinematics

The controller uses standard omni-wheel kinematics:

**Inverse Kinematics** (cmd_vel → wheel speeds):
```
ω_i = (1/r) * (vx*cos(θ_i) + vy*sin(θ_i) + L*ω_robot)
```

**Forward Kinematics** (wheel speeds → robot velocity):
Solves the inverse problem using a 3x3 linear system with Cramer's rule.

Where:
- `r` = wheel radius
- `L` = center to wheel distance
- `θ_i` = wheel angle for wheel i
- `ω_i` = angular velocity of wheel i (rad/s)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_joints` | string[] | ["wheel1", "wheel2", "wheel3"] | Joint names |
| `wheel_radius` | double | 0.05 | Wheel radius (m) |
| `center_to_wheel` | double | 0.15 | Distance from center to wheel (m) |
| `max_wheel_rad_s` | double | 50.0 | Max wheel angular velocity (rad/s) |
| `cmd_topic` | string | "/cmd_vel" | Command topic |
| `cmd_timeout_ms` | int | 500 | Command timeout (ms) |
| `base_rotation_deg` | double | 0.0 | Base rotation offset (degrees) |
| `wheel_spacing_deg` | double | 120.0 | Angle between wheels (degrees) |
| `odom_frame` | string | "odom" | Odometry frame ID |
| `base_frame` | string | "base_link" | Base frame ID |
| `cov_x` | double | 0.001 | X position covariance |
| `cov_y` | double | 0.001 | Y position covariance |
| `cov_yaw` | double | 0.001 | Yaw covariance |

## Troubleshooting

### Controller fails to load

Check that:
1. Plugin is properly exported in `plugin_description.xml`
2. Library is built correctly (`colcon build`)
3. Environment is sourced (`source install/setup.bash`)

### Wheels not moving

1. Verify hardware interface provides velocity command interfaces
2. Check joint names match your URDF/hardware
3. Verify controller is active: `ros2 control list_controllers`

### Odometry drift

Adjust covariance parameters (`cov_x`, `cov_y`, `cov_yaw`) based on your robot's characteristics.

## License

Apache-2.0











ros2 control list_controllers


ros2 control load_controller my_joint_controller


ros2 control load_controller my_controller --set-state configure


ros2 control set_controller_state <controller_name> <target_state>

ros2 control set_controller_state my_controller configure
ros2 control set_controller_state my_controller start
ros2 control set_controller_state my_controller stop







# Quick Reference Cheat Sheet
## View everything
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control list_controller_types

## Controller lifecycle
ros2 control load_controller NAME
ros2 control set_controller_state NAME configure
ros2 control set_controller_state NAME start
ros2 control set_controller_state NAME stop
ros2 control set_controller_state NAME cleanup
ros2 control unload_controller NAME

## Hot-swap
ros2 control switch_controllers --activate NEW --deactivate OLD

## Development
ros2 control reload_controller_libraries










## List controllers
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

## Load controller
ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: 'my_controller'}"

## Switch controllers
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['controller1'], deactivate_controllers: ['controller2']}"