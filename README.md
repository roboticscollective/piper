# Piper Robot Arm — ROS 2 Workspace

ROS 2 Humble workspace for the Piper 6-DOF robot arm with a two-finger gripper. It supports:

- Real-robot control over CAN
- RViz visualization
- MoveIt 2 motion planning
- Gazebo Classic simulation through `gazebo_ros2_control`
- MoveIt Servo Cartesian and joint-space control
- SpaceMouse teleoperation through `spacenav_node`

## System Overview

| Component | Configuration |
|---|---|
| Robot | Piper 6-DOF arm with two-finger prismatic gripper |
| ROS distribution | ROS 2 Humble |
| Planning | MoveIt 2 |
| Servoing | MoveIt Servo |
| Simulation | Gazebo Classic with `gazebo_ros2_control` |
| Real-robot transport | USB-to-CAN adapter, typically `can0` |
| Teleoperation | Xbox controller or 3Dconnexion SpaceMouse |
| Container | Docker Compose |
| Workspace | `/workspace` inside container |

## Packages

| Package | Purpose |
|---|---|
| `piper_description` | URDF/Xacro robot model and meshes |
| `piper` | Piper robot driver and real-robot launch files |
| `piper_no_gripper_moveit` | MoveIt configuration without gripper |
| `piper_with_gripper_moveit` | MoveIt configuration with gripper, including SRDF, kinematics, and limits |
| `piper_gazebo` | Gazebo world, robot spawning, and `ros2_controllers.yaml` |
| `piper_servo` | Custom Servo launch files, Servo YAML, and `twist_stamper` node |

## Prerequisites

- Docker Engine and Docker Compose plugin
- An X11-capable Linux host for RViz/Gazebo GUI
- USB-to-CAN adapter and configured CAN interface for real hardware
- Xbox controller, if using joystick teleoperation
- 3Dconnexion SpaceMouse and `spacenav_node`, if using SpaceMouse teleoperation

## Repository Setup

Clone the repository together with its submodules:

```bash
git clone --recurse-submodules <REPOSITORY_URL>
cd <REPOSITORY_DIRECTORY>
```

If the repository was already cloned, initialize and update the submodules:

```bash
git submodule update --init --recursive
```

## Docker Setup

Allow the local Docker container to connect to the X server:

```bash
# Allow local Docker processes to access the X server
xhost +local:docker

# Revoke access after finishing GUI work
xhost -local:docker
```

> `xhost +local:docker` is convenient for local development, but broadens X-server access. Prefer an Xauthority-cookie setup on shared workstations.

Build and start the container:

```bash
docker compose up -d
```

Enter the running ROS container:

```bash
docker exec -it piper_ros_container bash
```

Build the workspace inside the container:

```bash
cd /workspace
colcon build
source install/setup.bash
```

For iterative development, rebuild only the modified package:

```bash
cd /workspace
colcon build --packages-select piper_servo
source install/setup.bash
```

> In every new container shell, source the ROS installation and workspace overlay before running ROS commands:
>
> ```bash
> source /opt/ros/humble/setup.bash
> source /workspace/install/setup.bash
> ```
>
> If the container image sources these automatically through `.bashrc`, no manual sourcing is needed.

## CAN Bus Setup

Configure CAN on the **host**, before starting the container and before launching the real robot:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Verify the interface:

```bash
ip -details link show can0
```

Bring the interface down when required:

```bash
sudo ip link set can0 down
```

## Real Robot

Launch the Piper driver with the gripper enabled:

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  gripper_exist:=true
```

Launch the driver with RViz:

```bash
ros2 launch piper start_single_piper_rviz.launch.py
```

Enable the robot after confirming that the environment is clear and the robot is safe to energize:

```bash
ros2 service call /enable_srv piper_msgs/srv/Enable \
  "enable_request: true"
```

> Do not use simulated controller commands or unverified Servo configurations on the physical robot without independently validating velocity, acceleration, workspace, collision, emergency-stop, and enabling behavior.

## MoveIt 2

Launch MoveIt without the gripper:

```bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

Launch MoveIt with the gripper:

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

Launch the gripper-equipped MoveIt configuration separately:

```bash
ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
```

## Gazebo Simulation

Start Gazebo Classic simulation:

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

The simulation uses `gazebo_ros2_control`, which connects Gazebo Classic to the ROS 2 controller architecture.

Check that controllers are active:

```bash
ros2 control list_controllers
```

The expected active arm trajectory controller is:

```text
joint_state_broadcaster  active
arm_controller           active
```

The controller is **not** named `joint_trajectory_controller`. A mismatch in this name can cause apparent Servo failures because Servo publishes commands to the wrong output topic.

## Simulation Quick Start

Open separate shells in the container, source the workspace overlay in each shell, then start the stack in this order.

Terminal 1 — start Gazebo:

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

Terminal 2 — start the gripper-equipped MoveIt configuration:

```bash
ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
```

Terminal 3 — start the custom Servo stack. It commands the configured ready pose before starting `servo_server`, RViz, and `twist_stamper`:

```bash
ros2 launch piper_servo piper_servo.launch.py
```

Terminal 4 — verify Servo health, then explicitly enable it:

```bash
ros2 topic echo /servo_server/status --once
ros2 service call /servo_server/start_servo std_srvs/srv/Trigger {}
```

Start `spacenav_node` on the host, outside Docker, if using a SpaceMouse.

## MoveIt Servo

MoveIt Servo accepts Cartesian twist commands or joint jog commands and sends generated trajectories to the active arm controller. Its real-time servoing functionality includes singularity handling, collision checks, and joint-limit enforcement.

### Command Interfaces

| Name | Message type | Purpose |
|---|---|---|
| `/spacenav/twist` | `geometry_msgs/msg/Twist` | Raw SpaceMouse command |
| `/servo_server/delta_twist_cmds` | `geometry_msgs/msg/TwistStamped` | Cartesian velocity command to Servo |
| `/servo_server/delta_joint_cmds` | `control_msgs/msg/JointJog` | Joint-space velocity command to Servo |
| `/servo_server/status` | `moveit_msgs/msg/ServoStatus` | Servo health/status; `0` indicates healthy |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Servo output to the arm controller |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | MoveIt planning/execution interface; Servo publishes to its configured trajectory topic |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot feedback |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo simulation clock |

### Start Servo

Servo commonly starts paused. Start it before publishing motion commands:

```bash
ros2 service call /servo_server/start_servo std_srvs/srv/Trigger {}
```

Pause it when required:

```bash
ros2 service call /servo_server/pause_servo std_srvs/srv/Trigger {}
```

Check status:

```bash
ros2 topic echo /servo_server/status --once
```

A status of `0` indicates no Servo warning or halt condition. Any nonzero value requires inspection of Servo logs and the `moveit_msgs/msg/ServoStatus` definition; common causes include singularity, collision, joint-bound, stale-command, or invalid-command conditions.

## SpaceMouse Teleoperation

`spacenav_node` runs **outside the Docker container** and publishes raw unstamped twist messages:

```text
/spacenav/twist
```

The `piper_servo` package provides `twist_stamper`, which:

- Subscribes to `cmd_vel_in`, normally remapped from `/spacenav/twist`
- Publishes `twist_out`, normally remapped to `/servo_server/delta_twist_cmds`
- Converts `geometry_msgs/msg/Twist` to `geometry_msgs/msg/TwistStamped`
- Uses current ROS node time and a configurable frame ID
- Defaults to the `base_link` frame

The Docker container and host must share DDS discovery correctly for the SpaceMouse topic to be discovered across the host-container boundary.

## Simulation Time

When Gazebo is active, all relevant nodes must use simulation time:

```yaml
use_sim_time: true
```

This includes at minimum:

- Gazebo-related nodes
- `robot_state_publisher`
- `move_group`
- `servo_server`
- RViz
- `twist_stamper`
- Any custom publisher or test node

Check that Gazebo publishes a clock:

```bash
ros2 topic echo /clock
```

### Timestamp Warning

Do not publish commands timestamped with wall-clock time when Servo uses Gazebo simulation time. Such commands may be silently considered stale because their timestamps do not match `/clock`.

For manually published stamped messages, use:

```yaml
stamp: now
```

For robust testing, prefer a proper `rclpy` or `rclcpp` publisher configured with `use_sim_time: true`.

## Servo Configuration

Key values from `piper_simulated_config.yaml`:

```yaml
use_gazebo: true

command_in_type: "unitless"

scale:
  linear: 0.3
  rotational: 0.5
  joint: 2.0

publish_period: 0.034
incoming_command_timeout: 0.1

command_out_type: trajectory_msgs/JointTrajectory
command_out_topic: /arm_controller/joint_trajectory

move_group_name: arm
planning_frame: base_link
ee_frame_name: link7
robot_link_command_frame: base_link

cartesian_command_in_topic: ~/delta_twist_cmds
joint_command_in_topic: ~/delta_joint_cmds

lower_singularity_threshold: 17.0
hard_stop_singularity_threshold: 30.0
joint_limit_margin: 0.1
```

The output topic must remain:

```yaml
command_out_topic: /arm_controller/joint_trajectory
```

Do **not** use the obsolete or incorrect topic:

```text
/joint_trajectory_controller/joint_trajectory
```

## Simulation Joint Parameters

> These values describe the Gazebo URDF currently used by this workspace. They are simulation parameters, not validated physical actuator limits or safety limits for the real robot.

| Joint | Type | Range | Effort | Velocity | Damping |
|---|---|---:|---:|---:|---:|
| `joint1` | Revolute | -2.618 to 2.168 rad | 100 Nm | 5 rad/s | 300 |
| `joint2` | Revolute | 0 to 3.14 rad | 100 Nm | 5 rad/s | 100 |
| `joint3` | Revolute | -2.967 to 0 rad | 100 Nm | 5 rad/s | 20 |
| `joint4` | Revolute | -1.745 to 1.745 rad | 70 Nm | 5 rad/s | 5 |
| `joint5` | Revolute | -1.22 to 1.22 rad | 50 Nm | 5 rad/s | 2 |
| `joint6` | Revolute | -2.0944 to 2.0944 rad | 6 Nm | 3 rad/s | 0.1 |
| `joint7` | Prismatic | 0 to 0.035 m | 10 N | 1 m/s | 100 |
| `joint8` | Prismatic | -0.035 to 0 m | 10 N | 1 m/s | 100 |

The damping values and reduced effort limits for joints 4 to 6 were derived from a validated MuJoCo representation and added to the URDF `<dynamics>` elements. The original URDF lacked joint dynamics, leading to loose and unrealistic Gazebo behavior.

Reference MuJoCo actuator gains, if migrating to effort-controlled `ros2_control` interfaces:

```text
joint1 = 10000
joint2 = 2000
joint3 = 500
joint4 = 50
joint5 = 20
joint6 = 5
joint7 = 10000
joint8 = 10000
```

## Startup Sequence

The custom Servo launch sequence is designed to avoid starting Cartesian Servo at a singular home configuration:

1. Spawn Gazebo and load controllers.
2. Command the robot to a bent ready pose.
3. Start `servo_server`.
4. Start RViz.
5. Start `twist_stamper`.
6. Start `spacenav_node` externally on the host, if using a SpaceMouse.
7. Call `/servo_server/start_servo`.
8. Send Cartesian or joint-space commands.

The launch file uses `ExecuteProcess` and `RegisterEventHandler(OnProcessExit)` to start Servo-related nodes after issuing the ready-pose trajectory.

> `ros2 topic pub --once` indicates only that a message was sent; it does not verify that the arm finished executing the motion. For strict startup sequencing, use a trajectory action client and wait for its result, or introduce a conservative `TimerAction` delay.

## Ready Pose and Singularities

The all-zero home pose is kinematically singular for this robot. In particular, `joint2 = 0` corresponds to a fully extended configuration within its range.

Do not start Cartesian Servo at the zero pose. Move first to a bent pose, for example:

```text
[0.0, 0.7, -1.0, 0.3, 0.8, 0.3]
```

This candidate pose offsets joints 4 and 6 as well as joints 2, 3, and 5, reducing the risk of wrist-axis alignment.

A previously used pose similar to:

```text
[0.0, 0.6, -0.6, 0.0, 0.3, 0.0]
```

can still be near a wrist singularity even though the shoulder and elbow are bent.

## Diagnostics

### Controller State

```bash
ros2 control list_controllers
```

Expected relevant controller:

```text
arm_controller
```

### Servo Health

```bash
ros2 topic echo /servo_server/status --once
```

Start or pause Servo:

```bash
ros2 service call /servo_server/start_servo std_srvs/srv/Trigger {}
ros2 service call /servo_server/pause_servo std_srvs/srv/Trigger {}
```

### Kinematics and TF

Confirm that the KDL solver is configured:

```bash
ros2 param get /servo_server \
  robot_description_kinematics.arm.kinematics_solver
```

Expected value:

```text
kdl_kinematics_plugin/KDLKinematicsPlugin
```

Verify the planning frame and end-effector transform:

```bash
ros2 run tf2_ros tf2_echo base_link link7
```

### Topic Wiring

Inspect Servo output configuration:

```bash
ros2 param get /servo_server moveit_servo.command_out_topic
```

Inspect publishers, subscribers, and QoS compatibility:

```bash
ros2 topic info /arm_controller/joint_trajectory --verbose
```

Expected wiring:

```text
Publisher:  servo_server
Subscriber: arm_controller
```

### Simulation Clock

```bash
ros2 topic echo /clock
```

### Joint Jog Test

Joint jogging bypasses the Cartesian Jacobian and is useful for isolating singularity-related failures:

```bash
ros2 topic pub --use-sim-time --rate 10 \
  /servo_server/delta_joint_cmds \
  control_msgs/msg/JointJog \
  "{header: {frame_id: 'base_link', stamp: now},
    joint_names: ['joint2', 'joint3'],
    velocities: [0.3, -0.3],
    duration: 0.1}"
```

### Cartesian Twist Test

Publish a raw test twist through the SpaceMouse input topic:

```bash
ros2 topic pub --rate 10 \
  /spacenav/twist \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Known Issues

### Servo Does Nothing

Check these items in order:

1. Verify that `arm_controller` is active.
2. Verify that `command_out_topic` is `/arm_controller/joint_trajectory`.
3. Confirm `use_sim_time: true` for all relevant simulation nodes and use `ros2 topic pub --use-sim-time` for CLI tests.
4. Confirm stamped commands use Gazebo time rather than wall-clock time.
5. Start Servo explicitly using `/servo_server/start_servo`.
6. Confirm `/servo_server/status` is `0`.
7. Check topic wiring with `ros2 topic info ... --verbose`.
8. Verify that MoveIt or another process is not unexpectedly switching controllers.

### `run_duration: 0.1 (0.034)` Warning

This generally reflects a mismatch between:

```yaml
incoming_command_timeout: 0.1
publish_period: 0.034
```

It is often cosmetic if commands flow reliably. A tighter configuration can use approximately two publish periods:

```yaml
incoming_command_timeout: 0.068
```

### Recurring Cartesian Singularity Stops

Symptoms:

```text
Very close to a singularity, emergency stop
```

Likely causes:

- Startup from an extended or wrist-aligned pose
- Sustained Cartesian twist driving the robot toward a singular configuration
- A ready pose that bends only shoulder/elbow joints but leaves wrist axes near alignment

Confirmed observations:

- Joint-space jogging works under the same conditions.
- Cartesian twisting can trigger singularity stops.
- This isolates the issue to the Cartesian Jacobian/singularity path rather than basic controller wiring.

Potential mitigations after validating a safer ready pose:

```yaml
lower_singularity_threshold: 8.0
hard_stop_singularity_threshold: 20.0
```

Do not lower singularity protections on real hardware without validating the resulting behavior and ensuring that the robot cannot enter unsafe configurations.

## Proposed `twist_stamper` Filtering

> This is a proposed enhancement, not a confirmed implementation. Rebuild and test `piper_servo` after applying it.

Add a dead zone and clamping before sending SpaceMouse data to Servo to suppress input noise:

```cpp
#include <algorithm>
#include <cmath>

auto clamp_deadzone = [](double value, double deadzone, double max_value)
{
  if (std::abs(value) < deadzone)
  {
    return 0.0;
  }

  return std::clamp(value, -max_value, max_value);
};
```

This implementation requires C++17 because it uses `std::clamp`. Set `CMAKE_CXX_STANDARD` to `17`, or replace `std::clamp` with an equivalent C++14-safe bound operation.

Suggested initial values:

```text
deadzone = 0.05
max_value = 1.0
```

Apply the filter consistently to all six twist components:

- `linear.x`
- `linear.y`
- `linear.z`
- `angular.x`
- `angular.y`
- `angular.z`

## Open Items

1. Validate the wrist-safer ready pose:

   ```text
   [0.0, 0.7, -1.0, 0.3, 0.8, 0.3]
   ```

2. Confirm the `twist_stamper` dead-zone and clamping patch is implemented, rebuilt, and tested.

3. Tune Servo singularity thresholds only after validating the workspace and ready pose.

4. Consider adding static/Coulomb friction to the URDF if higher-fidelity simulation is required; current joint friction is zero and only viscous damping is modeled.

5. Replace the ready-pose `ros2 topic pub --once` sequence with an action-based trajectory command that waits for successful completion before enabling Servo input.
