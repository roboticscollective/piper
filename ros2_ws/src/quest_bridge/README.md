# quest_bridge — VR Teleoperation for the Piper Arm

Meta Quest controller teleoperation for the Piper 6-DOF arm, via MoveIt Servo.
`quest_piper_teleop` reads the right-hand Quest controller pose through OpenVR,
converts it into Cartesian velocity commands, and publishes them to
`/servo_server/delta_twist_cmds`. Everything downstream of that topic (Servo,
`arm_controller`, the real-hardware bridge) is identical to the existing
SpaceMouse teleoperation path — VR is just another input device feeding the
same command topic.

## Where things run

- **`quest_piper_teleop` runs on the host**, not inside the Docker container.
  It needs `openvr` (SteamVR/ALVR) for the headset, which the container does
  not have installed. Build and run it against the host's ROS install.
- **Everything else** (Gazebo/mock hardware, MoveIt, Servo, the real-robot
  driver) runs **inside `piper_ros_container`**, same as always.
- VR and the container communicate over DDS using the same
  `ROS_DOMAIN_ID` (`27` in this workspace) — export it in every shell.

## Prerequisites (host)

- ROS 2 installed on the host (this workspace has been run against Kilted).
- `python3 -m pip install openvr scipy` (or equivalent) — not declared in
  `package.xml` since they're not resolvable via `rosdep`.
- SteamVR (or ALVR for standalone Quest streaming) running and tracking the
  right-hand controller before you start `quest_piper_teleop`.

## Build

Host:
```bash
cd ~/PIPER/piper/ros2_ws
source /opt/ros/kilted/setup.bash   # or your host ROS distro
colcon build --packages-select quest_bridge
source install/setup.bash
```

Container (only needed if you also want `quest_driver`/`vr_servo_bridge`
available there, or after editing `piper_servo`):
```bash
docker exec -it piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && cd /workspace && colcon build --packages-select piper_servo quest_bridge"
```

---

## Simulation (no real robot)

`piper_servo.launch.py` is self-contained for this: it brings up its own
`ros2_control_node` on `mock_components/GenericSystem` (a software-only
hardware stand-in — commands are echoed back as state, no physics), plus
`robot_state_publisher`, `joint_state_broadcaster`, `arm_controller`,
`gripper_controller`, `servo_server`, `twist_stamper`, and RViz.

> **Do not also launch `piper_gazebo.launch.py` alongside this.** Both bring
> up their own `robot_state_publisher` and controller manager; running them
> together causes duplicate-node/service conflicts. Gazebo-based physics
> simulation is not currently compatible with this launch file — use one or
> the other, not both.

**1. Servo stack (container):**
```bash
docker exec -it piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch piper_servo piper_servo.launch.py"
```

**2. Verify it landed on the safe ready pose and Servo is healthy** — the
ready-pose bootstrap is a fire-and-forget `topic pub`, so it can race and
leave the arm at the singular all-zero pose. Don't skip this check:
```bash
docker exec piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=27 && \
   ros2 topic echo /joint_states --once --field position && \
   ros2 topic echo /servo_server/status --once --qos-reliability reliable --qos-durability transient_local"
```
Expect joint positions near `[0.0, 1.7, -1.0, 0.0, 0.8, 0.0]` (order in the
echoed name array may differ) and `status: 0`.

**If it didn't land correctly** (near-zero joints, or status non-zero — e.g. `2` =
`HALT_FOR_SINGULARITY`, `5` = `HALT_FOR_COLLISION`), recover manually:
```bash
docker exec piper_ros_container bash -c "
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=27
ros2 service call /servo_server/pause_servo std_srvs/srv/Trigger {}
sleep 1
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \\
  '{header: {frame_id: base_link}, joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, 1.7, -1.0, 0.0, 0.8, 0.0], time_from_start: {sec: 3}}]}'
sleep 4
ros2 service call /servo_server/start_servo std_srvs/srv/Trigger {}
"
```
Then re-run the check above to confirm.

**3. Start VR teleop (host):**
```bash
source /opt/ros/kilted/setup.bash
source ~/PIPER/piper/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=27
ros2 run quest_bridge quest_piper_teleop
```

**4. Fly it:** pull the trigger to engage, move your hand, release to stop.
See [Controls](#controls) below.

---

## Real hardware

`quest_piper_teleop` needs no changes to talk to the real arm — it publishes
to the same `/servo_server/delta_twist_cmds` topic used above. Real hardware
control works through an **unintentional topic-name bridge**, not a designed
interface — understand this before running it:

`start_single_piper.launch.py` remaps the real driver's command input topic
(`joint_ctrl_single`) to `/joint_states`. `piper_servo.launch.py`'s mock
`joint_state_broadcaster` *publishes* the mock arm's state to that same name.
Run both together and the mock's commanded position gets picked up by the
real driver as a live joint command. This means **Servo has no real feedback
about the physical robot's actual position** — it only knows what it last
told the mock to do. There is no closed-loop safety check if the real arm
lags, stalls, or is blocked.

> ⚠️ **Before enabling the real arm:** clear its workspace, know how to cut
> power, and never call the enable service while VR is actively driving —
> the arm will snap to whatever the last commanded (possibly large, unsettled)
> position was. Enable only once the arm is confirmed parked at the ready
> pose and idle (see step 4 below).

**1. Bring up `can0` on the host** (before starting the container, if not
already up):
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -br link show can0   # should show state UP
```

**2. Real robot driver (container)** — `auto_enable:=false` so you control
exactly when the motors energize:
```bash
docker exec -it piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=true"
```

**3. Servo stack (container)** — same as the simulation path, no Gazebo:
```bash
docker exec -it piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch piper_servo piper_servo.launch.py"
```

**4. Verify the arm is parked at the ready pose and idle** — use the same
check (and recovery, if needed) commands as the simulation section above.
Confirm `status: 0` and that joint positions are **not changing** between two
consecutive checks (i.e. nothing is actively driving it) before proceeding.

**5. Clear the workspace, then enable the real arm:**
```bash
docker exec piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=27 && \
   ros2 service call /enable_srv piper_msgs/srv/Enable 'enable_request: true'"
```

**6. Confirm it's enabled:**
```bash
docker exec piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=27 && \
   timeout 3 ros2 topic echo /arm_status --once"
```

**7. Start VR teleop (host)** — identical command to the simulation section:
```bash
source /opt/ros/kilted/setup.bash
source ~/PIPER/piper/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=27
ros2 run quest_bridge quest_piper_teleop
```

**8. Test in FINE mode first** (hold grip), small deliberate movements. Given
there's no real feedback loop protecting the physical arm, don't switch to
COARSE mode until you've confirmed the real arm tracks commands smoothly with
no lag or stall.

**To disable/stop:**
```bash
docker exec piper_ros_container bash -c \
  "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=27 && \
   ros2 service call /enable_srv piper_msgs/srv/Enable 'enable_request: false'"
```

---

## Controls

| Input | Action |
|---|---|
| **Trigger** | Hold to engage teleop (clutch). Hand motion since the trigger was pressed drives the arm relative to its pose at that instant. Release to stop and hold position. |
| **A button** | Cycle control mode: `POSITION + ORIENTATION` → `POSITION ONLY` → `ORIENTATION ONLY` → back to `BOTH` |
| **Grip** | Hold for FINE control (`Kp_pos=1.0`, `Kp_rot=0.5`); release for COARSE (`Kp_pos=5.0`, `Kp_rot=2.0`) |
| **B (hold 1s)** | Send the arm to its home pose (Cartesian move — can pass through singularities on the way; use with caution, especially on real hardware) |

**Position** maps 1:1 from hand motion (forward/back → robot X, left/right →
robot Y, up/down → robot Z). **Orientation** is attenuated
(`ROT_SCALE = 0.4` in `quest_piper_teleop.py`) because the wrist joint
(`joint5`) has only ±70° of range — far less than a human wrist — so a full
wrist twist doesn't demand more rotation than the arm can deliver. Re-clutch
(release/re-press trigger) between large reorientations rather than trying to
do them in one continuous hold.

## Troubleshooting

- **Arm doesn't move at all:** check `/servo_server/status` — Servo starts
  paused by default; `quest_piper_teleop` calls `/servo_server/start_servo`
  itself on startup, but if `servo_server` wasn't up yet when the node
  started, it'll log a warning and never retry. Restart the node once Servo
  is confirmed running.
- **`close to a position limit` / `close to a singularity` warnings, then
  nothing responds:** the arm is sitting at or near a joint limit for the
  *current* configuration — Servo halts the entire twist command in every
  direction that would push further into it, not just the offending one. Ease
  off the trigger, re-clutch from a more neutral pose, and reposition the
  shoulder/elbow before continuing.
- **Launch file changes not taking effect:** `ros2 launch` runs the
  *installed* copy, not the source directly. Rebuild after any change:
  `colcon build --packages-select piper_servo` (or `quest_bridge`).
- **Real driver receiving nothing:** confirm both `start_single_piper.launch.py`
  and `piper_servo.launch.py` are running together — the real driver has no
  command source on its own; it depends on the mock `joint_state_broadcaster`
  bridge described above.
