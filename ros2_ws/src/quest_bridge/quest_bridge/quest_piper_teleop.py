import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import openvr
import numpy as np
from geometry_msgs.msg import TwistStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Trigger
import sys

# OpenVR is Right-Hand Y-up; ROS is Right-Hand Z-up
VR_TO_ROS = np.array([
    [ 0,  0, -1,  0],
    [-1,  0,  0,  0],
    [ 0,  1,  0,  0],
    [ 0,  0,  0,  1],
], dtype=float)

# MoveIt Servo unitless input bounds
MAX_TWIST = 1.0

# Teleop control modes
MODE_BOTH        = 0
MODE_POSITION    = 1
MODE_ORIENTATION = 2
MODE_NAMES = {
    MODE_BOTH:        "POSITION + ORIENTATION",
    MODE_POSITION:    "POSITION ONLY",
    MODE_ORIENTATION: "ORIENTATION ONLY",
}

# OpenVR button masks
BUTTON_A    = 1 << openvr.k_EButton_A
BUTTON_B    = 1 << openvr.k_EButton_ApplicationMenu  # B button on Quest 3
BUTTON_GRIP = 1 << openvr.k_EButton_Grip

# -------------------------------------------------------------------
# Fine control (grip held) — slow, precise, stiff
# -------------------------------------------------------------------
FINE_KP_POS = 1.0
FINE_KP_ROT = 0.5

# -------------------------------------------------------------------
# Coarse control (grip released) — fast, compliant repositioning
# -------------------------------------------------------------------
COARSE_KP_POS = 5.0
COARSE_KP_ROT = 2.0

# -------------------------------------------------------------------
# Wrist joints (e.g. joint5) have far less range than a human wrist.
# Attenuate hand rotation so a full-range wrist twist doesn't demand
# more rotation than the robot can physically deliver.
# -------------------------------------------------------------------
ROT_SCALE = 0.4

# -------------------------------------------------------------------
# Home position — confirmed from controller state
# scipy from_quat expects [x, y, z, w] order
# -------------------------------------------------------------------
HOME_POSITION = np.array([-0.37157076372935005,
                            0.19487539864267492,
                            0.3276790677005921])
HOME_ORIENTATION = R.from_quat([
     5.537526767720718e-05,  # x
    -2.602330114019951e-06,  # y
     0.0001949197982194574,  # z
     0.9999999794665397,     # w — large value belongs here
])

# How long B button must be held to trigger home (seconds)
HOME_HOLD_DURATION = 1.0

class QuestPiperTeleop(Node):
    def __init__(self):
        super().__init__('quest_piper_teleop_node')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.publisher_ = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_servo_client = self.create_client(Trigger, '/servo_server/start_servo')
        self._start_servo()

        # Teleop state
        self.current_robot_pos   = None
        self.current_robot_rot   = None
        self.teleop_enabled      = False
        self.calibration_offset  = None
        self.calibration_hand_rot  = None
        self.calibration_robot_rot = None

        # Subscriber health tracking (TF instead)
        self.last_tf_time = None

        # Control mode — cycles with A button
        self.control_mode     = MODE_BOTH
        self.last_a_state     = False
        self.held_orientation = None
        self.held_position    = None

        # Fine/coarse control
        self.fine_control    = False
        self.last_grip_state = False

        # Home position state — B button hold
        self.b_hold_start    = None
        self.last_b_state    = False
        self.homing_active   = False

        try:
            self.vr = openvr.init(openvr.VRApplication_Background)
        except openvr.OpenVRError as e:
            self.get_logger().error(f"OpenVR Init Failed: {e}")
            sys.exit(1)

        self.get_logger().info("Waiting for first robot state...")
        self.get_logger().info(
            "Controls: TRIGGER = teleop | A = cycle mode | "
            "GRIP = fine control | B HOLD 1s = go to home")
        self.get_logger().info(
            f"Current mode: {MODE_NAMES[self.control_mode]}")

        self.timer          = self.create_timer(0.02, self.timer_callback)
        self.watchdog_timer = self.create_timer(5.0,  self.watchdog_callback)

    # ------------------------------------------------------------------
    # Servo activation
    # ------------------------------------------------------------------

    def _start_servo(self):
        """MoveIt Servo starts paused; explicitly enable it so twist commands
        are not silently dropped."""
        if not self.start_servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "/servo_server/start_servo not available — is servo_server "
                "running? Twist commands will be ignored until Servo is "
                "started (call the service manually or restart this node "
                "once Servo is up).")
            return
        future = self.start_servo_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_start_servo_response)

    def _on_start_servo_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"start_servo call failed: {e}")
            return
        if result.success:
            self.get_logger().info(f"Servo started: {result.message}")
        else:
            self.get_logger().warn(f"Servo start_servo returned failure: {result.message}")

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'gripper_base', rclpy.time.Time())
            pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
            rot = R.from_quat([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])
            return pos, rot
        except TransformException as e:
            return None, None

    def watchdog_callback(self):
        # We can just use this to check if TF is available
        pos, rot = self.get_robot_pose()
        if pos is None:
            self.get_logger().warn(
                "TF base_link -> gripper_base not available. Is the simulation running?",
                throttle_duration_sec=2.0
            )
            self.teleop_enabled = False

    # ------------------------------------------------------------------
    # Control mode cycling (A button)
    # ------------------------------------------------------------------

    def cycle_control_mode(self):
        """Cycle: BOTH → POSITION ONLY → ORIENTATION ONLY → BOTH"""
        self.control_mode = (self.control_mode + 1) % 3
        self.get_logger().info(
            f"Control mode → {MODE_NAMES[self.control_mode]}"
        )

        if self.current_robot_pos is not None:
            self.held_orientation = self.current_robot_rot
            self.held_position = self.current_robot_pos

        if self.teleop_enabled:
            self.get_logger().info("Re-calibrating after mode switch...")
            self.teleop_enabled        = False
            self.calibration_offset    = None
            self.calibration_hand_rot  = None
            self.calibration_robot_rot = None

    # ------------------------------------------------------------------
    # Home position
    # ------------------------------------------------------------------

    def trigger_home(self):
        self.get_logger().info("HOMING — moving to home position...")
        self.teleop_enabled        = False
        self.calibration_offset    = None
        self.calibration_hand_rot  = None
        self.calibration_robot_rot = None
        self.homing_active         = True
        self._home_check_fired  = False
        self.create_timer(5.0, self._check_home_reached)

    def _check_home_reached(self):
        if self._home_check_fired:
            return
        self._home_check_fired = True

        if self.current_robot_pos is None:
            self.homing_active = False
            return

        dist = np.linalg.norm(self.current_robot_pos - HOME_POSITION)

        if dist < 0.05:
            self.get_logger().info(
                f"Home reached — dist error: {dist*1000:.1f}mm. "
                "Press TRIGGER to resume teleop.")
        else:
            self.get_logger().warn(
                f"Homing complete but dist error: {dist*1000:.1f}mm. "
                "Press TRIGGER to resume teleop.")

        self.homing_active  = False

    # ------------------------------------------------------------------
    # VR helpers
    # ------------------------------------------------------------------

    def vr_matrix_to_ros(self, matrix):
        m = np.eye(4)
        for r in range(3):
            for c in range(4):
                m[r, c] = matrix[r][c]
        return VR_TO_ROS @ m

    # We rely on Servo's collision checking and velocity limits, clamping not strictly needed manually

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def timer_callback(self):
        pos, rot = self.get_robot_pose()
        if pos is None:
            return
        self.current_robot_pos = pos
        self.current_robot_rot = rot

        poses_type = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
        poses      = (poses_type)()
        self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseRawAndUncalibrated, 0, poses)

        # --- Controller pose diagnostic ---
        controller_found = False
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if self.vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_Controller:
                continue
            role = self.vr.getControllerRoleForTrackedDeviceIndex(i)
            if role != openvr.TrackedControllerRole_RightHand:
                continue

            if poses[i].bPoseIsValid:
                controller_found = True
                ros_matrix = self.vr_matrix_to_ros(poses[i].mDeviceToAbsoluteTracking)
                pos  = ros_matrix[:3, 3]
                quat = R.from_matrix(ros_matrix[:3, :3]).as_quat()
                self.get_logger().info(
                    f"Controller | "
                    f"pos: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} | "
                    f"ori: x={quat[0]:.3f} y={quat[1]:.3f} z={quat[2]:.3f} w={quat[3]:.3f} | "
                    f"mode: {MODE_NAMES[self.control_mode]} | "
                    f"{'[FINE]' if self.fine_control else '[COARSE]'} | "
                    f"{'[HOMING]' if self.homing_active else ''}",
                    throttle_duration_sec=0.5
                )
            else:
                controller_found = True
                self.get_logger().warn(
                    "Right Hand controller found but pose is NOT valid (tracking lost?)",
                    throttle_duration_sec=1.0
                )

        if not controller_found:
            self.get_logger().info(
                "Waiting for controller pose — no Right Hand controller detected.",
                throttle_duration_sec=1.0
            )
        # --- End diagnostic ---

        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if self.vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_Controller:
                continue
            role = self.vr.getControllerRoleForTrackedDeviceIndex(i)
            if role != openvr.TrackedControllerRole_RightHand or not poses[i].bPoseIsValid:
                continue

            _, state     = self.vr.getControllerState(i)
            is_triggered = state.rAxis[1].x > 0.5

            # --- A button: cycle control mode (rising edge) ---
            a_pressed = bool(state.ulButtonPressed & BUTTON_A)
            if a_pressed and not self.last_a_state:
                self.cycle_control_mode()
            self.last_a_state = a_pressed

            # --- Grip button: fine/coarse control ---
            grip_pressed = bool(state.ulButtonPressed & BUTTON_GRIP)
            if grip_pressed and not self.last_grip_state:
                self.fine_control = True
                self.get_logger().info("FINE control active")
            elif not grip_pressed and self.last_grip_state:
                self.fine_control = False
                self.get_logger().info("COARSE control active")
            self.last_grip_state = grip_pressed

            # --- B button: hold 1 second to home ---
            b_pressed = bool(state.ulButtonPressed & BUTTON_B)
            now_sec   = self.get_clock().now().nanoseconds / 1e9

            if b_pressed:
                if not self.last_b_state:
                    # Rising edge — start hold timer
                    self.b_hold_start = now_sec
                    self.get_logger().info(
                        "B held — hold for 1s to go to home...")
                else:
                    # Still held — check duration
                    if self.b_hold_start is not None:
                        held_duration = now_sec - self.b_hold_start
                        remaining     = HOME_HOLD_DURATION - held_duration

                        if held_duration >= HOME_HOLD_DURATION and not self.homing_active:
                            self.trigger_home()
                            self.b_hold_start = None
                        else:
                            self.get_logger().info(
                                f"Hold for {remaining:.1f}s more to home...",
                                throttle_duration_sec=0.2
                            )
            else:
                if self.last_b_state:
                    self.get_logger().info("B released — homing cancelled.")
                    self.b_hold_start = None

            self.last_b_state = b_pressed
            # --- End B button ---

            if self.homing_active:
                self.publish_command(is_homing=True)
                continue

            ros_matrix   = self.vr_matrix_to_ros(poses[i].mDeviceToAbsoluteTracking)
            hand_pos_ros = ros_matrix[:3, 3]
            hand_rot_ros = R.from_matrix(ros_matrix[:3, :3])

            robot_pos = self.current_robot_pos
            robot_rot = self.current_robot_rot

            if is_triggered:
                if not self.teleop_enabled:
                    self.get_logger().info(
                        f"Trigger pressed — starting teleop... "
                        f"Control mode: {MODE_NAMES[self.control_mode]}"
                    )
                    self.calibration_offset    = robot_pos - hand_pos_ros
                    self.calibration_hand_rot  = hand_rot_ros
                    self.calibration_robot_rot = robot_rot
                    self.teleop_enabled        = True

                    self.get_logger().info(
                        f"TELEOP ACTIVE | mode: {MODE_NAMES[self.control_mode]} | "
                        f"Pos offset: ({self.calibration_offset[0]:.3f}, "
                        f"{self.calibration_offset[1]:.3f}, "
                        f"{self.calibration_offset[2]:.3f})"
                    )

                self.publish_command(ros_matrix)
                self.was_triggered = True

            else:
                if self.teleop_enabled:
                    self.get_logger().info("Trigger Released: Teleop Stopped.")
                self.teleop_enabled        = False
                self.calibration_offset    = None
                self.calibration_hand_rot  = None
                self.calibration_robot_rot = None

                if getattr(self, 'was_triggered', False):
                    self.publish_zero_twist()
                    self.was_triggered = False

    # ------------------------------------------------------------------
    # Command publisher
    # ------------------------------------------------------------------

    def publish_zero_twist(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        self.publisher_.publish(twist)

    def publish_command(self, ros_matrix=None, is_homing=False):
        if not is_homing and ros_matrix is None:
            return

        if not is_homing:
            hand_pos = ros_matrix[:3, 3]
            hand_rot = R.from_matrix(ros_matrix[:3, :3])

            target_pos = hand_pos + self.calibration_offset

            # Attenuate hand rotation about the calibration pose so a full
            # wrist twist doesn't demand more rotation than the robot's
            # limited wrist joints can deliver.
            hand_delta   = self.calibration_hand_rot.inv() * hand_rot
            scaled_delta = R.from_rotvec(hand_delta.as_rotvec() * ROT_SCALE)
            target_rot   = self.calibration_robot_rot * scaled_delta
        else:
            target_pos = HOME_POSITION
            target_rot = HOME_ORIENTATION

        robot_pos = self.current_robot_pos
        robot_rot = self.current_robot_rot

        pos_err = target_pos - robot_pos
        rot_err = target_rot * robot_rot.inv()
        rot_vec = rot_err.as_rotvec()

        Kp_pos = FINE_KP_POS if self.fine_control else COARSE_KP_POS
        Kp_rot = FINE_KP_ROT if self.fine_control else COARSE_KP_ROT

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        if self.control_mode == MODE_BOTH or self.control_mode == MODE_POSITION:
            twist.twist.linear.x = float(np.clip(pos_err[0] * Kp_pos, -MAX_TWIST, MAX_TWIST))
            twist.twist.linear.y = float(np.clip(pos_err[1] * Kp_pos, -MAX_TWIST, MAX_TWIST))
            twist.twist.linear.z = float(np.clip(pos_err[2] * Kp_pos, -MAX_TWIST, MAX_TWIST))
        
        if self.control_mode == MODE_BOTH or self.control_mode == MODE_ORIENTATION:
            twist.twist.angular.x = float(np.clip(rot_vec[0] * Kp_rot, -MAX_TWIST, MAX_TWIST))
            twist.twist.angular.y = float(np.clip(rot_vec[1] * Kp_rot, -MAX_TWIST, MAX_TWIST))
            twist.twist.angular.z = float(np.clip(rot_vec[2] * Kp_rot, -MAX_TWIST, MAX_TWIST))

        self.publisher_.publish(twist)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = QuestPiperTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        openvr.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
