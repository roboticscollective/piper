import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Joy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation as R

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

# Fine control (grip held)
FINE_KP_POS = 1.0
FINE_KP_ROT = 0.5

# Coarse control (grip released)
COARSE_KP_POS = 5.0
COARSE_KP_ROT = 2.0

# Home position (specific to Piper)
HOME_POSITION = np.array([-0.371, 0.194, 0.327])
HOME_ORIENTATION = R.from_quat([0.0, 0.0, 0.0, 1.0])
HOME_HOLD_DURATION = 1.0

class VRServoBridge(Node):
    def __init__(self):
        super().__init__('vr_servo_bridge_node')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', qos)

        self.pose_sub = self.create_subscription(PoseStamped, '/quest/pose', self.pose_callback, qos)
        self.joy_sub  = self.create_subscription(Joy, '/quest/joy', self.joy_callback, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # VR state
        self.latest_vr_pos = None
        self.latest_vr_rot = None

        # Teleop state
        self.current_robot_pos   = None
        self.current_robot_rot   = None
        self.teleop_enabled      = False
        self.calibration_offset  = None
        self.calibration_rot     = None

        # Control mode
        self.control_mode     = MODE_BOTH
        self.last_a_state     = False

        # Fine/coarse control
        self.fine_control    = False
        self.last_grip_state = False

        # Home position state
        self.b_hold_start    = None
        self.last_b_state    = False
        self.homing_active   = False
        self._home_check_fired = False

        # Trigger state
        self.was_triggered = False

        self.get_logger().info("VR Servo Bridge initialized. Waiting for VR inputs and TF...")
        self.timer = self.create_timer(0.02, self.control_loop)

    def pose_callback(self, msg):
        self.latest_vr_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.latest_vr_rot = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

    def joy_callback(self, msg):
        a_pressed = bool(msg.buttons[0])
        b_pressed = bool(msg.buttons[1])
        grip_pressed = bool(msg.buttons[2])
        is_triggered = bool(msg.buttons[3])

        # --- A button: cycle control mode ---
        if a_pressed and not self.last_a_state:
            self.control_mode = (self.control_mode + 1) % 3
            self.get_logger().info(f"Control mode -> {MODE_NAMES[self.control_mode]}")
            if self.teleop_enabled:
                self.teleop_enabled = False
                self.calibration_offset = None
                self.calibration_rot = None
        self.last_a_state = a_pressed

        # --- Grip button: fine/coarse control ---
        if grip_pressed and not self.last_grip_state:
            self.fine_control = True
            self.get_logger().info("FINE control active")
        elif not grip_pressed and self.last_grip_state:
            self.fine_control = False
            self.get_logger().info("COARSE control active")
        self.last_grip_state = grip_pressed

        # --- B button: hold 1 second to home ---
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if b_pressed:
            if not self.last_b_state:
                self.b_hold_start = now_sec
                self.get_logger().info("B held — hold for 1s to go to home...")
            else:
                held_duration = now_sec - self.b_hold_start
                if held_duration >= HOME_HOLD_DURATION and not self.homing_active:
                    self.trigger_home()
                    self.b_hold_start = None
        else:
            if self.last_b_state:
                self.get_logger().info("B released — homing cancelled.")
                self.b_hold_start = None
        self.last_b_state = b_pressed

        # --- Trigger: Enable/Disable Teleop ---
        if not self.homing_active:
            if is_triggered:
                if not self.teleop_enabled and self.current_robot_pos is not None and self.latest_vr_pos is not None:
                    self.get_logger().info(f"Trigger pressed — starting teleop... ({MODE_NAMES[self.control_mode]})")
                    self.calibration_offset = self.current_robot_pos - self.latest_vr_pos
                    self.calibration_rot    = self.current_robot_rot * self.latest_vr_rot.inv()
                    self.teleop_enabled     = True
                self.was_triggered = True
            else:
                if self.teleop_enabled:
                    self.get_logger().info("Trigger Released: Teleop Stopped.")
                self.teleop_enabled     = False
                self.calibration_offset = None
                self.calibration_rot    = None
                if getattr(self, 'was_triggered', False):
                    self.publish_zero_twist()
                    self.was_triggered = False

    def trigger_home(self):
        self.get_logger().info("HOMING — moving to home position...")
        self.teleop_enabled     = False
        self.calibration_offset = None
        self.calibration_rot    = None
        self.homing_active      = True
        self._home_check_fired  = False
        self.create_timer(5.0, self._check_home_reached)

    def _check_home_reached(self):
        if self._home_check_fired: return
        self._home_check_fired = True
        self.homing_active = False

        if self.current_robot_pos is None: return
        dist = np.linalg.norm(self.current_robot_pos - HOME_POSITION)
        if dist < 0.05:
            self.get_logger().info(f"Home reached — dist error: {dist*1000:.1f}mm.")
        else:
            self.get_logger().warn(f"Homing complete but dist error: {dist*1000:.1f}mm.")

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'gripper_base', rclpy.time.Time())
            pos = np.array([
                trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
            ])
            rot = R.from_quat([
                trans.transform.rotation.x, trans.transform.rotation.y,
                trans.transform.rotation.z, trans.transform.rotation.w
            ])
            return pos, rot
        except TransformException:
            return None, None

    def publish_zero_twist(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        self.twist_pub.publish(twist)

    def control_loop(self):
        pos, rot = self.get_robot_pose()
        if pos is None:
            return
        self.current_robot_pos = pos
        self.current_robot_rot = rot

        if self.homing_active:
            self.publish_twist_towards(HOME_POSITION, HOME_ORIENTATION)
        elif self.teleop_enabled and self.latest_vr_pos is not None:
            target_pos = self.latest_vr_pos + self.calibration_offset
            target_rot = self.calibration_rot * self.latest_vr_rot
            self.publish_twist_towards(target_pos, target_rot)

    def publish_twist_towards(self, target_pos, target_rot):
        pos_err = target_pos - self.current_robot_pos
        rot_err = target_rot * self.current_robot_rot.inv()
        rot_vec = rot_err.as_rotvec()

        Kp_pos = FINE_KP_POS if self.fine_control else COARSE_KP_POS
        Kp_rot = FINE_KP_ROT if self.fine_control else COARSE_KP_ROT

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        if self.control_mode in (MODE_BOTH, MODE_POSITION):
            twist.twist.linear.x = float(np.clip(pos_err[0] * Kp_pos, -MAX_TWIST, MAX_TWIST))
            twist.twist.linear.y = float(np.clip(pos_err[1] * Kp_pos, -MAX_TWIST, MAX_TWIST))
            twist.twist.linear.z = float(np.clip(pos_err[2] * Kp_pos, -MAX_TWIST, MAX_TWIST))
        
        if self.control_mode in (MODE_BOTH, MODE_ORIENTATION):
            twist.twist.angular.x = float(np.clip(rot_vec[0] * Kp_rot, -MAX_TWIST, MAX_TWIST))
            twist.twist.angular.y = float(np.clip(rot_vec[1] * Kp_rot, -MAX_TWIST, MAX_TWIST))
            twist.twist.angular.z = float(np.clip(rot_vec[2] * Kp_rot, -MAX_TWIST, MAX_TWIST))

        self.twist_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VRServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
