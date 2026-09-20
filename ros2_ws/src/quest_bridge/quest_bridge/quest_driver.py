import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import openvr
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from scipy.spatial.transform import Rotation as R
import sys

VR_TO_ROS = np.array([
    [ 0,  0, -1,  0],
    [-1,  0,  0,  0],
    [ 0,  1,  0,  0],
    [ 0,  0,  0,  1],
], dtype=float)

BUTTON_A    = 1 << openvr.k_EButton_A
BUTTON_B    = 1 << openvr.k_EButton_ApplicationMenu
BUTTON_GRIP = 1 << openvr.k_EButton_Grip

class QuestDriver(Node):
    def __init__(self):
        super().__init__('quest_driver_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, '/quest/pose', qos)
        self.joy_pub  = self.create_publisher(Joy, '/quest/joy', qos)

        try:
            self.vr = openvr.init(openvr.VRApplication_Background)
        except openvr.OpenVRError as e:
            self.get_logger().error(f"OpenVR Init Failed: {e}")
            sys.exit(1)

        self.get_logger().info("Quest Driver initialized. Publishing to /quest/pose and /quest/joy")
        self.timer = self.create_timer(0.02, self.timer_callback)

    def vr_matrix_to_ros(self, matrix):
        m = np.eye(4)
        for r in range(3):
            for c in range(4):
                m[r, c] = matrix[r][c]
        return VR_TO_ROS @ m

    def timer_callback(self):
        poses_type = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
        poses      = (poses_type)()
        self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseRawAndUncalibrated, 0, poses)

        # Find right hand controller
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if self.vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_Controller:
                continue
            role = self.vr.getControllerRoleForTrackedDeviceIndex(i)
            if role != openvr.TrackedControllerRole_RightHand or not poses[i].bPoseIsValid:
                continue

            # Publish Pose
            ros_matrix = self.vr_matrix_to_ros(poses[i].mDeviceToAbsoluteTracking)
            pos  = ros_matrix[:3, 3]
            quat = R.from_matrix(ros_matrix[:3, :3]).as_quat()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "vr_world"
            pose_msg.pose.position.x = float(pos[0])
            pose_msg.pose.position.y = float(pos[1])
            pose_msg.pose.position.z = float(pos[2])
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            self.pose_pub.publish(pose_msg)

            # Publish Joy
            _, state = self.vr.getControllerState(i)
            joy_msg = Joy()
            joy_msg.header.stamp = pose_msg.header.stamp
            joy_msg.header.frame_id = "vr_world"
            
            trigger_val = state.rAxis[1].x
            is_triggered = trigger_val > 0.5
            a_pressed = bool(state.ulButtonPressed & BUTTON_A)
            b_pressed = bool(state.ulButtonPressed & BUTTON_B)
            grip_pressed = bool(state.ulButtonPressed & BUTTON_GRIP)

            joy_msg.axes = [float(trigger_val)]
            # Map buttons to a standardized array format
            joy_msg.buttons = [
                int(a_pressed),
                int(b_pressed),
                int(grip_pressed),
                int(is_triggered)
            ]
            self.joy_pub.publish(joy_msg)
            break

def main(args=None):
    rclpy.init(args=args)
    node = QuestDriver()
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
