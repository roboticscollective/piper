from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml, os, sys

demo_pkg_name = "piper_servo"


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    if not os.path.exists(absolute_file_path):
        print(f"[ERROR] File not found: {absolute_file_path}")
        sys.exit(1)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": load_yaml(demo_pkg_name, "config/piper_simulated_config.yaml")
    }

    # --- Step 1: Move to a known, non-singular pose via direct topic publish ---
    move_to_ready = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/arm_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{header: {frame_id: base_link}, "
            "joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], "
            "points: [{positions: [0.0, 1.7, -1.0, 0.0, 0.8, 0.0], "
            "time_from_start: {sec: 3}}]}",
        ],
        output="screen",
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_server",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare(demo_pkg_name), "rviz", "moveit.rviz"
        ])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": True},
        ],
    )

    twist_stamper_node = Node(
        package="piper_servo",
        executable="twist_stamper",
        name="twist_stamper",
        parameters=[
            {"frame_id": "base_link"},
            {"deadzone": 0.05},
            {"max_val": 1.0},
            {"smoothing": 0.2},
            {"coast_duration": 0.2},
            {"dominant_axis_lock": True},
            {"publish_period": 0.034},
            {"use_sim_time": True},
        ],
        remappings=[
            ("cmd_vel_in", "/spacenav/twist"),
            ("joy_in",     "/spacenav/joy"),
            ("twist_out",  "/servo_server/delta_twist_cmds"),
        ],
        output="screen",
    )

    start_after_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=move_to_ready,
            on_exit=[servo_node, rviz_node, twist_stamper_node],
        )
    )

    return LaunchDescription([
        move_to_ready,
        start_after_ready,
    ])
