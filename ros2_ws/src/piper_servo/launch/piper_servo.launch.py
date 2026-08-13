from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml, os, sys

demo_pkg_name = "piper_servo"
moveit_pkg_name = "piper_with_gripper_moveit"


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    if not os.path.exists(absolute_file_path):
        print(f"[ERROR] File not found: {absolute_file_path}")
        sys.exit(1)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():

    initial_positions_file = os.path.join(
        get_package_share_directory(moveit_pkg_name),
        "config/initial_positions.yaml",
    )

    # The URDF xacro includes piper.ros2_control.xacro (mock_components/GenericSystem),
    # so the same robot_description feeds both ros2_control_node and the servo node.
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name=moveit_pkg_name)
        .robot_description(mappings={"initial_positions_file": initial_positions_file})
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": load_yaml(demo_pkg_name, "config/piper_simulated_config.yaml")
    }

    ros2_controllers_path = os.path.join(
        get_package_share_directory(moveit_pkg_name),
        "config/ros2_controllers.yaml",
    )

    # ------------------------------------------------------------------
    # 1) controller_manager (ros2_control_node) + robot_state_publisher
    # ------------------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="log",
    )

    # ------------------------------------------------------------------
    # 2) Spawners. A spawner process EXITS once its controller is loaded
    #    and activated, which gives us a reliable "controller is up" event.
    # ------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # 3) Move to a known, non-singular pose.
    #    Triggered only AFTER arm_controller_spawner exits (= controller
    #    active + subscribing to /arm_controller/joint_trajectory), plus a
    #    short settle delay via TimerAction.
    # ------------------------------------------------------------------
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

    delayed_move_to_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                TimerAction(period=1.0, actions=[move_to_ready])
            ],
        )
    )

    # ------------------------------------------------------------------
    # 4) Servo + RViz + twist stamper, started once the arm reached (or at
    #    least accepted) the ready trajectory.
    #    NOTE: no use_sim_time here -- mock_components publishes no /clock,
    #    and a frozen clock silently stops Servo's timers.
    # ------------------------------------------------------------------
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_server",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
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
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        delayed_move_to_ready,
        start_after_ready,
    ])
