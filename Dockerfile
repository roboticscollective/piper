# Dockerfile for Piper + Hand-Eye Calibration on ROS 2 Humble (Ubuntu 22.04)
# Camera: Percipio GM465 E1 (camport_ros2 / TYCam SDK bundled in workspace)
# Calibration: aruco_ros + handeye_calibration_ros
FROM ros:humble

# 1. Piper arm & MoveIt2 dependencies
RUN apt-get update && \
    apt-get install -y \
    ros-humble-ros-base \
    ros-humble-moveit \
    ros-humble-warehouse-ros \
    ros-humble-rqt* \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-* \
    ros-humble-gripper-controllers \
    ros-humble-trajectory-msgs \
    ros-humble-controller-manager \
    # ArUco marker detection (aruco_ros)
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-eigen \
    ros-humble-tf2-sensor-msgs \
    ros-humble-visualization-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    # Percipio GM465 E1 camera (camport_ros2)
    ros-humble-image-publisher \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-pcl-conversions \
    ros-humble-rclcpp-lifecycle \
    ros-humble-lifecycle-msgs \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-moveit-servo \
    # Hand-eye calibration (handeye_calibration_ros)
    ros-humble-tf-transformations \
    # System libraries
    libopencv-dev \
    python3-opencv \
    libeigen3-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-scipy \
    ethtool \
    can-utils \
    iproute2 && \
    pip3 install piper_sdk python-can && \
    rm -rf /var/lib/apt/lists/*


# Copy and set up the entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Source ROS + workspace (built at /workspace via mounted volume)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc
