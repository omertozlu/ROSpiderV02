FROM osrf/ros:humble-desktop-full

# 1. Install dev tools, graphics utils, and dos2unix
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    nano \
    dos2unix \
    mesa-utils \
    ros-humble-xacro \
    ros-humble-ros-gz \
    ros-humble-joint-state-broadcaster \
    ros-humble-forward-command-controller \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control\
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

# 2. Setup workspace
WORKDIR /root/colcon_ws

# 3. Setup Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh

# --- NEW: Add this line to auto-source ROS in every new terminal ---
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# 4. Environment
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1

# Golem için Alias'ı tüm kullanıcılar için kalıcı yap
RUN echo "alias golem_run='/root/colcon_ws/src/hexapod_controller/scripts/start_golem.sh'" >> /etc/bash.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]