FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Instalar Gazebo, puente de ROS, herramientas de visión, dependencias del Tello y utilidades
RUN apt-get update && apt-get install -y \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-gz \
    ros-humble-vision-msgs \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-urdf-tutorial \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf-transformations \
    ros-humble-rclcpp-components \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-velocity-controllers \
    ros-humble-gz-ros2-control \
    ros-humble-gz-ros2-control-demos \
    ros-humble-ign-ros2-control \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-slam-toolbox \
    ros-humble-apriltag-ros \
    ros-humble-apriltag-msgs \
    ros-humble-imu-filter-madgwick \
    ros-humble-rqt-robot-steering \
    ros-humble-rviz-imu-plugin \
    python3-pip \
    python3-opencv \
    python3-matplotlib \
    python3-scipy \
    python3-transforms3d \
    libasio-dev \
    nano \
    tmux \
    git \
    && rm -rf /var/lib/apt/lists/*

# Compilar e instalar el SDK de YDLIDAR (requerido por ydlidar_ros2_driver)
RUN cd /tmp && git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cmake -S YDLidar-SDK -B YDLidar-SDK/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build YDLidar-SDK/build -j"$(nproc)" \
    && cmake --install YDLidar-SDK/build \
    && ldconfig \
    && rm -rf /tmp/YDLidar-SDK

# Instalar dependencias de Python para el Tello driver y transformaciones
RUN pip3 install "numpy==1.23.5" transformations catkin_pkg rospkg av image djitellopy2 pyyaml

# Configurar variables de entorno para NVIDIA
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,utility,compute

# Darle un color distinto al prompt para saber que estás en Docker
RUN echo "PS1='\[\033[01;36m\](docker) \[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["bash"]