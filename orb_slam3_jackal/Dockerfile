FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    libssl-dev \
    libglew-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libpython3-dev \
    python3-numpy \
    python3-pip \
    python3-matplotlib \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libgtk-3-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install Pangolin
WORKDIR /opt
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig

# Set library path
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Create workspace
RUN mkdir -p /root/clearpath_ws/src
WORKDIR /root/clearpath_ws/src

# Copy the package
COPY . /root/clearpath_ws/src/ros2_orb_slam3/

# Create symlink for config path (hardcoded in original code)
RUN mkdir -p /root/ros2_test/src && \
    ln -s /root/clearpath_ws/src/ros2_orb_slam3 /root/ros2_test/src/ros2_orb_slam3

# Build the workspace
WORKDIR /root/clearpath_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install"

# Setup entrypoint
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/clearpath_ws/install/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Add to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/clearpath_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc

WORKDIR /root/clearpath_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
