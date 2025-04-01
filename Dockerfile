# Use a multi-architecture base image
ARG TARGETPLATFORM
FROM --platform=$TARGETPLATFORM ros:humble-ros-core-jammy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    python3-colcon-common-extensions \
    libgl1-mesa-glx \
    libglib2.0-0 \
    python3-tf2-ros \
    python3-sensor-msgs \
    python3-geometry-msgs \
    python3-std-msgs \
    build-essential \
    libeigen3-dev \
    libtbb-dev \
    pybind11-dev \
    ninja-build \
    g++ \
    git \
    wget \
    curl \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Upgrade CMake to a compatible version with architecture-specific handling
RUN ARCH=$(dpkg --print-architecture) && \
    if [ "$ARCH" = "amd64" ]; then \
        wget -qO- "https://github.com/Kitware/CMake/releases/download/v3.27.5/cmake-3.27.5-linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local; \
    elif [ "$ARCH" = "arm64" ]; then \
        wget -qO- "https://github.com/Kitware/CMake/releases/download/v3.27.5/cmake-3.27.5-linux-aarch64.tar.gz" | tar --strip-components=1 -xz -C /usr/local; \
    fi

# Install Python dependencies
RUN pip install --no-cache-dir transforms3d "numpy<2" open3d "scikit-build-core<0.10" packaging

# Clone and install kiss_icp with submodule initialization
WORKDIR /workspace
RUN git clone --recursive https://github.com/PRBonn/kiss-icp.git && \
    cd kiss-icp && \
    git submodule update --init --recursive && \
    # Check if the CMakeLists.txt exists in the 'cpp/kiss_icp' subdirectory
    if [ -f cpp/kiss_icp/CMakeLists.txt ]; then \
        echo "CMakeLists.txt found in cpp/kiss_icp, proceeding with build"; \
    else \
        echo "CMakeLists.txt not found in cpp/kiss_icp"; exit 1; \
    fi && \
    mkdir build && \
    cd build && \
    cmake ../cpp/kiss_icp -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    # List all available targets to check for the correct build target
    make help

# Install Xvfb, x11vnc, novnc, and websockify for virtual display and browser access
RUN apt-get update && apt-get install -y xvfb x11vnc novnc websockify && rm -rf /var/lib/apt/lists/*

# Expose VNC and noVNC ports
EXPOSE 5900
EXPOSE 6080

# Set up workspace
WORKDIR /workspace

RUN echo '#!/bin/bash\n\
Xvfb :0 -screen 0 1920x1080x24 &\n\
sleep 2  # Give Xvfb time to start\n\
export DISPLAY=:0\n\
x11vnc -display :0 -forever -nopw -listen 0.0.0.0 -xkb &\n\
websockify --web=/usr/share/novnc 6080 localhost:5900 &\n\
source /opt/ros/humble/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh


ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
