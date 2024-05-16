# Image based Ubuntu 22.04, CUDA 11.8
# https://hub.docker.com/layers/nvidia/cuda/11.8.0-cudnn8-devel-ubuntu22.04/images/sha256-3fbac875b9fd3059d554226246c1ade676fad88aee1ac2cb2deaa6e77e4606a5?context=explore
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

MAINTAINER dongwookheo
# Setting environment variables
ENV DEBIAN_FRONTEND=noninteractive
ARG UBUNTU_RELEASE_YEAR=22
ARG CUDA_MAJOR=11
ARG CUDA_MINOR=8
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=0
ARG ZED_SDK_PATCH=7
ARG ROS_DISTRO=humble

# Related build...
RUN apt-get update && apt-get upgrade -y && \
    apt-get install build-essential -y && \
    apt-get install vim -y && \
    apt-get install cmake -y && \
    apt-get install git -y && \
    apt-get install sudo -y && \
    apt-get install wget -y && \
    apt-get install ninja-build -y && \
    apt-get install software-properties-common -y && \
    apt-get install python3 -y && \
    apt-get install python3-pip -y && \
    rm -rf /var/lib/apt/lists/*

# Related to JetBrains CLion Docker development...
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ssh && \
    apt-get install -y gcc && \
    apt-get install -y g++ && \
    apt-get install -y gdb && \
    apt-get install -y clang && \
    apt-get install -y cmake && \
    apt-get install -y rsync && \
    apt-get install -y tar && \
    apt-get install -y mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# Related to X11 remote display
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y libglu1-mesa-dev && \
    apt-get install -y mesa-common-dev && \
    apt-get install -y x11-utils && \
    apt-get install -y x11-apps && \
    apt-get install -y zip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN apt-get update && apt-get upgrade -y  && \
    apt-get install lsb-release -y && \
    apt-get install gnupg2 -y && \
    apt-get install curl -y
RUN apt-get install software-properties-common -y && \
    add-apt-repository universe && \
    apt-get update && apt-get upgrade -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get upgrade -y
RUN apt-get install ros-humble-desktop -y && \
    apt-get install ros-dev-tools -y && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install the ZED SDK
RUN echo "CUDA Version $CUDA_VERSION" > /usr/local/cuda/version.txt

# Setup the ZED SDK
RUN apt-get update || true && \
  apt-get install --no-install-recommends dialog bash-completion lsb-release wget less udev sudo  build-essential cmake zstd python3 python3-pip libpng-dev libgomp1 -y && \
  python3 -m pip install --upgrade pip; python3 -m pip install numpy opencv-python-headless && \
  wget -q -O ZED_SDK_Linux_Ubuntu.run https://download.stereolabs.com/zedsdk/$ZED_SDK_MAJOR.$ZED_SDK_MINOR/cu$CUDA_MAJOR$CUDA_MINOR/ubuntu$UBUNTU_RELEASE_YEAR && \
  chmod +x ZED_SDK_Linux_Ubuntu.run && \
  ./ZED_SDK_Linux_Ubuntu.run -- silent skip_tools skip_cuda && \
  ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
  rm ZED_SDK_Linux_Ubuntu.run && \
  rm -rf /var/lib/apt/lists/* && \
  apt-get autoremove && apt-get clean

# Install the ZED ROS2 Wrapper
WORKDIR /home/mint/mintcart_ws/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
WORKDIR /home/mint/mintcart_ws

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
                  apt-get update -y || true && \
                  rosdep install --from-paths src --ignore-src -r -y && \
                  colcon build --parallel-workers $(nproc) --symlink-install \
                  --event-handlers console_direct+ --base-paths src \
                  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
                  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
                  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' "

# Install package
WORKDIR /home/mint/mintcart_ws/src
RUN git clone -b humble https://github.com/ros-drivers/ros2_ouster_drivers.git && \
    git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git && \
    git clone https://github.com/CLOBOT-Co-Ltd/myahrs_ros2_driver.git

# Fix myahrs_ros2_driver.cpp
WORKDIR /home/mint/mintcart_ws/src/myahrs_ros2_driver/myahrs_ros2_driver/src
#RUN sed -i 's/declare_parameter("linear_acceleration_stddev")/declare_parameter("linear_acceleration_stddev", 0.0)/g' ./myahrs_ros2_driver.cpp && \
#    sed -i 's/declare_parameter("angular_velocity_stddev")/declare_parameter("angular_velocity_stddev", 0.0)/g' ./myahrs_ros2_driver.cpp && \
#    sed -i 's/declare_parameter("magnetic_field_stddev")/declare_parameter("magnetic_field_stddev", 0.0)/g' ./myahrs_ros2_driver.cpp && \
#    sed -i 's/declare_parameter("orientation_stddev")/declare_parameter("orientation_stddev", 0.0)/g' ./myahrs_ros2_driver.cpp
RUN sed -i 's/this->declare_parameter(\("[a-z_]*"\))/this->declare_parameter(\1, 0.0)/g' ./myahrs_ros2_driver.cpp

# Set working dir
WORKDIR /home/mint/mintcart_ws

# Environment settings with aliasing
RUN echo "\
# MINT-Lab settings\n\
alias uu=\"sudo apt update && sudo apt upgrade -y\"\n\
alias humble=\"source /opt/ros/humble/setup.bash; echo \\\"ROS 2 Humble is activated!\\\"\"\n\
alias sb=\"source /root/.bashrc; echo \\\"bashrc is reloaded!\\\"; humble; ros_domain; mintcart\"\n\
alias ros_domain=\"export ROS_DOMAIN_ID=220; echo \\\"ROS DOMAIN ID is set to \${ROS_DOMAIN_ID}!\\\"\"\n\
alias mintcart=\"source /home/mint/mintcart_ws/install/setup.bash; echo \\\"mintcart_ws is activated!\\\"\"\n\
\n\
# CUDA\n\
export PATH=\"/usr/local/cuda-11.8/bin:\$PATH\"\n\
export LD_LIBRARY_PATH=\"/usr/local/cuda-11.8/lib64:\$LD_LIBRARY_PATH\"\n" >> /root/.bashrc

# Install ROS Packages Dependencies
WORKDIR /home/mint/mintcart_ws
RUN apt-get update && apt-get upgrade -y && \
    /bin/bash -c "source /root/.bashrc && \
                  source /home/mint/mintcart_ws/install/setup.bash && \
                  rosdep update && \
                  rosdep install --from-path src --ignore-src -r -y && \
                  colcon build --symlink-install"

RUN apt-get update && apt-get upgrade -y && \
    apt-get install ros-$ROS_DISTRO-rviz-imu-plugin && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean
