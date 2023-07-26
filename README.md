# Mint Cart for ROS2

**mint_cart_ros** is a package for **MINT-CART** of [MINT LAB](https://mint-lab.github.io/) to get dataset and test sensor fusion.

## Work condition
- ROS2 humble & Ubuntu 22.04


## Using sensors
  - LiDAR : [Ouster OS1-128](https://www.dataspeedinc.com/app/uploads/2019/10/Ouster-OS1-Datasheet.pdf) ([ros2_ouster_drivers](https://github.com/ros-drivers/ros2_ouster_drivers))
    - FOV : 45°
    - Resolution : 1.2 cm
    - Rotation Rate : 10 to 20 Hz
  - Stereo Camera : [ZED 2i](https://www.stereolabs.com/zed-2i) ([zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper))
    - Resolution & FPS: WVGA to 2.2K & 15 to 100 Hz
    - Depth : 0.2 to 20m, Up to 100Hz
    - Built-in sensors : Accelerometer, Gyroscope, Barometer, Magnetometer, Temperature sensor
  - IMU : [myAHRS+](http://withrobot.com/en/sensor/myahrsplus/) ([myahrs_ros2_driver](https://github.com/CLOBOT-Co-Ltd/myahrs_ros2_driver))
    - 9-DOF
      - 3-Axis Gyroscope : ± 2000 dps
      - 3-Axis Accelerometer : ± 16 g
      - 3-Axis Geomagnetic Sensor : ± 1200 μT
    - Up to 100Hz
  - GPS : [Ascen-GPS-620](https://ascenkorea.net/?page_id=690) ([nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2))
    - Frequeny : 1 to 10 Hz
    - Accuracy
      - Autonomous < 2.5m
      - SBAS < 2.0m
    - TTFF
      - Hot start < 1s
      - Warm start < 28s
      - Cold start < 29s
  - GPS-RTK : [SparkFun RTK Express Plus](https://www.sparkfun.com/products/18590) ([Ublox_ros](https://github.com/KumarRobotics/ublox))
    - GNSS Receiver : ZED-F9R
    - Built-in IMU (3-Axis Accel, 3-Axis Gyro)
    - Frequency : Up to 4Hz
    - Accuracy
      - With IMU < 30cm
      - With RTK & IMU < 1.4cm
    - TTFF
      - Hot start < 2s
      - Cold start < 25s

## Installation

### Prerequisite
- **ROS2** : Follow the instruction below to install ROS2 on PC.
  - [ROS2 Debian packages(humble)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- **ZED SDK** : 

### Creating a work space
Create work space
- ```
    mkdir -p mint_ws/src && cd mint_ws/src
   ```
Download or git clone below packages
- [ros2_ouster_drivers](https://github.com/ros-drivers/ros2_ouster_drivers/tree/humble) ( OS-1 LiDAR )
- [ublox_ros](https://github.com/KumarRobotics/ublox/tree/ros2) ( RTK_Express_Plus )
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) ( ZED 2i )
- [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2) ( Asen GPS )
- [myahrs_ros2_driver](https://github.com/CLOBOT-Co-Ltd/myahrs_ros2_driver) ( myAHRS+ )
- [rtcm_msgs](https://github.com/tilk/rtcm_msgs)
- [ntrip_client](https://github.com/LORD-MicroStrain/ntrip_client/tree/ros2)
- [fix2nmea](https://github.com/olvdhrm/RTK_GPS_NTRIP/tree/main/fix2nmea)

Install packages using below commands.
```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```
