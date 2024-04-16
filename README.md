# MINT Cart for ROS2

**mint_cart_ros** is a package for **MINT-CART** of [MINT LAB](https://mint-lab.github.io/) to get dataset and test sensor fusion.

## Work condition
- ROS2 humble & Ubuntu 22.04
- ASUS ROG Flow X13
  - CPU : [AMD Ryzen™ 9 5900HS](https://www.amd.com/en/products/apu/amd-ryzen-9-5900hs)
  - GPU : [NVIDIA GeForce RTX 3050 Ti Mobile](https://www.notebookcheck.net/NVIDIA-GeForce-RTX-3050-Ti-Laptop-GPU-Benchmarks-and-Specs.527430.0.html)


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
  - GPS-RTK : [SparkFun RTK Express Plus](https://www.sparkfun.com/products/18590) ([nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2))
    - GNSS Receiver : ZED-F9R
    - Built-in IMU (3-Axis Accel, 3-Axis Gyro)
    - Frequency : Up to 20Hz(RTK)
    - Accuracy
      - With IMU < 30cm
      - With RTK & IMU < 1.4cm
    - TTFF
      - Hot start < 2s
      - Cold start < 25s
  
  - ### [Sensors outline](https://cad.onshape.com/documents/e604f5206b6b069382c1478e/w/2c3c9b12e499277badf01ed1/e/66ac3166ca2dbf7da4c255b7)
    ![sensors outline](images/Sensors_Outline.png)
## Installation

### Prerequisite
- **[Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal)** or **[Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)**
- **ROS2** : Follow the instruction below to install ROS2 on PC.
  - [ROS2 Debian packages(humble)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- **[ZED SDK](https://www.stereolabs.com/developers/release/)**
  - Install [CUDA](https://developer.nvidia.com/cuda-11-8-0-download-archive) (Recommend 11.8) & [cuDNN](https://developer.nvidia.com/rdp/cudnn-archive)
  - Following [Getting started](https://github.com/stereolabs/zed-sdk#getting-started)

### Creating a work space
1. Create work space
   ```
   mkdir -p mint_ws/src
   ```

2. Follow below instruction to install **[zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) ( ZED 2i )**
    ```
    cd ~/mint_ws/src/ #use your current ros2 workspace folder
    git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
    echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
    source ~/.bashrc
   ```
3. Download or git clone below packages
   - [ros2_ouster_drivers](https://github.com/ros-drivers/ros2_ouster_drivers/tree/humble) ( OS-1 LiDAR )
   - [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2) ( Asen GPS )
   - [myahrs_ros2_driver](https://github.com/CLOBOT-Co-Ltd/myahrs_ros2_driver) ( myAHRS+ )  

4. Install packages using below commands.
    ```
    cd mint_ws
    colcon build --symlink-install
    source ~/.bashrc
    ```

    **Note**: If you using ros2 humble(22.04), you might got this error when build **myahrs_ros2**
    ![my_ahrs_error](images/myahrs_error.png)

    This is due to this package is made for **foxy** and **declare_parameter** don't have default value in humble([foxy](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a095ea977b26e7464d9371efea5f36c42), [humble](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a095ea977b26e7464d9371efea5f36c42)). So, you have to insert default value.  
    To insert, open mint_ws/src/myahrs_ros2_driver-master/myahrs_ros2_driver/src/myahrs_ros2_driver.cpp  

    Next, insert default value end of declare_parameter function
    ```c++
    namespace WithRobot
    {
    MyAhrsDriverForROS::MyAhrsDriverForROS(std::string port, int baud_rate)
    : iMyAhrsPlus(port, baud_rate), Node("myahrs_ros2")
    {
      // dependent on user device
      publish_tf_ = false;
      frame_id_ = "imu_link";
      parent_frame_id_ = "base_link";

      this->declare_parameter("linear_acceleration_stddev", 0.0);
      this->declare_parameter("angular_velocity_stddev", 0.0);
      this->declare_parameter("magnetic_field_stddev", 0.0);
      this->declare_parameter("orientation_stddev", 0.0);

      this->get_parameter(
        "linear_acceleration_stddev", linear_acceleration_stddev_);
      this->get_parameter("angular_velocity_stddev", angular_velocity_stddev_);
      this->get_parameter("magnetic_field_stddev", magnetic_field_stddev_);
      this->get_parameter("orientation_stddev", orientation_stddev_);
    ``` 
    This values will be updated based on your .config or .yaml file when you launch the node using .launch.py.  
    So, don't worry about value and just match value type. (If parameters are not declared in .config or .yaml, you need to insert your values)

### Change launch.py files
To change port name, you have to change .config or .yaml file due to original launch.py files not declare this variable as a parameter. Therefore, converting this variable into a declared parameter is convenient.

Forexample, if you want to change nmea_navsat_driver, replace nmea_serial_driver.launch.py to below code.
```python
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate a launch description for a single serial driver."""
    config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "nmea_serial_driver.yaml")
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM2')
    driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        remappings=[("fix", "ascen/fix")],
        parameters=[config_file, {'port': LaunchConfiguration('port')}])

    return LaunchDescription([port_arg, driver_node])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)
```  
Next, remove **port:** parameter in ublox_gps/config/zed_f9p.yaml.  
Open terminal and input below code to check changed launch.py file.
```
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py -s
```
