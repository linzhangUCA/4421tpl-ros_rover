# Project Instructions

## Overview
The scientists on the Earch are thrilled and encouraged by the successful recovery of Element M2-5 during the R1B rover's mission.
To futher explore the planet, an upgraded rover, HomeR, will be sent to LSC-001e to sample the java over there.
The geography of the planet did not change much, and the home base is staying at same location.
HomeR is featured with:
- Two [brushed DC motors with encoders](https://www.pololu.com/product/4805) for the controllable differential drive mobile base.
- An [inertial measurement unit (IMU)](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) for extra information of the robot's motion.
- A [light detection and ranging (LiDAR) sensor](http://bucket.download.slamtec.com/b90ae0a89feba3756bc5aaa0654c296dc76ba3ff/LD108_SLAMTEC_rplidar_datasheet_A1M8_v2.2_en.pdf) for measugin surrounding object's distances.

In this project, you and your teammate(s) are expected to achieve following goals. 
- Develop an autonomous navigation system with the help of the new and upgraded hardware.
- Test the dead reckoning odometry and analyze the affection of the motion sensors.
- Adapt new sensors to the ROS framework.
- Document your designs and usage of the robot using this Github repository.

![ros_rover](/drawings/ros_rover.drawio.png)


## Resources
- Please refer to HomeR's [documentation](https://linzhanguca.github.io/homer_docs) if you need to be inspired.
- ROS official [document](https://docs.ros.org/en/jazzy/index.html) site is the gold.

## Requirements
### Deliverables
- Upload ROS package(s) with dedicated executables and launch files for navigation.
- Write [README](README.md).
- Demonstrate autonoomus navigation.

### 1. Navigation Strategy
Describe your navigation strategy with math language. 
Write your methodology down in [README](README.md)
- If your robot's navigation is broken down into several phases, specify and briefly describe them.
- For each phase, clearly define key physics quantities involved using (commonly accecpted) letters/symbols/characters.
- Write down equations guiding your robot to the destination then plug in actual data/measurements to verify your equations. 

### 2. ROS Infrastructure
- Publish RPLidar A1's data under the `/scan` topic with [`sensor_msgs/LaserScan`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html) message (at a reasonable frequency).
- Publish MPU6050's data under the `/imu` topic with [`sensor_msgs/Imu`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html) message at 50 Hz frequency.
- Publish robot's pose and velocity under the `/odom` topic with [`nav_msgs/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) message at 50 Hz frequency.
- Broadcast transformation from `odom` frame to `base_link` frame at 50 Hz frequency.
- Define and illustrate `odom` frame and `base_link` frame (from a reasonable viewing angle). 

### 3. Odometry

### 4. LiDAR Assistance

### 5. ROS Package Organization 
- (40%) Develop ROS package executable(s) with at least one node to fulfill the following demands:
  - **Subscribe** to the `/cmd_vel` **topic** and retrieve the correct linear and angular velocity from the embedded [`Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) **message**.
  - **Transmit** the subscribed linear and angular velocity to the Pico board as the reference velocity for the robot.
  - **Receive** measured velocity from the Pico board at an appropriate rate.
  - **Publish** a message uDARnder the **`/<your_robot_name>/measured_velocity` topic** at **50 Hz** with a [`Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) **message**.
- (5%) Edit `setup.py` (or `CMakeLists.txt` if your package is with type of `ament_cmake`) to introduce all your executables to ROS.
- (5%) Edit `setup.py` and `package.xml` with correct `description`, `maintainer`, `email`, and `license` information.
- (10%) Illustrate the relationship among your node(s), [`teleop_twist_joy`](https://index.ros.org/r/teleop_twist_joy/#jazzy) node and the [`teleop_twist_keyboard`](https://index.ros.org/r/teleop_twist_keyboard/#jazzy) node use a node graph with topics and messages information. Upload the node graph to [drawings/](drawings/) and **display it in the [README](README.md)**.

> [!WARNING]
> Your grade will be hugely discounted if the submitted package failed `colcon build` on instructor's machine.

> [!NOTE]
> - To validate if your executable is recognizable by ROS, run command: `ros2 run <package_name> <executable_name>`.
> - It is students' responsibility to maintain the code running on their teams' Pico boards. Team failed to bring up a functional Pico board during the demonstration will loss 50% of the ROS Package Development credits.
> - Extra 1% of project total credits will be given to the teams achieved to **launch** all the nodes (include `teleop_twist_joy` node and the `teleop_twist_keyboard` node) with only one command.

### 6. AI Policies
Please acknowledge AI's contributions according to the policies in the syllabus.

## Demonstration Rules
> [!IMPORTANT]
> - Demonstrations are scheduled on Thursday, 02/12/2026 from 10:50 AM to 1:30 PM at the robotics lab (LSCA 105).
> - Teams with fewer attempts have higher priority in the cue.
> - **Changes submitted after the demonstration day will not be graded**.
> - To secure a good demonstration, make sure you and your teammate(s) practice it a couple times ahead. And the batteries are fully charged.

## Bonus Points
- Mechanical
- Electrical
- LiDAR guided navigation
- Final distance to homne base

