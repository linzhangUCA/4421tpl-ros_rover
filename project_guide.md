# Project Instructions

## Overview
The scientists on the Earch are thrilled and encouraged by the successful recovery of Element M2-5 during the R1B rover's mission.
To futher explore the planet, an upgraded rover, HomeR, will be sent to LSC-001e to sample the java over there.
The geography of the planet did not change much, and the home base is staying at same location.
You roboticists' mission is to develop an approach to navigate HomeR from the sampling site back to the home base autonomously and safely.  

![ros_rover](/drawings/ros_rover.drawio.png)

In this project, you and your teammate(s) are expected to achieve following goals. 
- Develop an autonomous navigation system with the help of the new and upgraded hardware.
- Adapt new sensors to the ROS framework and come up with navigation strategies based upon them.
- Analyze the affection of the motion sensors.

## Resources
- HomeR [documentation](https://linzhanguca.github.io/homer_docs).
- ROS 2 Jazzy official [documentation](https://docs.ros.org/en/jazzy/index.html).
Featured components on HomeR:
- Two [brushed DC motors with encoders](https://www.pololu.com/product/4805) for the controllable differential drive mobile base.
- An [inertial measurement unit (IMU)](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) for extra information of the robot's motion.
- A [light detection and ranging (LiDAR) sensor](http://bucket.download.slamtec.com/b90ae0a89feba3756bc5aaa0654c296dc76ba3ff/LD108_SLAMTEC_rplidar_datasheet_A1M8_v2.2_en.pdf) for measuring surrounding objects' distance.

## Demonstration Debriefing
### Procedure
Each teammember needs to individually complete the first 4 steps below at least once.
1. Place the robot on/behind the "Start Line".
2. Start all ROS nodes required by the autonomous navigation.
3. Stop the robot and end the navigation with a visible indication.
4. Distance from the robot to the center of the "Home Base" will be measured to determine your team's bonus and penalty.
5. (Optional) Take interview, one question for each teammember.

> [!NOTE]
> **Bonus Points**:
> (1%) Copiable Linux commands.

> [!IMPORTANT]
> This project is the most difficult one among the three this semester.
Please make sure you are very familiar with the robot and extensive practice ahead of the demonstration is recommended.   
> - Demonstrations need to be done **on/before** Thursday, 03/12/2026 1:30 PM at the robotics lab (LSCA 105).
> - Each teammember has **5 attempts**. Best record will be used for grading.
> - **Repository changes submitted after the demonstration day will not be graded**.


## Requirements
### Deliverables
- Upload ROS package(s) with dedicated executables and launch files for navigation.
- Write [README](README.md).
- Demonstrate autonoomus navigation.

### 1. Usage Instructions
Assume you are going to use the ROS package(s) developed for this project on a Raspberry Pi with newly installed ROS 2 Jazzy.
- Please write down all the key steps to start the navigation in [README](README.md)
> [!NOTE]
> **Bonus Points**:
> (1%) Copiable Linux commands.

### 2. Navigation Strategy
Describe your navigation strategy with math language. 
Write your methodology down in [README](README.md)
- If your robot's navigation is broken down into several phases, specify and briefly describe them.
- For each phase, clearly define key physics quantities involved using (commonly accecpted) letters/symbols/characters.
- Write down equations guiding your robot to the destination then plug in actual data/measurements to verify your equations. 

### 3. ROS Infrastructure
- Let `/cmd_vel` topic with [`geometry_msgs/Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) message be the only one taking care of the mobile base's movement.
Publish `/cmd_vel` topic with reasonable values at appropriate instants to navigate the robot to the **HOME BASE**.
- Publish LiDAR data under the `/scan` topic with [`sensor_msgs/LaserScan`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html) message (at a reasonable frequency).
- Publish IMU data under the `/imu` topic with [`sensor_msgs/Imu`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html) message at 50 Hz frequency.
- Publish robot's pose and velocity under the `/odom` topic with [`nav_msgs/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) message at 50 Hz frequency.
- Broadcast transformation from `odom` frame to `base_link` frame at 50 Hz frequency.
- Define and illustrate `odom` frame and `base_link` frame (from a reasonable viewing angle) in [README](README.md).
- Edit related files so that the pakcage(s) can be built by `colcon`. Make sure the executables, launch files, config files, etc. are registered.
- Edit `setup.py` and `package.xml` with correct `description`, `maintainer`, `email`, and `license` information.
- Illustrate the relationship among your node(s), [`teleop_twist_joy`](https://index.ros.org/r/teleop_twist_joy/#jazzy) node and the [`teleop_twist_keyboard`](https://index.ros.org/r/teleop_twist_keyboard/#jazzy) node use a node graph with topics and messages information. Upload the node graph to [drawings/](drawings/) and **display it in the [README](README.md)**.

> [!NOTE]
> **Bonus Points**:
> - LiDAR data is not just published, but also used for navigation (e.g. wall following). 
> - Publish and broadcast improved odometry by fusing IMU's and encoders' data.
> - Launch everything with one command.

### 4. Motion Sensing Analysis
Start a navigation and record the data in `/imu` and `/odom` topics using [`rosbag`](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).
- Upload the recorded `rosbag` file. 
- Calculate the robot's trajectory using recorded `/odom` (encoder) data.
Upload trajectory graph to the repository.
- Calculate the robot's trajectory using recorded `/imu` data.
Upload trajectory graph to the repository. 
- Compare and analyze performance of these sensors.

> [!NOTE]
> **Bonus Points**:
> - Upload estimated robot trajectory using fused motion sensor data.
> - Analysis includes fused motion sensing. 

## AI Policies
Please acknowledge AI's contributions according to the policies in the syllabus.

