# Project Instructions

## Overview
Recall the final project of Robotics I.
Were you struggling with managing the components on the lab rover (despite how naive it was)?
Have you ever imagined upgrading the rover with extended computers, sensors and actuators?

The [Robot Operating System (ROS)](https://docs.ros.org/en/jazzy/index.html) is here to help.
With its inherent modularity, which effectively turns software development into a "building block" architecture similar to assembling Legos. 
Rather than writing a single, monolithic script that is hard to maintain and debug, ROS allows you to break your robot’s complex functions into independent, isolated "nodes" that communicate through a universal messaging system. 

In this project, you and your teammate(s) are expected to achieve following goals. 
- Build a mobile base for a coffee delivery robot.
- Test and verify motor control and inter-computer communications of the robot.
- Manage the robot with a ROS package and nodes.
- Document your designs and usage of the robot using this Github repository.


## Resources
- Please refer to HomeR's [documentation](https://linzhanguca.github.io/homer_docs) if you need to be inspired.
- Mechanical designs using FreeCAD and 3D printable parts of HomeR are hosted here: [https://github.com/linzhangUCA/homer_me](https://github.com/linzhangUCA/homer_me).
- KiCAD designs of the electronics relay PCB are hosted here: [https://github.com/linzhangUCA/homer_ee](https://github.com/linzhangUCA/homer_ee).
- MicroPython scripts for controlling the differential drive mobile base of the HomeR can be found here: [https://github.com/linzhangUCA/homer_pico](https://github.com/linzhangUCA/homer_pico).
- ROS official [document](https://docs.ros.org/en/jazzy/index.html) site is the gold.

## Requirements
- Upload your ROS package to this repository.
- Complete documentation in [README](README.md) (, a practice to inspire interested people).

### 1. (5%) Portrait
- Take a **clear** picture of your robot.
- **Label** the key components of the robot in the picture.
- **Display** the labeled portrait in [README](README.md).

> [!IMPORTANT]
> The portrait has to match the robot used for the demonstration.

> [!TIP]
> Find a good viewing angle.
> BearCar has an [example](https://ucaengineeringphysics.github.io/bearcar_docs/images/bearcar_annotate.png).

### 2. (15%) Mechanical Design.
Please upload all the mechanical drawings into the [drawings/mechanical](drawings/mechanical) directory to illustrate the following.
- (5%) All 3D printed and customized parts' dimensions.
- (5%) The driving wheel's dimensions.
- (5%) Layout (spatial relationship) of the base, Raspberry Pi 5, driving wheels and the caster wheel.

> [!NOTE]
> - Engineering drawings' [format](https://www.mcgill.ca/engineeringdesign/step-step-design-process/basics-graphics-communication/drawing-format-and-elements) is prefered.
> - **Bonus** will be given to **innovated** mechanical designs in a scale of 0% to 5% of the project's total.

### 3. (20%) Electrical Design.
Please upload all the electrical drawings into the [drawings/electrical](drawings/electrical) directory to illustrate the following.
- (10%) Power distribution/management for all the electrical components.
  - Please specify input and/or output voltage of each component.
  - Please specify the power connectors/wires' names.
- (10%) Signal flows among Raspberry Pi 5, Pico, and the motor drivers.
  - Please specify hardware interfaces used for these signals.
  - Please specify direction of each signal.

> [!NOTE]
> - The electric flows output to motors are considered as power consumptions, thus can be ignored in the signal wiring diagram.  
> - **Bonus** will be given to innovated electrical designs in a scale of 0% to 5% of the project's total.

### 4. (60%) ROS Package Development
- (40%) Develop ROS package executable(s) with at least one node to fulfill the following demands:
  - **Subscribe** to the `/cmd_vel` **topic** and retrieve the correct linear and angular velocity from the embedded [`Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) **message**.
  - **Transmit** the subscribed linear and angular velocity to the Pico board as the reference velocity for the robot.
  - **Receive** measured velocity from the Pico board at an appropriate rate.
  - **Publish** a message under the **`/<your_robot_name>/measured_velocity` topic** at **50 Hz** with a [`Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) **message**.
- (5%) Edit `setup.py` (or `CMakeLists.txt` if your package is with type of `ament_cmake`) to introduce all your executables to ROS.
- (5%) Edit `setup.py` and `package.xml` with correct `description`, `maintainer`, `email`, and `license` information.
- (10%) Illustrate the relationship among your node(s), [`teleop_twist_joy`](https://index.ros.org/r/teleop_twist_joy/#jazzy) node and the [`teleop_twist_keyboard`](https://index.ros.org/r/teleop_twist_keyboard/#jazzy) node use a node graph with topics and messages information. Upload the node graph to [drawings/](drawings/) and **display it in the [README](README.md)**.

> [!WARNING]
> Your grade will be hugely discounted if the submitted package failed `colcon build` on instructor's machine.

> [!NOTE]
> - To validate if your executable is recognizable by ROS, run command: `ros2 run <package_name> <executable_name>`.
> - It is students' responsibility to maintain the code running on their teams' Pico boards. Team failed to bring up a functional Pico board during the demonstration will loss 50% of the ROS Package Development credits.
> - Extra 1% of project total credits will be given to the teams achieved to **launch** all the nodes (include `teleop_twist_joy` node and the `teleop_twist_keyboard` node) with only one command.

### 5. AI Policies
Please acknowledge AI's contributions according to the policies in the syllabus.

## Demonstration Rules
> [!IMPORTANT]
> - Demonstrations are scheduled on Thursday, 02/12/2026 from 10:50 AM to 1:30 PM at the robotics lab (LSCA 105).
> - Teams with fewer attempts have higher priority in the cue.
> - **Changes submitted after the demonstration day will not be graded**.
> - To secure a good demonstration, make sure you and your teammate(s) practice it a couple times ahead. And the batteries are fully charged.

We will reuse the track from Robotics 1's [final project](https://classroom.github.com/a/96OtuQyX) to demonstrate the effectiveness of ROS management and validity of motor control by following steps.
1. Start/Launch/Run all the ROS nodes for the remote control demonstration.
2. The first student operator will drive the robot from the "start line" to the "home base" with a generic PS type gamepad.
3. The second student operator will drive the robot from the "home base" back to the "start line" with the keyboard on a laptop. 


