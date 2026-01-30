Cartesian Task-Space Teleoperation Controller (ROS 2)

A ROS 2-based Cartesian task-space teleoperation framework for a 6-DOF robotic manipulator.
This project enables incremental, safe end-effector control in task space, visualized in RViz, and is architected to seamlessly support future force-compliance (admittance / impedance) control.

 Features

 Full 6-DOF Cartesian teleoperation
 Jacobian-based task-space inverse kinematics
 Clean separation of input, control, and robot description
 RViz-only safe execution (no torque commands)
 Compliance-ready architecture (no refactor required later)
 ROS 2 Python packages (Humble-compatible)

System Architecture (High Level)

User Input (Keyboard / Haptic)
        ↓
Cartesian Increment Δx
        ↓
(Optional Compliance Layer)
        ↓
Jacobian Pseudo-Inverse IK (J⁺)
        ↓
Joint Increment Δq
        ↓
Robot Model (RViz / Hardware)

Project Folder Structure


├── cartesian_teleop_controller/
│   ├── cartesian_teleop_controller/
│   │   ├── __init__.py
│   │   ├── ik_controller.py
│   │   ├── joint_space_controller.py
│   │   ├── task_space_ik_control.py
│   │   ├── task_space_teleop.py
│   │   └── __pycache__/
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── dsr_a0509_description/
│   ├── config/
│   ├── include/
│   ├── launch/
│   │   ├── display.launch.py
│   │   └── display_teleop.launch.py
│   ├── meshes/
│   ├── ros2_control/
│   ├── src/
│   ├── urdf/
│   │   └── a0509.urdf.xacro
│   ├── xacro/
│   │   └── macro.a0509.white.xacro
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── teleop.rviz
│
├── teleop_input/
│   ├── teleop_input/
│   │   ├── __init__.py
│   │   ├── keyboard_teleop.py
│   │   └── __pycache__/
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── robot_bringup/
├── picture/
└── README.md



Package Overview

cartesian_teleop_controller

Core control and kinematics package
Handles all task-space and joint-space control logic.


Key files

task_space_teleop.py
Task-space teleoperation control loop

joint_space_controller.py
joint-space teleoperation control loop

Ignore other files.Because they are developed during initial stages.

dsr_a0509_description

Robot model and visualization
Contains the complete URDF/Xacro description, meshes, and RViz configuration.

Key files

urdf/a0509.urdf.xacro – main robot description
xacro/macro.a0509.white.xacro – reusable macros
launch/display.launch.py – RViz visualization
teleop.rviz – preconfigured RViz layout

teleop_input

User input abstraction
Responsible for mapping keyboard input → Cartesian motion commands.

Key file

keyboard_teleop.py 
keyboard-based teleoperation used for joint-space teleoperation control loop

robot_bringup

Reserved for real robot integration, hardware drivers, and controller startup.

The structured and detailed explanation for 
  ○ Node architecture
  ○ Control logic
  ○ Control strategy used
  ○ Why it is safe
  ○ Where force-compliance would plug in.

 are mentioned in a document named 6-DOF Task-Space Teleoperation Architecture.pdf and Joint Space Teleop .pdf  Kindly refer to this document for detailed understanding.

Author
Balachandar P
Robotics Engineer – Task-Space Control, Teleoperation, Medical Robotics


