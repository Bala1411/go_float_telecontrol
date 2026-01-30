In this project I doing full 6 dof teleoperation using keyoard inputs in both task space control using inverse kinematics and joint spcae control.

For both control the initial robot launch steps are same.

steps:
1. run ros2 launch dsr_a0509_description display.launch.py in one terminal
Rviz will open and joint state publisher gui will open.
2. now in rviz go to-->file-->open config, and select the config file in this path '/src/dsr_a0509_description/teleop.rviz'.
3. Now close the joint state publisher gui before moving into teleoperation, because teleopeartion nodes also /joint_states for communication.

Task_space_teleoperation:
1. Once the rviz is setted, then in another terminal run, ros2 run cartesian_teleop_controller task_space_teleop, then based on the keyboard input instaructions move the robot.
"Press\n\n"
            "A/D : +X  / -X\n"
            "S/W : +Y  / -Y\n"
            "F/R : +Z  / -Z\n"
            "I/K : +Rx / -Rx\n"
            "J/L : +Ry / -Ry\n"
            "U/O : +Rz / -Rz\n"
            "Q   : stop teleop and quit"

The structured and detailed explanation for 
  ○ Node architecture
  ○ Control logic

  ○ Control strategy used
  ○ Why it is safe
  ○ Where force-compliance would plug in.

  are mentioned in document named 6-DOF Task-Space Teleoperation Architecture.pdf . Kindly refer to this document for detailed understanding.

  Joint_space_teleoperation:
  1. Follow the same steps for rviz setting.
  2. Now in another terminal run , ros2 run cartesian_teleop_controller joint_space_controller 
  3. then in another terminal run , ros2 run teleop_input keyboard_teleop and follow the keyboard input instructions.
  "A/D : +J1 / -J1\n"
            "W/S : +J2 / -j2\n"
            "Q/E : +J3 / -J3\n"
            "I/K : +J4 / -J4\n"
            "J/L : +J5 / -J5\n"
            "U/O : +J6 / -J6\n"

The structured and detailed explanation for 
  ○ Node architecture
  ○ Control logic

  ○ Control strategy used
  ○ Why it is safe
  ○ Where force-compliance would plug in.

  are mentioned in document named Joint Space Teleop .pdf . Kindly refer to this document for detailed understanding.

   project_root
├──  cartesian_teleop_controller
│   ├──  cartesian_teleop_controller
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
└── 6-DOF Task-Space Teleoperation Architecture.pdf*
└── Joint Space Teleop .pdf*

Package Descriptions

cartesian_teleop_controller:

Contains all task-space and joint-space control logic.

Numerical Jacobian computation

Jacobian pseudoinverse IK

Task-space teleoperation pipeline

Joint state publishing (/joint_states)

This is the core control package.

dsr_a0509_description:

Robot description and visualization package.

URDF/XACRO model of the robot

Meshes and visual assets

RViz configuration

Display and teleop launch files

Used by:

robot_state_publisher

RViz visualization

IK model loading (IKPy)

teleop_input:

Dedicated input abstraction package.

Keyboard teleoperation logic

Clean separation between input and control

Easily extendable to joystick / haptic devices

robot_bringup:

(Reserved / optional)

Future hardware bringup

Controller startup

Sensor integration

