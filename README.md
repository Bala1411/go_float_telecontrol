# Cartesian Task-Space Teleoperation Controller (ROS 2)

A ROS 2–based **Cartesian task-space teleoperation framework** for a 6-DOF robotic manipulator.

This project enables **incremental, safe end-effector control in task space**, visualized in **RViz**, and is architected to **seamlessly support future force-compliance (admittance / impedance) control**.

---

##  Features

* Full **6-DOF Cartesian teleoperation**
* **Jacobian-based task-space inverse kinematics**
* Clean separation of **input**, **control**, and **robot description**
* **RViz-only safe execution** (no torque commands)
* **Compliance-ready architecture** (no refactor required later)
* ROS 2 **Python packages (Humble-compatible)**

---

##  System Architecture (High Level)

```text
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
```

---

##  Project Folder Structure

```text
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
```

---

##  Package Overview

### `cartesian_teleop_controller`

**Core control and kinematics package**
Handles all **task-space** and **joint-space** control logic.

#### Key files

* `task_space_teleop.py`
  Task-space teleoperation control loop

* `joint_space_controller.py`
  Joint-space teleoperation control loop

> Other files were developed during initial stages and can be ignored for core understanding.

---

### `dsr_a0509_description`

**Robot model and visualization package**
Contains the complete **URDF/Xacro description**, meshes, and **RViz configuration**.

#### Key files

* `urdf/a0509.urdf.xacro` – Main robot description
* `xacro/macro.a0509.white.xacro` – Reusable robot macros
* `launch/display.launch.py` – RViz visualization launch
* `teleop.rviz` – Preconfigured RViz layout

---

### `teleop_input`

**User input abstraction layer**
Responsible for mapping **keyboard input → Cartesian motion commands**.

#### Key file

* `keyboard_teleop.py`
  Keyboard-based teleoperation used for **joint-space teleoperation control loop**

---

### `robot_bringup`

Reserved for **real robot integration**, hardware drivers, and controller startup.

---

##  Detailed Design Documentation

The **structured and detailed explanation** for:

* Node architecture
* Control logic
* Control strategy used
* Why the system is safe
* Where force-compliance (admittance / impedance) plugs in

are documented in the following files:

* **`6-DOF Task-Space Teleoperation Architecture.pdf`**
* **`Joint Space Teleop.pdf`**

Kindly refer to these documents for a **deep technical understanding** of the system design.

---

##  Author

**Balachandar P**
Robotics Engineer – Task-Space Control, Teleoperation, Medical Robotics
