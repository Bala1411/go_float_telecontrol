# Cartesian Task-Space Teleoperation Controller (ROS 2)

A ROS 2–based **Cartesian task-space teleoperation framework** for a 6-DOF robotic manipulator.

This project enables **incremental, safe end-effector control in task space**, visualized in **RViz**, and is architected to **seamlessly support future force-compliance (admittance / impedance) control**.

---

##  Features

* Full **6-DOF Cartesian teleoperation**
* **Jacobian-based task-space inverse kinematics**
* Clean separation of **input**, **control**, and **robot description**
* **RViz-only safe execution** (no torque commands)
* **Compliance-ready architecture** (no refactor required later)
* ROS 2 **Python packages (Humble-compatible)**

---

##  System Architecture (High Level)

```text
User Input (Keyboard / Haptic)
        ↓
Cartesian Increment Δx
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

* **`Taskspace_teleoperation _architecture.pdf`**
* **`Jointspace_teleop .pdf`**

 Kindly refer to these documents for a **deep technical understanding** of the system design.

---

## ▶ How to Run the Teleoperation

This project supports **full 6-DOF teleoperation** using **keyboard inputs** in:

* **Task-space control** (Cartesian teleoperation using inverse kinematics)
* **Joint-space control** (Direct joint-level teleoperation)

For **both modes**, the **initial robot launch and RViz setup steps are identical**.

---

### 🔹 Common Setup (Required for Both Control Modes)

1. Launch the robot model and RViz:

```bash
ros2 launch dsr_a0509_description display.launch.py
```

* RViz will open
* **Joint State Publisher GUI** will also open

2. Load the teleoperation RViz configuration:

In RViz:

```
File → Open Config → select:
/src/dsr_a0509_description/teleop.rviz
```

3. **Close the Joint State Publisher GUI** before starting teleoperation.

> Important: Teleoperation nodes also publish and subscribe to `/joint_states`.
> Keeping the Joint State Publisher GUI open will cause conflicts.

---

##  Task-Space Teleoperation (Cartesian Control)

This mode enables **incremental 6-DOF end-effector motion** in Cartesian space using **Jacobian-based inverse kinematics**.

### Steps

1. Complete the **Common Setup** above.

2. In a **new terminal**, run the task-space teleoperation node:

```bash
ros2 run cartesian_teleop_controller task_space_teleop
```

3. Control the robot using the keyboard:

```text
A / D : +X  / -X
S / W : +Y  / -Y
F / R : +Z  / -Z
I / K : +Rx / -Rx
J / L : +Ry / -Ry
U / O : +Rz / -Rz

Q     : Stop teleoperation and quit
```

The robot will move **incrementally and safely** in task space, visualized entirely in **RViz**.

### Detailed Design Reference

A structured and in-depth explanation of:

* Node architecture
* Control logic
* Control strategy
* Safety guarantees
* Force-compliance (admittance / impedance) integration points

is provided in:

* **`Taskspace_teleoperation _architecture.pdf`**

Kindly refer to this document for a **complete technical understanding**.

---

##  Joint-Space Teleoperation

This mode enables **direct joint-level control** using keyboard inputs.

### Steps

1. Complete the **Common Setup** above.

2. In a **new terminal**, start the joint-space controller:

```bash
ros2 run cartesian_teleop_controller joint_space_controller
```

3. In **another terminal**, start the keyboard input node:

```bash
ros2 run teleop_input keyboard_teleop
```

4. Control individual joints using the keyboard:

```text
A / D : +J1 / -J1
W / S : +J2 / -J2
Q / E : +J3 / -J3
I / K : +J4 / -J4
J / L : +J5 / -J5
U / O : +J6 / -J6
```

This mode is useful for **joint-level testing, debugging, and calibration**.

---

##  Author

**Balachandar P**
Robotics Engineer – 6 DOF Task-Space Control, Teleoperation, Medical Robotics
