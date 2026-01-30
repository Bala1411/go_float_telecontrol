#!/usr/bin/env python3
import os
import sys
import tempfile
import termios
import tty
import select
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

import xacro
from ikpy.chain import Chain
from ikpy.utils import geometry as ik_geom

# Joint names in your URDF
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

# Degrees → Radians for initial pose
INITIAL_JOINTS_DEG = [90, 0, 90, 0, 90, 0]

class TaskSpaceIKTeleop(Node):
    def __init__(self):
        super().__init__("task_space_ik_teleop")

        # -----------------------------
        # Load URDF from XACRO
        # -----------------------------
        desc_pkg = get_package_share_directory("dsr_a0509_description")
        xacro_path = os.path.join(desc_pkg, "urdf", "a0509.urdf.xacro")

        doc = xacro.process_file(xacro_path)
        urdf_xml = doc.toprettyxml()

        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
        tmp.write(urdf_xml.encode())
        tmp.close()

        self.chain = Chain.from_urdf_file(
            tmp.name,
            base_elements=["base_link"],
            active_links_mask=[False, True, True, True, True, True, True]
        )

        self.num_joints = len(self.chain.links) - 1

        # -----------------------------
        # Initial joint pose (radians)
        # -----------------------------
        self.current_joints = np.deg2rad(np.array(INITIAL_JOINTS_DEG))

        # Compute initial end-effector position
        self.ee_pos = ik_geom.forward_kinematics(
            self.chain,
            np.concatenate(([0.0], self.current_joints))
        )[:3, 3]

        # Teleop step size (meters)
        self.step = 0.01

        # -----------------------------
        # Publisher to RViz
        # -----------------------------
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # -----------------------------
        # Timer for loop
        # -----------------------------
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("✅ Task-space IK teleop READY")
        self.get_logger().info("Use WASD (XY), R/F (Z), Q to quit")

    # -----------------------------
    # Non-blocking keyboard read
    # -----------------------------
    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return None

    # -----------------------------
    # Teleop + IK loop
    # -----------------------------
    def control_loop(self):
        key = self.get_key()
        if key is None:
            return

        if key == "w":
            self.ee_pos[0] += self.step
        elif key == "s":
            self.ee_pos[0] -= self.step
        elif key == "a":
            self.ee_pos[1] += self.step
        elif key == "d":
            self.ee_pos[1] -= self.step
        elif key == "r":
            self.ee_pos[2] += self.step
        elif key == "f":
            self.ee_pos[2] -= self.step
        elif key == "q":
            self.get_logger().info("Quit teleop")
            rclpy.shutdown()
            return

        try:
            # Compute IK from current joint state
            ik = self.chain.inverse_kinematics(
                target_position=self.ee_pos,
                initial_position=np.concatenate(([0.0], self.current_joints))
            )

            # Update current joints (skip first dummy 0)
            self.current_joints = ik[1:]

            # Publish to RViz
            self.publish_joints(self.current_joints)

        except Exception as e:
            self.get_logger().warn(f"IK failed: {e}")

    # -----------------------------
    # Publish joint positions
    # -----------------------------
    def publish_joints(self, joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = joints.tolist()
        self.joint_pub.publish(msg)

def main():
    rclpy.init()
    node = TaskSpaceIKTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
