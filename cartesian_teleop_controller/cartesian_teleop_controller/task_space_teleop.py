#!/usr/bin/env python3
import os
import sys
import termios
import tty
import tempfile
import numpy as np
from numpy.linalg import pinv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

import xacro
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R



# Joint names as in URDF
JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3','joint_4', 'joint_5', 'joint_6']

# keyboard read 
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


# 6 DOF Task space teleoperation Node 
class TaskSpaceTeleop6DOF(Node):

    def __init__(self):
        super().__init__("task_space_teleop_6dof")

        desc_pkg = get_package_share_directory("dsr_a0509_description")
        xacro_path = os.path.join(desc_pkg, "urdf", "a0509.urdf.xacro") # Load URDF via XACRO for IKPy

        doc = xacro.process_file(xacro_path)
        urdf_xml = doc.toprettyxml()

        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
        tmp.write(urdf_xml.encode())
        tmp.close()

        # Build IK chain with added tool link to visulize the end effector motion.
        self.chain = Chain.from_urdf_file(tmp.name,base_elements=["base_link"],
        active_links_mask=[False, True, True, True, True, True, True, False])
        # active_links_mask=[False, True, True, True, True, True, True]

        # Initial joint configuration to avoid singularity 
        self.q = np.deg2rad([90, 0, 90, 0, 90, 0])  # Defines the starting Cartesian pose

        # Teleop increments
        self.step_trans = 0.002   # meters per key press
        self.step_rot = 0.01      # radians per key press

        # Initial forward kinematics
        fk = self.chain.forward_kinematics(np.concatenate(([0.0], self.q, [0.0])))
        self.ee_pos = fk[:3, 3]      # End-effector position
        self.ee_rot = fk[:3, :3]     # End-effector orientation

        # Joint state publisher (rviz listens here)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        self.get_logger().info(
            "\nFull 6-DOF task-space teleop READY\n"
            "------------------------\n"
            "Press\n\n"
            "A/D : +X  / -X\n"
            "S/W : +Y  / -Y\n"
            "F/R : +Z  / -Z\n"
            "I/K : +Rx / -Rx\n"
            "J/L : +Ry / -Ry\n"
            "U/O : +Rz / -Rz\n"
            "Q   : stop teleop and quit")

    # Numerical Jacobian (6x6 defines rank of matrix )
    def compute_jacobian(self, q):
        """
        Computes numerical Jacobian.
        J maps Cartesian twist to joint velocity.
        """
        eps = 1e-6
        J = np.zeros((6, 6))

        fk_ref = self.chain.forward_kinematics(
            np.concatenate(([0.0], q, [0.0]))
        )
        pos_ref = fk_ref[:3, 3]
        rot_ref = fk_ref[:3, :3]

        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps

            fk_new = self.chain.forward_kinematics(
                np.concatenate(([0.0], q + dq, [0.0]))
            )

            # Linear velocity 
            J[:3, i] = (fk_new[:3, 3] - pos_ref) / eps

            # Angular velocity or rotation vector
            R_err = fk_new[:3, :3] @ rot_ref.T
            J[3:, i] = R.from_matrix(R_err).as_rotvec() / eps

        return J

    # Main teleoperation loop
    def run(self):
        while rclpy.ok():
            key = get_key()
            delta_x = np.zeros(6)  # [dx, dy, dz, droll, dpitch, dyaw]

            # Translational Mapping 
            if key == "w":
                delta_x[0] += self.step_trans
            elif key == "s":
                delta_x[0] -= self.step_trans
            elif key == "a":
                delta_x[1] += self.step_trans
            elif key == "d":
                delta_x[1] -= self.step_trans
            elif key == "r":
                delta_x[2] += self.step_trans
            elif key == "f":
                delta_x[2] -= self.step_trans

            # Orientational Mapping
            elif key == "i":
                delta_x[3] += self.step_rot
            elif key == "k":
                delta_x[3] -= self.step_rot
            elif key == "j":
                delta_x[4] += self.step_rot
            elif key == "l":
                delta_x[4] -= self.step_rot
            elif key == "u":
                delta_x[5] += self.step_rot
            elif key == "o":
                delta_x[5] -= self.step_rot

            elif key == "q":
                break
            else:
                continue

            # Convert world translation to End effector frame
            delta_trans_ee = self.ee_rot.T @ delta_x[:3]
            delta_cart = np.concatenate([delta_trans_ee, delta_x[3:]])

            # try:
            #     q_sol = self.chain.inverse_kinematics(
            #         target_position=self.ee_pos,
            #         target_orientation=self.ee_rot,
            #         initial_position=np.concatenate(([0.0], self.q))
            #     )
            #     self.q = q_sol[1:]  # skip dummy 0
            # except Exception as e:
            #     self.get_logger().warn(f"IK failed: {e}")
                # continue

            # Inverse Kinematics with jacobian
            J = self.compute_jacobian(self.q)  #dq = J⁺ * dx
            dq = pinv(J) @ delta_cart
            self.q += dq

            # Update Endeffector pose
            fk = self.chain.forward_kinematics(np.concatenate(([0.0], self.q, [0.0])))
            self.ee_pos = fk[:3, 3]
            self.ee_rot = fk[:3, :3]

            # Publish joint states
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = self.q.tolist()
            self.pub.publish(msg)


# Main
def main():
    rclpy.init()
    node = TaskSpaceTeleop6DOF()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()




# #!/usr/bin/env python3
# import os
# import sys
# import termios
# import tty
# import tempfile
# import numpy as np

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from ament_index_python.packages import get_package_share_directory

# import xacro
# from ikpy.chain import Chain

# # Joint names in your URDF
# JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
#                'joint_4', 'joint_5', 'joint_6']

# def get_key():
#     fd = sys.stdin.fileno()
#     settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         key = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, settings)
#     return key

# class TaskSpaceTeleop(Node):
#     def __init__(self):
#         super().__init__("task_space_teleop")

#         # -----------------------------
#         # Load XACRO → URDF
#         # -----------------------------
#         desc_pkg = get_package_share_directory("dsr_a0509_description")
#         xacro_path = os.path.join(desc_pkg, "urdf", "a0509.urdf.xacro")
#         doc = xacro.process_file(xacro_path)
#         urdf_xml = doc.toprettyxml()

#         tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
#         tmp.write(urdf_xml.encode())
#         tmp.close()

#         self.chain = Chain.from_urdf_file(
#             tmp.name,
#             base_elements=["base_link"],
#             active_links_mask=[False, True, True, True, True, True, True]
#         )

#         # -----------------------------
#         # Initial joint configuration
#         # -----------------------------
#         self.q = np.deg2rad(np.array([90, 0, 90, 0, 90, 0]))  # radians

#         # Step size for smooth translation
#         self.step = 0.002

#         # -----------------------------
#         # Compute initial EE position and orientation
#         # -----------------------------
#         fk = self.chain.forward_kinematics(np.concatenate(([0.0], self.q)))
#         self.ee_pos = fk[:3, 3]
#         self.ee_rot = fk[:3, :3]  # fixed rotation

#         # -----------------------------
#         # Publisher
#         # -----------------------------
#         self.pub = self.create_publisher(JointState, "/joint_states", 10)

#         self.get_logger().info(
#             "✅ Task-space teleop READY\n"
#             "Move: W/S: +X/-X, A/D: +Y/-Y, R/F: +Z/-Z\n"
#             "Orientation is FIXED.\n"
#             "Q: quit"
#         )

#     def run(self):
#         while rclpy.ok():
#             key = get_key()

#             # ---------------- Position control ----------------
#             if key == "w":
#                 self.ee_pos[0] += self.step
#             elif key == "s":
#                 self.ee_pos[0] -= self.step
#             elif key == "a":
#                 self.ee_pos[1] += self.step
#             elif key == "d":
#                 self.ee_pos[1] -= self.step
#             elif key == "r":
#                 self.ee_pos[2] += self.step
#             elif key == "f":
#                 self.ee_pos[2] -= self.step
#             elif key == "q":
#                 break
#             else:
#                 continue

#             # ---------------- Compute IK (position + fixed orientation) ----------------
#             try:
#                 q_sol = self.chain.inverse_kinematics(
#                     target_position=self.ee_pos,
#                     target_orientation=self.ee_rot,
#                     initial_position=np.concatenate(([0.0], self.q))
#                 )
#                 self.q = q_sol[1:]  # skip dummy 0
#             except Exception as e:
#                 self.get_logger().warn(f"IK failed: {e}")
#                 continue

#             # ---------------- Publish joint states ----------------
#             msg = JointState()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.name = JOINT_NAMES
#             msg.position = self.q.tolist()
#             self.pub.publish(msg)

# def main():
#     rclpy.init()
#     node = TaskSpaceTeleop()
#     try:
#         node.run()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()






