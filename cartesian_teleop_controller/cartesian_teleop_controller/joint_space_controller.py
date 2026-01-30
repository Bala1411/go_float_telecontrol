import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
from math import pi


# Joint names as in URDF
JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3',
    'joint_4', 'joint_5', 'joint_6'
]


class JointSpaceController(Node):
    """
    Simple joint-space teleoperation controller.
    - Receives incremental commands as Twist
    - Maps each Twist component to a joint increment
    - Publishes JointState directly (RViz visualization)
    """

    def __init__(self):
        super().__init__('joint_space_controller')

        # Initial joint configuration (home pose)
        
        self.joints = np.deg2rad([90, 0, 90, 0, 90, 0])# Degrees → radians

        # Teleop input incremental commands
        
        self.teleop_sub = self.create_subscription(Twist,'/teleop/pos_delta', self.teleop_cb, 10 )

        # Joint state publisher
        self.js_pub = self.create_publisher(JointState,'/joint_states', 10)

        # self.js_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_cb,10)

        # Publish joint states at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info("Joint space controller started\n" "Initial pose: [90, 0, 90, 0, 90, 0] degrees")

    def teleop_cb(self, msg: Twist):
        """
        Teleop incremental joint control.
        Twist mapping:
        linear --- joints 1,2,3
        angular--- joints 4,5,6
        """

        Step_scale = 0.5  # scaling factor (rad per keypress)

        self.joints[0] += Step_scale * msg.linear.y
        self.joints[1] += Step_scale * msg.linear.x
        self.joints[2] += Step_scale * msg.linear.z
        self.joints[3] += Step_scale * msg.angular.x
        self.joints[4] += Step_scale * msg.angular.y
        self.joints[5] += Step_scale * msg.angular.z

        self.joints = np.clip(self.joints, -pi, pi)       # Safety joint limit protection


    def publish_state(self):
        """
        Publish joint states for RViz/robot_state_publisher.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.joints.tolist()
        self.js_pub.publish(msg)


def main():
    rclpy.init()
    node = JointSpaceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
