import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class CartesianIKController(Node):

    def __init__(self):
        super().__init__('cartesian_ik_controller')

        self.create_subscription(
            Twist,
            '/teleop/cartesian_delta',
            self.teleop_cb,
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.current_joint_state = None
        self.get_logger().info("Cartesian IK Controller started")

    def joint_state_cb(self, msg):
        self.current_joint_state = msg

    def teleop_cb(self, msg):
        if self.current_joint_state is None:
            return

        # Placeholder – IK logic comes next
        self.get_logger().info("Received Cartesian delta")


def main():
    rclpy.init()
    node = CartesianIKController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
