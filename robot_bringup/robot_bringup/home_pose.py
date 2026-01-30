import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HomePosePublisher(Node):

    def __init__(self):
        super().__init__('home_pose_publisher')

        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_home)

        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        self.home_position = [
            1.5708, 0.0, 1.5708,
            0.0,    1.5708, 0.0
        ]

        self.get_logger().info("Publishing HOME pose")

    def publish_home(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.home_position
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HomePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
