import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class KeyboardTeleop(Node):
    """
    Keyboard-based teleoperation node.

    - Publishes incremental commands as geometry_msgs/Twist
    - Twist is used as a *delta command
    - Linear fields  --- joints 1,2,3
    - Angular fields --- joints 4,5,6
    """

    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publisher for incremental teleop commands
        self.pub = self.create_publisher(Twist,'/teleop/pos_delta',10)
        self.timer = self.create_timer(0.05, self.loop)

        self.settings = termios.tcgetattr(sys.stdin)       # terminal  


        # Increment sizes
        self.step = 0.01       # positional increment 
        self.rot_step = 0.05   # rotational increment 

        self.get_logger().info(
            "\nKeyboard Teleop started\n"
            "------------------------\n"
            "A/D : +J1 / -J1\n"
            "W/S : +J2 / -j2\n"
            "Q/E : +J3 / -J3\n"
            "I/K : +J4 / -J4\n"
            "J/L : +J5 / -J5\n"
            "U/O : +J6 / -J6\n"
        )

    def get_key(self):
        """
        Non-blocking keyboard read.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)

        key = sys.stdin.read(1) if rlist else ''

        # Restore terminal state
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        """
        Read key press and publish corresponding Twist delta.
        """
        key = self.get_key()
        if not key:
            return

        msg = Twist()

        # Linear part ---> joints 1–3
        if key == 'w':
            msg.linear.x = self.step
        elif key == 's':
            msg.linear.x = -self.step
        elif key == 'a':
            msg.linear.y = self.step
        elif key == 'd':
            msg.linear.y = -self.step
        elif key == 'q':
            msg.linear.z = self.step
        elif key == 'e':
            msg.linear.z = -self.step

        # Angular part ---> joints 4–6
        elif key == 'i':
            msg.angular.x = self.rot_step
        elif key == 'k':
            msg.angular.x = -self.rot_step
        elif key == 'j':
            msg.angular.y = self.rot_step
        elif key == 'l':
            msg.angular.y = -self.rot_step
        elif key == 'u':
            msg.angular.z = self.rot_step
        elif key == 'o':
            msg.angular.z = -self.rot_step
        else:
            return  

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
