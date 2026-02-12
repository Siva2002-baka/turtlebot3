import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

# Key mapping
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84
LIN_VEL_STEP = 0.01
ANG_VEL_STEP = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    self.linear_velocity += LIN_VEL_STEP
                elif key == 'x':
                    self.linear_velocity -= LIN_VEL_STEP
                elif key == 'a':
                    self.angular_velocity += ANG_VEL_STEP
                elif key == 'd':
                    self.angular_velocity -= ANG_VEL_STEP
                elif key == ' ' or key == 's':
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0
                elif key == '\x03': # CTRL-C
                    break
                
                # Limit velocities
                self.linear_velocity = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, self.linear_velocity))
                self.angular_velocity = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, self.angular_velocity))

                twist = Twist()
                twist.linear.x = self.linear_velocity
                twist.angular.z = self.angular_velocity
                self.publisher_.publish(twist)
                
        except Exception as e:
            print(e)
        finally:
            # Stop robot on exit
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()