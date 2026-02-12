import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller_node')
        self.get_logger().info("Turtlebot : Activated")
        # Subscriber to receive commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Controller Node Started. Waiting for commands...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Linear: {msg.linear.x}, Angular: {msg.angular.z}')
        # In a real robot scenario, you might do additional processing here
        # or forward the command to another internal topic if needed.
        # For simple teleop, the bringup node already subscribes to 'cmd_vel'.

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBot3Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()