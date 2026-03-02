import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to obstacle direction
        self.sub = self.create_subscription(
            String,
            '/obstacle_direction',
            self.obstacle_callback,
            10
        )

        # Default velocities
        self.forward_speed = 1.5
        self.arc_turn = 0.1

        # Current obstacle status
        self.current_obstacle = 'clear'

        # Timer to publish commands
        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10 Hz

    def obstacle_callback(self, msg: String):
        self.current_obstacle = msg.data
        self.get_logger().info(f"Received obstacle: {self.current_obstacle}")

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.forward_speed
        cmd.angular.z = self.arc_turn  # default arc

        # Modify based on obstacle
        if self.current_obstacle == 'left':
            cmd.angular.z = -0.5  # turn right
        elif self.current_obstacle == 'right':
            cmd.angular.z = 0.5   # turn left
        elif self.current_obstacle == 'straight':
            cmd.angular.z = 0.0
            cmd.linear.x = 10.    # slow down

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()