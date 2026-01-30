import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DepthCameraSubscriber(Node):
    def __init__(self):
        super().__init__('depth_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/model/robot/link/camera/depth/image',  # ROS2 topic bridged from Ignition
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Image):
        self.get_logger().info(f'Received depth image: width={msg.width}, height={msg.height}, encoding={msg.encoding}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()