import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Subscribe to depth camera
        self.subscription = self.create_subscription(
            Image,
            '/model/robot/link/camera/depth/image',  # your depth topic
            self.depth_callback,
            10
        )
        self.bridge = CvBridge()

        # Publish obstacle direction
        self.obstacle_pub = self.create_publisher(String, '/obstacle_direction', 10)

        # Depth threshold
        self.block_threshold = 1.0  # meters

    def depth_callback(self, msg: Image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        depth_image = np.nan_to_num(depth_image)
        depth_image = np.clip(depth_image, 0.0, 10.0)
        h, w = depth_image.shape

        # Split left, center, right
        left = depth_image[:, :w//3]
        center = depth_image[:, w//3:2*w//3]
        right = depth_image[:, 2*w//3:]

        left_block = np.sum(left < self.block_threshold)
        center_block = np.sum(center < self.block_threshold)
        right_block = np.sum(right < self.block_threshold)

        # Decide obstacle position
        if center_block > max(left_block, right_block):
            direction = 'straight'
        elif left_block > right_block:
            direction = 'left'
        elif right_block > left_block:
            direction = 'right'
        else:
            direction = 'clear'

        # Publish
        msg_out = String()
        msg_out.data = direction
        self.obstacle_pub.publish(msg_out)
        self.get_logger().info(f"Obstacle detected: {direction}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()