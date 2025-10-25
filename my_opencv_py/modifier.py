#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')

        # CvBridge instance
        self.bridge = CvBridge()

        # Subscriber to the original camera topic
        self.sub = self.create_subscription(Image, 'camera/image', self.image_callback, 10 )

        # Publisher for the modified image
        self.pub = self.create_publisher(Image, 'camera/image_with_circle', 10)

        # OpenCV window
        cv2.namedWindow("view", cv2.WINDOW_AUTOSIZE)

        self.get_logger().info('Node started: subscribed to camera/image, publishing camera/image_with_circle')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Draw a red circle in the center
            center = (cv_image.shape[1] // 2, cv_image.shape[0] // 2)
            cv2.circle(cv_image, center, 50, (0, 0, 255), 3)

            # Show the image
            cv2.imshow("view", cv_image)
            cv2.waitKey(1)

            # Convert back to ROS Image and publish
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = msg.header  # preserve original header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
