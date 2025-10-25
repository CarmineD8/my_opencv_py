#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2


class CompressedImageProcessor(Node):
    def __init__(self):
        super().__init__('compressed_image_processor')

        # CvBridge for conversions
        self.bridge = CvBridge()

        # Subscriber to compressed input image
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10
        )

        # Publishers for processed images
        self.pub_compressed = self.create_publisher(CompressedImage, '/camera/image_with_circle/compressed', 10)

        # OpenCV window
        cv2.namedWindow('view', cv2.WINDOW_AUTOSIZE)
        self.get_logger().info('Subscribed to /camera/image/compressed')
        self.get_logger().info('Publishing to /camera/image_with_circle/compressed')

    def image_callback(self, msg):
        try:
            # Convert incoming compressed image to OpenCV
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Draw a red circle in the center
            center = (cv_image.shape[1] // 2, cv_image.shape[0] // 2)
            cv2.circle(cv_image, center, 50, (0, 0, 255), 3)

            # Display the image
            cv2.imshow('view', cv_image)
            cv2.waitKey(1)

            # --- Publish compressed image ---
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image, dst_format='jpeg')
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = 'camera_frame'
            self.pub_compressed.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
