#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2


class X500MonoCam(Node):
    def __init__(self):
        super().__init__('x500_mono_cam_node')

        # Create a resizable OpenCV window and set an initial size
        cv2.namedWindow('x500 mono cam', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('x500 mono cam', 800, 600)

        # ROS 2 image topic published by ros_gz_image image_bridge
        self.topic = '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'
        self.get_logger().info(f'Subscribing to: {self.topic}')

        # Subscription to the image topic
        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            10
        )

        self.frame_count = 0
        self.logged_encoding = False

    def image_callback(self, msg: Image):
        # Log encoding info only once
        if not self.logged_encoding:
            self.get_logger().info(
                f"Image encoding='{msg.encoding}', size={msg.width}x{msg.height}, step={msg.step}"
            )
            self.logged_encoding = True

        # Determine number of channels from encoding
        if msg.encoding in ('rgb8', 'bgr8'):
            channels = 3
        elif msg.encoding in ('mono8', '8UC1'):
            channels = 1
        else:
            self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
            return

        # Convert flat byte buffer to a NumPy array
        img_array = np.frombuffer(msg.data, dtype=np.uint8)

        # Reshape according to image dimensions and channels
        if channels == 1:
            img = img_array.reshape((msg.height, msg.width))
        else:
            img = img_array.reshape((msg.height, msg.width, channels))

        # Convert RGB to BGR for OpenCV if needed
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Optionally scale down the image before displaying (uncomment if needed)
        # img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

        # Show image in the OpenCV window
        cv2.imshow('x500 mono cam', img)
        cv2.waitKey(1)

        # Save the first frame to disk for debugging
        if self.frame_count == 0:
            cv2.imwrite('x500_first_frame.png', img)
            self.get_logger().info('Saved x500_first_frame.png')

        self.frame_count += 1


def main():
    rclpy.init()
    node = X500MonoCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
