#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


# import subprocess, time, cv2

# subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-fmt-video=width=1280,height=720,pixelformat=MJPG'])
# subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-parm=30'])
class WebcamPublisher(Node):
    def __init__(self, device_index: int = 0, publish_hz: float = 10.0):
        super().__init__('webcam_publisher')

        self.bridge = CvBridge()
        self.device_index = device_index

        self.pub = self.create_publisher(
            Image,
            'camera/image_raw',
            qos_profile_sensor_data
        )

        # self.cap = cv2.VideoCapture(self.device_index)
        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(
                f"Impossibile aprire la webcam (indice {self.device_index})."
            )
            raise RuntimeError("Camera not opened")
        
        # 1. forza formato (fondamentale)
        # self.cap.set(
        #     cv2.CAP_PROP_FOURCC,
        #     cv2.VideoWriter_fourcc(*'MJPG')
        # )

        # 2. forza risoluzione
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 3. (best effort) fps
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Riduci buffering
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # (best effort) FPS
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        period = 1.0 / publish_hz if publish_hz > 0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"WebcamPublisher avviato: device={self.device_index} @ {publish_hz}Hz"
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Frame non letto dalla camera.")
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            self.pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(
                f"Errore nella conversione/publish dell'immagine: {e}"
            )

    def destroy(self):
        if hasattr(self, 'timer'):
            self.timer.cancel()

        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = WebcamPublisher(device_index=2, publish_hz=30.0)
        rclpy.spin(node)
    except Exception as e:
        print(f"Errore: {e}", file=sys.stderr)
    finally:
        if node is not None:
            node.get_logger().info("Shutting down webcam publisher...")
            node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
