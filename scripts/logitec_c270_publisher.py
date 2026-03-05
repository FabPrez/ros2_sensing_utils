#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

import os
import contextlib

@contextlib.contextmanager
def suppress_stderr_fd():
    """
    Temporaneamente reindirizza il file descriptor 2 (stderr) su /dev/null.
    Usalo SOLO attorno alle operazioni che generano i messaggi (es. VideoCapture.open()).
    """
    # apri /dev/null
    devnull = os.open(os.devnull, os.O_WRONLY)
    # duplica fd stderr corrente per poter restaurare dopo
    old_stderr_fd = os.dup(2)
    try:
        # reindirizza stderr -> /dev/null
        os.dup2(devnull, 2)
        yield
    finally:
        # ripristina stderr originale
        os.dup2(old_stderr_fd, 2)
        os.close(old_stderr_fd)
        os.close(devnull)

class WebcamPublisher(Node):
    def __init__(self, device_index: int = None, publish_hz: float = None):
        super().__init__('webcam_publisher')
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
        

        # dichiara parametri: se costruttore fornisce valori usali come default
        default_device = int(device_index) if device_index is not None else 0
        default_hz = float(publish_hz) if publish_hz is not None else 10.0

        self.declare_parameter('device_index', default_device)
        self.declare_parameter('publish_hz', default_hz)

        # leggi i parametri effettivi
        self.device_index = int(self.get_parameter('device_index').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.bridge = CvBridge()

        self.pub = self.create_publisher(
            Image,
            'camera/image_raw',
            qos_profile_sensor_data
        )

        # apri la camera
        
        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)

        # best-effort settaggi (alcuni driver stampano messaggi qui)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # ---- Finestra di Preview Iniziale (3 secondi) ----
        self.get_logger().info(f"WebcamPublisher avviato: preview in corso per il device={self.device_index} (3s)...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            ret, frame = self.cap.read()
            if ret:
                # Mostra anteprima con testo info
                preview = frame.copy()
                cv2.putText(preview, f"Anteprima Device Index: {self.device_index}", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(f"Webcam Preview (Chiude in automatico)", preview)
                cv2.waitKey(30)
                
        try:
            cv2.destroyWindow(f"Webcam Preview (Chiude in automatico)")
        except Exception:
            pass
        # ---------------------------------------------------

        period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f"Preview completata. Pubblicazione su topic a {self.publish_hz}Hz.")

    def timer_callback(self):
        with suppress_stderr_fd():
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
        # NOTE: qui non passiamo device_index/publish_hz: saranno presi dai parametri di runtime
        node = WebcamPublisher()
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
