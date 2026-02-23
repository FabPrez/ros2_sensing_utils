#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CharucoCalibrator(Node):
    def __init__(self):
        super().__init__("charuco_intrinsic_calibrator")

        # --- Parametri (modificali da CLI se vuoi) ---
        self.declare_parameter("image_topic", "/camera/image_raw")

        # Dal tuo generator: Rows=8, Columns=11  -> squares_y=8, squares_x=11
        self.declare_parameter("squares_x", 11)   # colonne (orizzontale)
        self.declare_parameter("squares_y", 8)    # righe (verticale)

        # QUADRATO: 18 cm = 0.18 m (se invece 18 mm -> 0.018)
        self.declare_parameter("square_length", 0.018)  # [m]

        # Marker size: nel tuo screenshot era 11mm vs square 15mm -> ratio 11/15
        # Se hai stampato scalando tutto, il rapporto resta lo stesso.
        self.declare_parameter("marker_length_ratio", 11.0 / 15.0)
        # In alternativa puoi settare direttamente marker_length da parametro e ignorare ratio
        self.declare_parameter("marker_length", -1.0)  # [m] se >0, sovrascrive ratio

        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("min_corners", 12)  # min charuco corners per accettare il frame
        self.declare_parameter("required_frames", 40)  # quanti frame buoni per calibrare

        self.declare_parameter("show_debug", True)
        self.declare_parameter("output_yaml", "camera_intrinsics.yaml")
        
        self.declare_parameter("min_interval_sec", 0.7)  # tempo minimo tra frame accettati
        self.min_interval = float(self.get_parameter("min_interval_sec").value)
        self.last_accept_time = None

        # --- Lettura parametri ---
        self.image_topic = self.get_parameter("image_topic").value
        self.squares_x = int(self.get_parameter("squares_x").value)
        self.squares_y = int(self.get_parameter("squares_y").value)

        self.square_length = float(self.get_parameter("square_length").value)
        marker_length_param = float(self.get_parameter("marker_length").value)
        ratio = float(self.get_parameter("marker_length_ratio").value)

        if marker_length_param > 0:
            self.marker_length = marker_length_param
        else:
            self.marker_length = self.square_length * ratio

        dict_name = self.get_parameter("aruco_dictionary").value
        self.min_corners = int(self.get_parameter("min_corners").value)
        self.required_frames = int(self.get_parameter("required_frames").value)

        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.output_yaml = self.get_parameter("output_yaml").value
        
        self.aruco_dict = self._make_dictionary(dict_name)
        
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.adaptiveThreshWinSizeMin = 5
        self.aruco_params.adaptiveThreshWinSizeMax = 45
        self.aruco_params.adaptiveThreshWinSizeStep = 10

        self.charuco_params = cv2.aruco.CharucoParameters()
        self.charuco_params.tryRefineMarkers = True


        # Board ChArUco
        self.board = cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y),
            self.square_length,
            self.marker_length,
            self.aruco_dict
        )

        self.charuco_detector = cv2.aruco.CharucoDetector(
            self.board,
            self.charuco_params,
            self.aruco_params
        )


        # Accumulator
        self.all_charuco_corners = []
        self.all_charuco_ids = []
        self.image_size = None

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Subscribed to {self.image_topic}\n"
            f"Board: squares_x={self.squares_x}, squares_y={self.squares_y}, "
            f"square_length={self.square_length}m, marker_length={self.marker_length}m\n"
            f"Need {self.required_frames} good frames (min_corners={self.min_corners})"
        )

    def _make_dictionary(self, name: str):
        # Supporta i nomi più comuni
        mapping = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        }
        if name not in mapping:
            raise ValueError(f"Unknown aruco_dictionary '{name}'. Try e.g. DICT_4X4_50")
        return cv2.aruco.getPredefinedDictionary(mapping[name])

    def on_image(self, msg: Image):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # salva image size una volta sola
        if self.image_size is None:
            self.image_size = (gray.shape[1], gray.shape[0])  # (w, h)

        debug = frame.copy()

        # Detect ChArUco board (marker + charuco) direttamente
        # Ritorna: charucoCorners, charucoIds, markerCorners, markerIds
        try:
            charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.detectBoard(gray)
        except cv2.error as e:
            self.get_logger().error(f"OpenCV error in detectBoard(): {e}")
            return

        # Disegna markers se trovati
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug, marker_corners, marker_ids)
            
        if marker_ids is not None:
            mi, ma = int(marker_ids.min()), int(marker_ids.max())
            self.get_logger().info(f"Marker id range: {mi}..{ma}  count={len(marker_ids)}")

        # Se non ci sono charuco corners sufficienti, log e debug
        if charuco_ids is None or len(charuco_ids) < self.min_corners:
            if self.show_debug:
                if marker_ids is None or len(marker_ids) == 0:
                    self.get_logger().info("No ArUco markers detected.")
                else:
                    self.get_logger().info(
                        f"Markers detected ({len(marker_ids)}) but not enough Charuco corners "
                        f"({0 if charuco_ids is None else len(charuco_ids)}/{self.min_corners})."
                    )

            if self.show_debug:
                cv2.imshow("charuco_calibration_debug", debug)
                cv2.waitKey(1)
            return
        
        now = self.get_clock().now().nanoseconds * 1e-9  # tempo in secondi

        if self.last_accept_time is not None:
            if (now - self.last_accept_time) < self.min_interval:
                # troppo presto, ignora questo frame
                return

        self.last_accept_time = now

        # Frame buono: salva osservazione
        self.all_charuco_corners.append(charuco_corners)
        self.all_charuco_ids.append(charuco_ids)

        # Disegna charuco corners
        cv2.aruco.drawDetectedCornersCharuco(debug, charuco_corners, charuco_ids)

        self.get_logger().info(
            f"[{len(self.all_charuco_corners)}/{self.required_frames}] "
            f"Accepted frame with {len(charuco_ids)} Charuco corners"
        )

        # Se abbiamo abbastanza frame, calibra e chiudi
        if len(self.all_charuco_corners) >= self.required_frames:
            self.calibrate_and_save()
            rclpy.shutdown()
            return

        # Debug view
        if self.show_debug:
            cv2.imshow("charuco_calibration_debug", debug)
            cv2.waitKey(1)

    def calibrate_and_save(self):
        self.get_logger().info("Calibrating...")

        if self.image_size is None:
            raise RuntimeError("image_size is None: no frames received?")

        # Converti raccolta ChArUco -> (objectPoints, imagePoints) per calibrateCamera
        objpoints = []  # lista di array Nx3
        imgpoints = []  # lista di array Nx2

        used = 0
        for corners, ids in zip(self.all_charuco_corners, self.all_charuco_ids):
            if ids is None or len(ids) == 0:
                continue

            # matchImagePoints ritorna objectPoints (3D) e imagePoints (2D) corrispondenti
            obj, img = self.board.matchImagePoints(corners, ids)

            # In alcune build img può essere Nx1x2: normalizziamo a Nx2
            if img is None or obj is None:
                continue

            obj = np.asarray(obj, dtype=np.float32)
            img = np.asarray(img, dtype=np.float32)

            if img.ndim == 3 and img.shape[1] == 1 and img.shape[2] == 2:
                img = img.reshape(-1, 2)

            if obj.ndim == 3 and obj.shape[1] == 1 and obj.shape[2] == 3:
                obj = obj.reshape(-1, 3)

            # Serve almeno qualche punto
            if len(obj) < 6 or len(img) < 6:
                continue

            objpoints.append(obj)
            imgpoints.append(img)
            used += 1

        if used < 5:
            raise RuntimeError(f"Not enough valid frames for calibration (used={used}).")

        # Calibrazione classica OpenCV
        # distCoeffs: default 5 parametri (k1,k2,p1,p2,k3) -> plumb_bob
        flags = 0
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=objpoints,
            imagePoints=imgpoints,
            imageSize=self.image_size,
            cameraMatrix=None,
            distCoeffs=None,
            flags=flags,
            criteria=criteria
        )

        self.get_logger().info(f"Calibration done. RMS reprojection error = {rms:.6f}")
        self.get_logger().info(f"K=\n{K}")
        self.get_logger().info(f"D=\n{D.ravel()}")

        data = {
            "image_width": int(self.image_size[0]),
            "image_height": int(self.image_size[1]),
            "camera_name": "camera",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": K.flatten().tolist()
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(D.size),
                "data": D.flatten().tolist()
            },
            "charuco_board": {
                "squares_x": self.squares_x,
                "squares_y": self.squares_y,
                "square_length_m": float(self.square_length),
                "marker_length_m": float(self.marker_length),
            },
            "frames_used": used
        }

        out_path = os.path.abspath(self.output_yaml)
        with open(out_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        self.get_logger().info(f"Saved intrinsics to: {out_path}")

def main():
    rclpy.init()
    node = CharucoCalibrator()
    try:
        rclpy.spin(node)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()