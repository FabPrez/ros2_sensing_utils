#!/usr/bin/env python3
"""
Publishes ONLY:
  1) /hand/pose             (geometry_msgs/PoseStamped)  -> pose of ONE chosen marker (id0 or id1) in camera frame
  2) /hand/fingers_position (sensor_msgs/JointState)     -> position[0] = aperture in [0..1]
OPTIONAL:
  - enable_gui:=True -> OpenCV window with debug overlay.
"""

import os
import time
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


# ============================================================
# ================== NON-ROS CONFIG (EDIT) ===================
# ============================================================

# Choose which marker pose to output on /hand/pose
POSE_OUTPUT_MARKER_ID = 1  # 0 or 1

# -------- Basic filters (lightweight) --------
USE_UNDISTORT_FOR_DETECTION = True
USE_SOLVEPNP_REFINE_LM = True

USE_REPROJECTION_OUTLIER_REJECTION = True
REPROJ_ERR_THRESH_PX = 2.5

# Light smoothing (optional). Keep small alpha to avoid lag.
USE_TVEC_EMA_FILTER = False
EMA_ALPHA_TVEC = 0.12   # @20Hz: 0.10..0.18

USE_QUAT_SLERP_FILTER = False
ALPHA_ROT = 0.12        # @20Hz: 0.10..0.18

# -------- Robust fingers detection (anti false positives) --------
FINGERS_METRIC = "3D"   # "3D" or "X"
FINGERS_OPEN_THRESH = 0.178
FINGERS_CLOSE_THRESH = 0.170  # must be < open
FINGERS_REQUIRED_CONSECUTIVE = 1

FINGERS_REQUIRE_LOW_REPROJ_ERR = True
FINGERS_MAX_REPROJ_ERR_PX = 2.5

# Aperture output mapping for robot
OUTPUT_MIN = 0.0
OUTPUT_MAX = 1.0

USE_CONTINUOUS_APERTURE = True
DIST_CLOSED = 0.140   # meters
DIST_OPEN = 0.180     # meters

# Rotation to align camera frame with robot
ROTATE_CAMERA_Z_90 = True

# ============================================================


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return (q / n).astype(np.float64)


def quat_dot(q1: np.ndarray, q2: np.ndarray) -> float:
    return float(np.dot(q1, q2))


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    dot = quat_dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)

    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return quat_normalize(q)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)

    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    q = (s0 * q0) + (s1 * q1)
    return quat_normalize(q)


def rmat_to_quat(R: np.ndarray) -> np.ndarray:
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return quat_normalize(np.array([qx, qy, qz, qw], dtype=np.float64))


class MinimalTwoAruco(Node):
    def __init__(self):
        super().__init__("aruco_hand_pose_and_fingers")

        # -------------------------
        # ROS params (I/O only)
        # -------------------------
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("camera_yaml", "/home/fabioprez/projects/fruit_ws/camera_intrinsics.yaml")

        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("target_ids", [0, 1])
        self.declare_parameter("marker_length", 0.035)

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("log_throttle_sec", 1.0)

        # GUI debug switch
        self.declare_parameter("enable_gui", True)
        self.declare_parameter("gui_rate_hz", 10.0)

        # Outputs topics
        self.declare_parameter("pose_topic", "/hand/pose")
        self.declare_parameter("fingers_topic", "/hand/fingers_position")
        self.declare_parameter("fingers_joint_name", "gripper_aperture")

        # Read ROS params
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.camera_yaml = os.path.expanduser(self.get_parameter("camera_yaml").value)

        self.dict_name = self.get_parameter("dictionary").value
        self.target_ids = [int(x) for x in self.get_parameter("target_ids").value]
        self.marker_length = float(self.get_parameter("marker_length").value)

        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.log_throttle_sec = float(self.get_parameter("log_throttle_sec").value)

        self.enable_gui = bool(self.get_parameter("enable_gui").value)
        self.gui_rate_hz = float(self.get_parameter("gui_rate_hz").value)
        self._last_gui_t = 0.0  # wall clock timestamp (time.time())

        self.pose_topic = self.get_parameter("pose_topic").value
        self.fingers_topic = self.get_parameter("fingers_topic").value
        self.fingers_joint_name = self.get_parameter("fingers_joint_name").value

        # Intrinsics
        self.K, self.D = self._load_camera_yaml(self.camera_yaml)

        # ArUco
        self.aruco_dict = self._make_dictionary(self.dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 3
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.01
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Filtered pose state per marker
        self.filt_t = {}   # id -> np.array(3,)
        self.filt_q = {}   # id -> np.array(4,)
        self.raw_err = {}  # id -> float or None

        # Keep raw rvec/tvec for GUI axes drawing (not filtered)
        self.raw_rvec = {}  # id -> np.array(3,)
        self.raw_tvec = {}  # id -> np.array(3,)

        # Fingers robust state
        self.fingers_state = None
        self._open_votes = 0
        self._close_votes = 0
        self._last_dist = None
        self._last_aperture = None  # last numeric aperture [0..1]

        # ROS I/O
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        # Outputs
        self.pub_pose = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.pub_fingers = self.create_publisher(JointState, self.fingers_topic, 10)

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            "Aruco hand node started\n"
            f"  topic  -> {self.image_topic}\n"
            f"  pose   -> {self.pose_topic} (marker id={POSE_OUTPUT_MARKER_ID})\n"
            f"  fingers-> {self.fingers_topic} (JointState.position[0] in [0..1])\n"
            f"  enable_gui={self.enable_gui} (gui_rate_hz={self.gui_rate_hz})"
        )

        # Watchdog: avvisa se la camera non manda messaggi
        self._received_first_image = False
        self._watchdog_timer = self.create_timer(5.0, self._camera_watchdog)

    def _camera_watchdog(self):
        """Chiamato ogni 5 s. Avvisa se non arrivano messaggi dalla camera."""
        if not self._received_first_image:
            self.get_logger().warn(
                f"[WATCHDOG] Nessun messaggio ricevuto su '{self.image_topic}'.\n"
                f"           Assicurati che la camera sia attiva e che il topic sia corretto."
            )

    def _make_dictionary(self, name: str):
        mapping = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        }
        if name not in mapping:
            raise ValueError(f"Unknown dictionary '{name}'")
        return cv2.aruco.getPredefinedDictionary(mapping[name])

    def _load_camera_yaml(self, path: str):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1, 1)
        return K, D

    def _object_points_marker(self):
        L = self.marker_length
        return np.array([
            [-L/2,  L/2, 0.0],
            [ L/2,  L/2, 0.0],
            [ L/2, -L/2, 0.0],
            [-L/2, -L/2, 0.0],
        ], dtype=np.float64)

    def _reproj_error_px(self, objp, imgp, rvec, tvec, K, D_or_none):
        proj, _ = cv2.projectPoints(objp, rvec.reshape(3, 1), tvec.reshape(3, 1), K, D_or_none)
        proj = proj.reshape(-1, 2)
        imgp2 = imgp.reshape(-1, 2)
        return float(np.linalg.norm(proj - imgp2, axis=1).mean())

    def _pose_to_msg(self, t: np.ndarray, q: np.ndarray, stamp, frame_id: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(t[0])
        msg.pose.position.y = float(t[1])
        msg.pose.position.z = float(t[2])
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        return msg

    def _publish_tf(self, parent: str, child: str, t: np.ndarray, q: np.ndarray, stamp):
        if not self.publish_tf:
            return
        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = parent
        tfm.child_frame_id = child
        tfm.transform.translation.x = float(t[0])
        tfm.transform.translation.y = float(t[1])
        tfm.transform.translation.z = float(t[2])
        tfm.transform.rotation.x = float(q[0])
        tfm.transform.rotation.y = float(q[1])
        tfm.transform.rotation.z = float(q[2])
        tfm.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tfm)

    def _estimate_pose(self, imgp_4x2: np.ndarray, use_dist: bool):
        objp = self._object_points_marker()
        imgp = imgp_4x2.reshape(4, 2).astype(np.float64)
        dist = self.D if use_dist else None

        ok, rvec, tvec = cv2.solvePnP(
            objectPoints=objp,
            imagePoints=imgp,
            cameraMatrix=self.K,
            distCoeffs=dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            return None, None, None, None, None

        rvec = rvec.reshape(3)
        tvec = tvec.reshape(3)

        if USE_SOLVEPNP_REFINE_LM and hasattr(cv2, "solvePnPRefineLM"):
            try:
                rvec, tvec = cv2.solvePnPRefineLM(objp, imgp, self.K, dist, rvec, tvec)
                rvec = np.asarray(rvec).reshape(3)
                tvec = np.asarray(tvec).reshape(3)
            except cv2.error:
                pass

        R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        
        if ROTATE_CAMERA_Z_90:
            # Rotate +90 around Z: X_new = Y_old, Y_new = -X_old, Z_new = Z_old
            R_cam_rot = np.array([
                [ 0.0,  1.0, 0.0],
                [-1.0,  0.0, 0.0],
                [ 0.0,  0.0, 1.0]
            ], dtype=np.float64)
            tvec_out = R_cam_rot @ tvec
            R_out = R_cam_rot @ R
        else:
            tvec_out = tvec.copy()
            R_out = R
            
        q_out = rmat_to_quat(R_out)

        if USE_REPROJECTION_OUTLIER_REJECTION:
            err = self._reproj_error_px(objp, imgp, rvec, tvec, self.K, dist)
            if err > REPROJ_ERR_THRESH_PX:
                return None, None, None, None, None
        else:
            err = None

        return rvec, tvec, tvec_out, q_out, err

    def _update_filtered(self, marker_id: int, t_raw: np.ndarray, q_raw: np.ndarray):
        if marker_id not in self.filt_t:
            self.filt_t[marker_id] = t_raw.copy()
            self.filt_q[marker_id] = q_raw.copy()
            return

        if USE_TVEC_EMA_FILTER:
            a = EMA_ALPHA_TVEC
            self.filt_t[marker_id] = (1.0 - a) * self.filt_t[marker_id] + a * t_raw
        else:
            self.filt_t[marker_id] = t_raw.copy()

        if USE_QUAT_SLERP_FILTER:
            self.filt_q[marker_id] = quat_slerp(self.filt_q[marker_id], q_raw, ALPHA_ROT)
        else:
            self.filt_q[marker_id] = q_raw.copy()

    def _fingers_update(self, t0: np.ndarray, t1: np.ndarray, err0, err1):
        if FINGERS_REQUIRE_LOW_REPROJ_ERR:
            if (err0 is None) or (err1 is None):
                return None, None
            if (err0 > FINGERS_MAX_REPROJ_ERR_PX) or (err1 > FINGERS_MAX_REPROJ_ERR_PX):
                return None, None

        if FINGERS_METRIC == "X":
            d = abs(float(t0[0]) - float(t1[0]))
            open_th, close_th = FINGERS_OPEN_THRESH, FINGERS_CLOSE_THRESH
        else:
            d = float(np.linalg.norm(t0 - t1))
            open_th, close_th = FINGERS_OPEN_THRESH, FINGERS_CLOSE_THRESH

        if self.fingers_state is None:
            self.fingers_state = "OPEN" if d > open_th else "CLOSED"
            self._open_votes = 0
            self._close_votes = 0
            return self.fingers_state, d

        if self.fingers_state == "CLOSED":
            if d > open_th:
                self._open_votes += 1
                self._close_votes = 0
                if self._open_votes >= FINGERS_REQUIRED_CONSECUTIVE:
                    self.fingers_state = "OPEN"
                    self._open_votes = 0
            else:
                self._open_votes = 0
        else:
            if d < close_th:
                self._close_votes += 1
                self._open_votes = 0
                if self._close_votes >= FINGERS_REQUIRED_CONSECUTIVE:
                    self.fingers_state = "CLOSED"
                    self._close_votes = 0
            else:
                self._close_votes = 0

        return self.fingers_state, d

    def _map_aperture(self, state: str, dist: float) -> float:
        if not USE_CONTINUOUS_APERTURE:
            return OUTPUT_MAX if state == "OPEN" else OUTPUT_MIN

        if dist is None:
            return OUTPUT_MIN

        if dist <= DIST_CLOSED:
            u = 0.0
        elif dist >= DIST_OPEN:
            u = 1.0
        else:
            u = (dist - DIST_CLOSED) / max(1e-9, (DIST_OPEN - DIST_CLOSED))

        return float(OUTPUT_MIN + u * (OUTPUT_MAX - OUTPUT_MIN))

    def _should_draw_gui_now(self) -> bool:
        if not self.enable_gui:
            return False
        if self.gui_rate_hz <= 0:
            return True
        # Usa wall clock (time.time()) per evitare problemi col clock ROS all'avvio
        now = time.time()
        dt = now - self._last_gui_t
        if dt >= (1.0 / self.gui_rate_hz):
            self._last_gui_t = now
            return True
        return False

    def _publish_fingers_jointstate(self, stamp, raw_dist: float):
        js = JointState()
        js.header.stamp = stamp
        # frame_id non strettamente necessario, ma puoi metterlo se vuoi
        # js.header.frame_id = self.camera_frame
        js.name = [self.fingers_joint_name]
        js.position = [float(raw_dist)]
        self.pub_fingers.publish(js)

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if USE_UNDISTORT_FOR_DETECTION:
            gray_det = cv2.undistort(gray, self.K, self.D)
            use_dist_in_pnp = False
        else:
            gray_det = gray
            use_dist_in_pnp = True

        corners_list, ids, _ = self.detector.detectMarkers(gray_det)

        got = {}   # id -> (t_raw, q_raw, err)
        filt = {}  # id -> (t_f, q_f)

        draw_gui = self._should_draw_gui_now()
        debug = frame.copy() if draw_gui else None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()

            if draw_gui:
                cv2.aruco.drawDetectedMarkers(debug, corners_list, ids)

            for i, mid in enumerate(ids_flat):
                mid = int(mid)
                if mid not in self.target_ids:
                    continue

                imgp = np.array(corners_list[i]).reshape(4, 2)

                rvec, tvec, tvec_out, q_out, err = self._estimate_pose(imgp, use_dist=use_dist_in_pnp)
                if tvec is None:
                    continue

                self.raw_rvec[mid] = rvec
                self.raw_tvec[mid] = tvec
                self.raw_err[mid] = err

                got[mid] = (tvec_out, q_out, err)

                self._update_filtered(mid, tvec_out, q_out)
                filt[mid] = (self.filt_t[mid], self.filt_q[mid])

                self._publish_tf(self.camera_frame, f"aruco_{mid}", self.filt_t[mid], self.filt_q[mid], msg.header.stamp)

                if draw_gui:
                    dist_for_axes = None if not use_dist_in_pnp else self.D
                    cv2.drawFrameAxes(debug, self.K, dist_for_axes, rvec.reshape(3, 1), tvec.reshape(3, 1), self.marker_length * 0.5)

                    px, py = int(imgp[0, 0]), int(imgp[0, 1])
                    s = f"id {mid}  x={float(tvec_out[0]):.3f}m y={float(tvec_out[1]):.3f}m z={float(tvec_out[2]):.3f}m"
                    if err is not None:
                        s += f" err={err:.2f}px"
                    cv2.putText(debug, s, (px, max(20, py - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # Fingers update (needs both markers 0 and 1)
        if (0 in got) and (1 in got) and (0 in filt) and (1 in filt):
            t0 = filt[0][0]
            t1 = filt[1][0]
            err0 = got[0][2]
            err1 = got[1][2]

            state, dist_val = self._fingers_update(t0, t1, err0, err1)
            if state is not None:
                self._last_dist = dist_val
                self._last_aperture = dist_val

                # Publish fingers as JointState on /hand/fingers_position
                self._publish_fingers_jointstate(msg.header.stamp, self._last_aperture)

                self.get_logger().info(
                    f"FINGERS: {state} dist={dist_val:.4f}m aperture={self._last_aperture:.3f}",
                    throttle_duration_sec=self.log_throttle_sec
                )

                if draw_gui:
                    cv2.putText(debug,
                                f"FINGERS: {state} dist={dist_val:.3f}m ap={self._last_aperture:.2f}",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
            else:
                self._publish_fingers_jointstate(msg.header.stamp, float('nan'))
        else:
            self._publish_fingers_jointstate(msg.header.stamp, float('nan'))

        # Publish pose of selected marker on /hand/pose
        out_id = int(POSE_OUTPUT_MARKER_ID)
        if out_id in filt:
            t, q = filt[out_id]
            self.pub_pose.publish(self._pose_to_msg(t, q, msg.header.stamp, self.camera_frame))
        else:
            t_nan = np.array([np.nan, np.nan, np.nan])
            q_nan = np.array([np.nan, np.nan, np.nan, np.nan])
            self.pub_pose.publish(self._pose_to_msg(t_nan, q_nan, msg.header.stamp, self.camera_frame))

        # GUI window (solo se enable_gui e rate lo permette)
        if draw_gui and debug is not None:
            cv2.imshow("aruco_debug", debug)
            cv2.waitKey(1)

        # Prima immagine ricevuta: disattiva watchdog
        if not self._received_first_image:
            self._received_first_image = True
            self._watchdog_timer.cancel()
            self.get_logger().info(
                f"[OK] Prima immagine ricevuta da '{self.image_topic}' — tracking attivo."
            )


def main():
    rclpy.init()
    node = MinimalTwoAruco()
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