#!/usr/bin/env python3
"""
YOLO + depth-camera localization pipeline.

Detects objects with YOLO (COCO weights) in the RGB stream, reads the
depth value at the detection center, and estimates the ground-plane
distance to the object using the camera pitch angle.  Compares against
Gazebo ground-truth poses every second.
"""
import math
import os
import time
import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from ultralytics import YOLO

# From the URDF:
#   camera joint: xyz="0.0 0.0 0.60185" rpy="0.0 0.4084 0.0"
#   base_link sits 0.048 m above ground (base_footprint joint)
CAMERA_PITCH_RAD = 0.354999969855647  # 20.34 deg downward
CAMERA_Z_FROM_BASE = 0.60185
BASE_LINK_HEIGHT = 0.048
CAMERA_HEIGHT = BASE_LINK_HEIGHT + CAMERA_Z_FROM_BASE  # 0.64985 m
CAN_RADIUS = 0.075  # depth hits the surface; center is one body radius deeper


class DepthLocalizer(Node):
    def __init__(self):
        super().__init__("depth_localizer")

        self.bridge = CvBridge()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        default_model = os.path.join(project_root, "models", "yolo26n.pt")
        model_name = os.environ.get("YOLO_MODEL", default_model)
        self.conf = float(os.environ.get("YOLO_CONF", "0.25"))
        self.target_class = os.environ.get("YOLO_TARGET", "bottle")
        self.target_selection = os.environ.get("YOLO_SELECTION", "center").lower()
        default_calibration = os.path.join(
            project_root, "results", "camera_calibration.json"
        )
        self.calibration_path = os.environ.get(
            "CAMERA_CALIBRATION_JSON", default_calibration
        )
        self.calibration = self._load_camera_calibration()
        default_correction = os.path.join(
            project_root, "results", "error_correction_curves.json"
        )
        self.enable_correction = os.environ.get("ENABLE_ERROR_CORRECTION", "0") == "1"
        self.correction_path = os.environ.get("ERROR_CORRECTION_JSON", default_correction)
        self.correction = self._load_correction() if self.enable_correction else None

        self.get_logger().info(f"Loading YOLO model: {model_name}")
        self.model = YOLO(model_name)
        self.get_logger().info(
            f"YOLO ready  |  target class: {self.target_class!r}"
        )

        self.latest_depth = None
        self.robot_pos = None
        self.target_pos = None
        self.fx = self.fy = self.cx = self.cy = None
        self.distortion_model = ""
        self.distortion_coeffs = []
        self.camera_info_width = None
        self.camera_info_height = None
        self.image_width = None
        self.image_height = None
        self.logged_effective_intrinsics = False
        self.logged_distortion_warning = False
        self.last_report = 0.0

        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._camera_info_cb, 10
        )
        self.create_subscription(
            Image, "/camera/depth_image", self._depth_cb, 10
        )
        self.create_subscription(Image, "/camera/image", self._image_cb, 10)
        # Use a small queue with KEEP_LAST so the localizer always gets the
        # most recent pose rather than processing a backlog of stale messages.
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            TFMessage,
            "/world/litterbot_world/pose/info",
            self._pose_cb,
            pose_qos,
        )

    # ---- callbacks -------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        if self.fx is None:
            if self.calibration:
                intr = self.calibration["effective_intrinsics"]
                self.fx = intr["fx"]
                self.fy = intr["fy"]
                self.cx = intr["cx"]
                self.cy = intr["cy"]
                raw_info = self.calibration.get("camera_info", {})
                self.distortion_model = raw_info.get("distortion_model", "")
                self.distortion_coeffs = raw_info.get("d", [])
            else:
                self.fx = msg.k[0]
                self.fy = msg.k[4]
                self.cx = msg.k[2]
                self.cy = msg.k[5]
                self.distortion_model = msg.distortion_model
                self.distortion_coeffs = list(msg.d)
            self.camera_info_width = msg.width
            self.camera_info_height = msg.height
            self.get_logger().info(
                f"Intrinsics: fx={self.fx:.1f}  fy={self.fy:.1f}  "
                f"cx={self.cx:.1f}  cy={self.cy:.1f}  "
                f"size={self.camera_info_width}x{self.camera_info_height}  "
                f"distortion={self.distortion_model or 'none'}"
            )

    def _depth_cb(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def _read_target_from_file(self):
        """Read target pose from /tmp/litterbot_target_pose.txt (written by move_can.sh)."""
        try:
            with open("/tmp/litterbot_target_pose.txt", "r") as f:
                parts = f.read().strip().split()
            if len(parts) >= 2:
                self.target_pos = np.array([float(parts[0]), float(parts[1]),
                                            float(parts[2]) if len(parts) > 2 else 0.19])
        except (FileNotFoundError, ValueError):
            pass

    def _pose_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            t = tf.transform.translation
            if tf.child_frame_id == "simple_bot":
                self.robot_pos = np.array([t.x, t.y, t.z])
            elif tf.child_frame_id == "target_coke_can":
                # Only use Gazebo pose if no file override exists
                if not os.path.exists("/tmp/litterbot_target_pose.txt"):
                    self.target_pos = np.array([t.x, t.y, t.z])

    def _image_cb(self, msg: Image):
        now = time.monotonic()
        # Read target position from file for instant ground-truth updates
        self._read_target_from_file()
        if now - self.last_report < 0.3:
            return
        if self.latest_depth is None:
            return
        if self.robot_pos is None or self.target_pos is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image_height, self.image_width = frame.shape[:2]
        results = self.model.predict(
            frame, imgsz=640, conf=self.conf, verbose=False
        )
        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            if now - self.last_report > 3.0:
                self.get_logger().info("No detections")
                self.last_report = now
            return

        candidates = []
        for box, cls_idx, conf_val in zip(
            result.boxes.xyxy, result.boxes.cls, result.boxes.conf
        ):
            name = result.names[int(cls_idx)]
            if self.target_class.lower() in name.lower():
                box_np = box.cpu().numpy()
                cu = float((box_np[0] + box_np[2]) / 2)
                cv = float((box_np[1] + box_np[3]) / 2)
                center_error = abs(cu - frame.shape[1] / 2) + abs(cv - frame.shape[0] / 2)
                candidates.append((box_np, float(conf_val), center_error))

        if not candidates:
            return

        if self.target_selection == "confidence":
            best_box, best_conf, _ = max(candidates, key=lambda item: item[1])
        else:
            best_box, best_conf, _ = min(candidates, key=lambda item: item[2])

        u = int((best_box[0] + best_box[2]) / 2)
        v = int((best_box[1] + best_box[3]) / 2)

        depth_z = self._read_depth(u, v)
        if depth_z is None:
            return

        # --- estimated ground-plane distance from pinhole projection -------
        depth_to_center = depth_z + CAN_RADIUS
        est_forward = None
        est_lateral = None
        est_height = None
        if self.fx is not None and self.fy is not None:
            est_forward, est_lateral, est_height = self._project_depth_to_base(
                u, v, depth_to_center
            )
            est_ground_dist = math.sqrt(est_forward * est_forward + est_lateral * est_lateral)
        else:
            est_ground_dist = depth_to_center * math.cos(CAMERA_PITCH_RAD)

        # --- ground truth from Gazebo -------------------------------------
        dx = self.target_pos[0] - self.robot_pos[0]
        dy = self.target_pos[1] - self.robot_pos[1]
        gt_ground_dist = math.sqrt(dx * dx + dy * dy)

        error = est_ground_dist - gt_ground_dist
        pct = (error / gt_ground_dist * 100) if gt_ground_dist > 0.01 else float("nan")
        corrected_ground_dist = None
        corrected_error = None
        corrected_pct = None
        correction_delta = None

        if self.correction:
            correction_delta = self._correction_delta(est_ground_dist, est_lateral)
            corrected_ground_dist = est_ground_dist - correction_delta
            corrected_error = corrected_ground_dist - gt_ground_dist
            corrected_pct = (
                corrected_error / gt_ground_dist * 100
                if gt_ground_dist > 0.01
                else float("nan")
            )

        correction_report = ""
        if corrected_ground_dist is not None:
            correction_report = (
                f"Correction applied:     {correction_delta:+.3f} m\n"
                f"Corrected distance:     {corrected_ground_dist:.3f} m\n"
                f"Corrected error:        {corrected_error:+.3f} m  "
                f"({corrected_pct:+.1f} %)\n"
            )

        lateral_report = ""
        if est_lateral is not None:
            lateral_report = (
                f"Est. forward offset:    {est_forward:+.3f} m\n"
                f"Est. lateral offset:    {est_lateral:+.3f} m  "
                "(pinhole projection)\n"
                f"Est. height offset:     {est_height:+.3f} m\n"
            )

        report = (
            "\n"
            "============ DEPTH LOCALIZATION REPORT ============\n"
            f"Detection: {self.target_class} ({best_conf:.2f})  "
            f"pixel ({u}, {v})\n"
            f"Depth (surface):        {depth_z:.3f} m\n"
            f"Depth (center):         {depth_to_center:.3f} m  "
            f"(+{CAN_RADIUS:.2f} m radius)\n"
            f"Camera pitch:           {math.degrees(CAMERA_PITCH_RAD):.1f} deg\n"
            f"\n"
            f"Est. ground distance:   {est_ground_dist:.3f} m  "
            f"(pinhole projection)\n"
            f"{lateral_report}"
            f"Ground-truth distance:  {gt_ground_dist:.3f} m  "
            f"(Gazebo XY plane)\n"
            f"Error:                  {error:+.3f} m  ({pct:+.1f} %)\n"
            f"{correction_report}"
            f"\n"
            f"Gazebo poses:\n"
            f"  Robot:  ({self.robot_pos[0]:.3f}, "
            f"{self.robot_pos[1]:.3f}, {self.robot_pos[2]:.3f})\n"
            f"  Target: ({self.target_pos[0]:.3f}, "
            f"{self.target_pos[1]:.3f}, {self.target_pos[2]:.3f})\n"
            "==================================================="
        )
        self.last_report = now
        self.get_logger().info(report)

    # ---- helpers ---------------------------------------------------------

    def _load_correction(self):
        try:
            with open(self.correction_path, encoding="utf-8") as f:
                correction = json.load(f)
        except FileNotFoundError:
            self.get_logger().warn(
                f"Error correction requested, but file was not found: "
                f"{self.correction_path}"
            )
            return None

        self.get_logger().info(f"Loaded error correction: {self.correction_path}")
        return correction

    def _load_camera_calibration(self):
        try:
            with open(self.calibration_path, encoding="utf-8") as f:
                calibration = json.load(f)
        except FileNotFoundError:
            return None

        self.get_logger().info(f"Loaded camera calibration: {self.calibration_path}")
        intr = calibration.get("effective_intrinsics", {})
        image = calibration.get("image", {})
        width = image.get("width")
        height = image.get("height")
        if width and intr.get("cx") and abs(intr["cx"] - width / 2.0) > width * 0.10:
            scale = width / (2.0 * intr["cx"])
            intr["fx"] *= scale
            intr["cx"] *= scale
            intr["scale_x"] = intr.get("scale_x", 1.0) * scale
            intr["principal_point_scaled"] = True
        if height and intr.get("cy") and abs(intr["cy"] - height / 2.0) > height * 0.10:
            scale = height / (2.0 * intr["cy"])
            intr["fy"] *= scale
            intr["cy"] *= scale
            intr["scale_y"] = intr.get("scale_y", 1.0) * scale
            intr["principal_point_scaled"] = True
        return calibration

    def _eval_poly(self, coeffs, value):
        result = 0.0
        for coeff in coeffs:
            result = result * value + float(coeff)
        return result

    def _project_depth_to_base(self, u, v, depth_z):
        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy

        if not self.logged_effective_intrinsics:
            self.get_logger().info(
                f"Effective intrinsics for image {self.image_width}x{self.image_height}: "
                f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
            )
            self.logged_effective_intrinsics = True

        u, v = self._undistort_pixel(u, v, fx, fy, cx, cy)
        x_opt = (u - cx) * depth_z / fx
        y_opt = (v - cy) * depth_z / fy
        z_opt = depth_z

        # Optical frame: x=right, y=down, z=forward.
        # Base frame: x=forward, y=left, z=up.
        # First map an unpitched optical camera into the base convention,
        # then rotate by the camera's downward pitch about the base y-axis.
        x_unpitched = z_opt
        y_base = -x_opt
        z_unpitched = -y_opt

        c = math.cos(CAMERA_PITCH_RAD)
        s = math.sin(CAMERA_PITCH_RAD)
        x_base = x_unpitched * c + z_unpitched * s
        z_base = CAMERA_HEIGHT - x_unpitched * s + z_unpitched * c

        return x_base, y_base, z_base

    def _undistort_pixel(self, u, v, fx, fy, cx, cy):
        coeffs = np.array(self.distortion_coeffs, dtype=np.float64)
        if coeffs.size == 0 or not np.any(np.abs(coeffs) > 1e-12):
            return float(u), float(v)
        if self.distortion_model not in ("plumb_bob", "rational_polynomial"):
            if not self.logged_distortion_warning:
                self.get_logger().warn(
                    f"Unsupported distortion model {self.distortion_model!r}; "
                    "using raw pixel coordinates"
                )
                self.logged_distortion_warning = True
            return float(u), float(v)

        camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        point = np.array([[[float(u), float(v)]]], dtype=np.float64)
        undistorted = cv2.undistortPoints(
            point,
            camera_matrix,
            coeffs,
            P=camera_matrix,
        )
        return float(undistorted[0, 0, 0]), float(undistorted[0, 0, 1])

    def _correction_delta(self, est_ground_dist, est_lateral):
        longitudinal = self.correction.get("longitudinal")
        lateral = self.correction.get("lateral")

        if lateral and est_lateral is not None and abs(est_lateral) > 0.25:
            coeffs = lateral["coefficients"]
            return self._eval_poly(coeffs, est_lateral)

        if longitudinal:
            coeffs = longitudinal["coefficients"]
            return self._eval_poly(coeffs, est_ground_dist)

        return 0.0

    def _read_depth(self, u: int, v: int):
        h, w = self.latest_depth.shape[:2]
        if 0 <= u < w and 0 <= v < h:
            d = float(self.latest_depth[v, u])
            if math.isfinite(d) and d > 0:
                return d

        for r in (1, 2, 3, 5):
            samples = []
            for dv in range(-r, r + 1):
                for du in range(-r, r + 1):
                    sv, su = v + dv, u + du
                    if 0 <= su < w and 0 <= sv < h:
                        d = float(self.latest_depth[sv, su])
                        if math.isfinite(d) and d > 0:
                            samples.append(d)
            if samples:
                return float(np.median(samples))
        return None


def main():
    rclpy.init()
    node = DepthLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
