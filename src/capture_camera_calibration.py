#!/usr/bin/env python3
import json
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CalibrationCapture(Node):
    def __init__(self):
        super().__init__("camera_calibration_capture")
        self.camera_info = None
        self.image = None
        self.create_subscription(CameraInfo, "/camera/camera_info", self._info_cb, 10)
        self.create_subscription(Image, "/camera/image", self._image_cb, 10)

    def _info_cb(self, msg):
        if self.camera_info is None:
            self.camera_info = msg

    def _image_cb(self, msg):
        if self.image is None:
            self.image = msg


def scaled_intrinsics(info, image):
    fx = float(info.k[0])
    fy = float(info.k[4])
    cx = float(info.k[2])
    cy = float(info.k[5])

    info_width = int(info.width)
    info_height = int(info.height)
    image_width = int(image.width)
    image_height = int(image.height)

    cx_implies_half_width = cx > 0 and abs(cx - image_width / 2.0) > image_width * 0.10
    cy_implies_half_height = cy > 0 and abs(cy - image_height / 2.0) > image_height * 0.10

    if cx_implies_half_width:
        sx = image_width / (2.0 * cx)
    elif info_width > 0:
        sx = image_width / info_width
    elif cx > 0:
        sx = image_width / (2.0 * cx)
    else:
        sx = 1.0

    if cy_implies_half_height:
        sy = image_height / (2.0 * cy)
    elif info_height > 0:
        sy = image_height / info_height
    elif cy > 0:
        sy = image_height / (2.0 * cy)
    else:
        sy = 1.0

    return {
        "fx": fx * sx,
        "fy": fy * sy,
        "cx": cx * sx,
        "cy": cy * sy,
        "scale_x": sx,
        "scale_y": sy,
        "principal_point_scaled": cx_implies_half_width or cy_implies_half_height,
    }


def main():
    rclpy.init()
    node = CalibrationCapture()

    deadline = time.monotonic() + 10.0
    while rclpy.ok() and time.monotonic() < deadline:
        if node.camera_info is not None and node.image is not None:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.camera_info is None:
        raise SystemExit("No /camera/camera_info received within 10 seconds")
    if node.image is None:
        raise SystemExit("No /camera/image received within 10 seconds")

    info = node.camera_info
    image = node.image
    effective = scaled_intrinsics(info, image)

    data = {
        "source_topics": {
            "camera_info": "/camera/camera_info",
            "image": "/camera/image",
        },
        "camera_info": {
            "width": int(info.width),
            "height": int(info.height),
            "distortion_model": info.distortion_model,
            "d": [float(v) for v in info.d],
            "k": [float(v) for v in info.k],
            "r": [float(v) for v in info.r],
            "p": [float(v) for v in info.p],
        },
        "image": {
            "width": int(image.width),
            "height": int(image.height),
            "encoding": image.encoding,
        },
        "effective_intrinsics": effective,
    }

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    out_dir = os.path.join(project_root, "results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.environ.get(
        "CAMERA_CALIBRATION_JSON",
        os.path.join(out_dir, "camera_calibration.json"),
    )

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"Wrote calibration: {out_path}")
    print(
        "Raw camera_info: "
        f"{info.width}x{info.height}, "
        f"fx={info.k[0]:.3f}, fy={info.k[4]:.3f}, "
        f"cx={info.k[2]:.3f}, cy={info.k[5]:.3f}"
    )
    print(
        "Image: "
        f"{image.width}x{image.height}, encoding={image.encoding}"
    )
    print(
        "Effective intrinsics: "
        f"fx={effective['fx']:.3f}, fy={effective['fy']:.3f}, "
        f"cx={effective['cx']:.3f}, cy={effective['cy']:.3f}"
    )
    print(f"Distortion model: {info.distortion_model!r}")
    print(f"Distortion coefficients: {[float(v) for v in info.d]}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
