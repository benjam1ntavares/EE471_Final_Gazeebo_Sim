#!/usr/bin/env python3
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloLiveViewer(Node):
    def __init__(self):
        super().__init__("yolo_live_viewer")

        self.bridge = CvBridge()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_model = os.path.join(script_dir, "models", "yolo26n.pt")
        self.model_name = os.environ.get("YOLO_MODEL", default_model)
        self.conf = float(os.environ.get("YOLO_CONF", "0.25"))
        self.max_rate_hz = float(os.environ.get("YOLO_MAX_RATE_HZ", "5.0"))
        self.target_name = os.environ.get("YOLO_TARGET", "stop sign")
        self.show_window = os.environ.get("YOLO_SHOW", "1") != "0"
        self.last_inference_time = 0.0
        self.last_log_time = 0.0

        self.get_logger().info(f"Loading YOLO model: {self.model_name}")
        self.model = YOLO(self.model_name)
        self.get_logger().info(
            f"YOLO ready. Looking for target class containing: {self.target_name!r}"
        )

        self.annotated_pub = self.create_publisher(Image, "/yolo/image", 10)
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image",
            self.image_callback,
            10,
        )

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.publish_rgb_frame(frame)

        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / self.max_rate_hz:
            return
        self.last_inference_time = now

        results = self.model.predict(frame, imgsz=640, conf=self.conf, verbose=False)
        result = results[0]
        annotated = result.plot()

        detections = []
        if result.boxes is not None:
            for cls_idx, conf in zip(result.boxes.cls, result.boxes.conf):
                name = result.names[int(cls_idx)]
                detections.append((name, float(conf)))

        target_hits = [
            (name, conf)
            for name, conf in detections
            if self.target_name.lower() in name.lower()
        ]

        if now - self.last_log_time > 1.0:
            if target_hits:
                hit_str = ", ".join(f"{name} {conf:.2f}" for name, conf in target_hits)
                self.get_logger().info(f"Target detected: {hit_str}")
            elif detections:
                det_str = ", ".join(f"{name} {conf:.2f}" for name, conf in detections[:5])
                self.get_logger().info(f"Detections, no target: {det_str}")
            else:
                self.get_logger().info("No detections")
            self.last_log_time = now

        self.publish_rgb_frame(annotated)

        if self.show_window:
            try:
                cv2.imshow("YOLO detections (/camera/image)", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    raise KeyboardInterrupt
            except cv2.error as exc:
                self.get_logger().warn(
                    f"OpenCV window failed, continuing with /yolo/image only: {exc}"
                )
                self.show_window = False

    def publish_rgb_frame(self, frame_bgr):
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        self.annotated_pub.publish(
            self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
        )


def main():
    rclpy.init()
    node = YoloLiveViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
