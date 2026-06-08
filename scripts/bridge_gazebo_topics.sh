#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image[ignition.msgs.Image \
  /camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image \
  /camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo \
  /world/litterbot_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V
