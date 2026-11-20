#!/usr/bin/env python3
"""Publish the off-board D435i colour stream as a ROS image topic.

This node is the **single owner** of ``/dev/video4``. The studio gateway used
to open the UVC device directly for its aux-camera preview, which made a ROS
image path impossible; the gateway now subscribes to this topic instead.

It publishes:

- ``/ext_cam/color/image_raw`` (sensor_msgs/Image, bgr8, sensor-data QoS)
- ``/ext_cam/color/camera_info`` (latched, from the 2026-11-20 calibration)
- ``base_link -> ext_cam_optical`` (static, from the same calibration)

Run inside the robot container (deployed as ``armpi-ext-cam.service``):

    python3 /home/ubuntu/shared/armpi_ext_cam/ext_cam_node.py \
        --extrinsic /home/ubuntu/shared/armpi_ext_cam/front_camera_extrinsic.json \
        --intrinsics /home/ubuntu/shared/armpi_ext_cam/uvc_color_intrinsics_aspect.json

Depth is **not** published: the raw UVC depth needs librealsense, and the
grasp band is inside the D435i's 0.2 m minimum range anyway.
"""

import argparse
import time
from array import array

import cv2
import numpy as np

import ext_cam_frames as frames

DEFAULT_TOPIC = '/ext_cam/color/image_raw'


def open_capture(device, width, height, fps):
    """Open the UVC device and request the configured mode."""
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        return None
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish the front D435i colour stream into ROS.')
    parser.add_argument('--device', default='/dev/video4')
    parser.add_argument('--extrinsic', required=True)
    parser.add_argument('--intrinsics', required=True)
    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    parser.add_argument('--camera-info-topic', default='/ext_cam/color/camera_info')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=15.0)
    args = parser.parse_args(argv)

    calibration = frames.load_calibration(args.extrinsic, args.intrinsics)

    capture = open_capture(args.device, args.width, args.height, args.fps)
    if capture is None:
        raise SystemExit(
            'cannot open {} (is another process holding the UVC device?)'
            .format(args.device))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros.static_transform_broadcaster import \
        StaticTransformBroadcaster

    rclpy.init()
    node = Node('ext_cam_node')
    image_qos = QoSProfile(
        depth=1, history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT)
    image_publisher = node.create_publisher(Image, args.topic, image_qos)
    info_qos = QoSProfile(
        depth=1, history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE)
    info_publisher = node.create_publisher(
        CameraInfo, args.camera_info_topic, info_qos)

    broadcaster = StaticTransformBroadcaster(node)
    transform = frames.build_transform(calibration)
    transform.header.stamp = node.get_clock().now().to_msg()
    broadcaster.sendTransform(transform)
    camera_info = frames.build_camera_info(calibration)
    camera_info.header.stamp = transform.header.stamp
    info_publisher.publish(camera_info)

    node.get_logger().info(
        'publishing {} ({}x{} @ {:.0f} fps) and {}'.format(
            args.topic, args.width, args.height, args.fps,
            args.camera_info_topic))

    period = 1.0 / args.fps if args.fps > 0 else 0.0
    published = 0
    failures = 0
    last_info = 0.0
    last_report = time.monotonic()
    published_at_report = 0
    t_read = t_pub = t_spin = t_sleep = t_build = 0.0
    n_timed = 0
    next_deadline = time.monotonic()
    loop_start = time.monotonic()
    try:
        while rclpy.ok():
            read_start = time.monotonic()
            ok, frame = capture.read()
            t_read += time.monotonic() - read_start
            if not ok or frame is None:
                failures += 1
                if failures > 50:
                    node.get_logger().error('camera stopped delivering frames')
                    capture.release()
                    capture = open_capture(
                        args.device, args.width, args.height, args.fps)
                    failures = 0
                    if capture is None:
                        time.sleep(1.0)
                        continue
                time.sleep(0.01)
                continue
            failures = 0
            build_start = time.monotonic()
            stamp = node.get_clock().now().to_msg()
            message = Image()
            message.header.stamp = stamp
            message.header.frame_id = frames.CHILD_FRAME
            message.height, message.width = frame.shape[:2]
            message.encoding = 'bgr8'
            message.is_bigendian = 0
            message.step = int(message.width * 3)
            # rclpy copies a bytes object into uint8[] element by element
            # (~420 ms for 720p); an array('B') is taken by reference.
            message.data = array(
                'B', np.ascontiguousarray(frame).tobytes())
            t_build += time.monotonic() - build_start
            pub_start = time.monotonic()
            image_publisher.publish(message)
            t_pub += time.monotonic() - pub_start
            published += 1
            if time.monotonic() - last_info > 10.0:
                camera_info.header.stamp = stamp
                info_publisher.publish(camera_info)
                last_info = time.monotonic()
            spin_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            t_spin += time.monotonic() - spin_start
            n_timed += 1
            now = time.monotonic()
            if now - last_report >= 5.0:
                node.get_logger().info(
                    '{:.1f} fps  read {:.0f}  build {:.0f}  publish {:.0f}  '
                    'spin {:.1f}  sleep {:.0f}  iter {:.0f} ms'.format(
                        (published - published_at_report) / (now - last_report),
                        1000.0 * t_read / max(n_timed, 1),
                        1000.0 * t_build / max(n_timed, 1),
                        1000.0 * t_pub / max(n_timed, 1),
                        1000.0 * t_spin / max(n_timed, 1),
                        1000.0 * t_sleep / max(n_timed, 1),
                        1000.0 * (now - loop_start) / max(n_timed, 1)))
                t_read = t_pub = t_spin = t_sleep = t_build = 0.0
                n_timed = 0
                loop_start = now
                last_report = now
                published_at_report = published
            if period:
                next_deadline += period
                delay = next_deadline - time.monotonic()
                if delay > 0:
                    sleep_start = time.monotonic()
                    time.sleep(delay)
                    t_sleep += time.monotonic() - sleep_start
                else:
                    next_deadline = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        node.get_logger().info('published {} frames'.format(published))
        rclpy.shutdown()


if __name__ == '__main__':
    main()
