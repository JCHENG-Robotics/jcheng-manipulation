"""Entry point for the ArmPi Debug Gateway.

Read-only by default. Run on the robot host, inside the ROS 2 environment that
owns the vendor stack:

    source /opt/ros/humble/setup.bash
    source <vendor_ws>/install/setup.bash
    export ARMPI_GATEWAY_TOKEN=change-me
    python3 -m armpi_debug_gateway.main --host 127.0.0.1 --port 8765

Supervised motion requires **all** of: a token, ``--enable-motion`` and a
validated ``--motion-config`` JSON file. Without them the motion endpoints
answer ``MOTION_DISABLED`` and the node never creates a publisher.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .auth import TokenError, resolve_token as resolve_shared_token
from .aux_camera import RosImageCamera, UvcCamera
from .motion import MotionConfigError, load_motion_config
from .node import GatewayNode
from .server import GatewayHTTPServer

LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ArmPi Debug Studio read-only ROS 2 gateway")
    parser.add_argument("--battery-topic", default="/ros_robot_controller/battery")
    parser.add_argument("--camera-topic", default="/depth_cam/rgb/image_raw")
    parser.add_argument("--joint-state-topic", default="/controller_manager/joint_states")
    parser.add_argument(
        "--servo-state-topic",
        default="/controller_manager/servo_states",
        help="measured bus-servo positions; required for motion completion",
    )
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument(
        "--aux-camera-device",
        default="/dev/video4",
        help="UVC color device of the off-arm D435i (it has no ROS driver)",
    )
    parser.add_argument("--aux-camera-width", type=int, default=640)
    parser.add_argument("--aux-camera-height", type=int, default=480)
    parser.add_argument("--aux-camera-fps", type=float, default=10.0)
    parser.add_argument(
        "--aux-camera-topic",
        default="",
        help="subscribe to this Image topic instead of opening --aux-camera-device "
        "(single-owner mode with the robot-side ext_cam_node)",
    )
    parser.add_argument(
        "--no-aux-camera",
        action="store_true",
        help="do not open the UVC aux camera; /aux-camera/* then reports unavailable",
    )
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="enable the supervised motion endpoints; requires --motion-config and a token",
    )
    parser.add_argument(
        "--motion-config",
        default=None,
        help="JSON limits file generated from configs/joint_mapping.yaml",
    )
    parser.add_argument("--host", default="127.0.0.1", help="bind address; keep localhost unless using a private tailnet")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--token", default=None, help="shared token (prefer --token-file or ARMPI_GATEWAY_TOKEN)")
    parser.add_argument(
        "--token-file",
        default=os.getenv("ARMPI_GATEWAY_TOKEN_FILE"),
        help="read the shared token from this file (recommended for service deployment)",
    )
    return parser.parse_args(argv)


def _resolve_token(args: argparse.Namespace) -> str | None:
    """Resolve the shared token and turn configuration errors into exit code 2."""
    try:
        return resolve_shared_token(token=args.token, token_file=args.token_file)
    except TokenError as exc:
        raise SystemExit(f"ERROR: {exc}") from exc


def _install_signal_handlers(stop_event: threading.Event) -> None:
    def _stop(signum: int, _frame: object) -> None:
        print(f"armpi_debug_gateway: received signal {signum}, shutting down", flush=True)
        stop_event.set()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    token = _resolve_token(args)
    if args.host not in LOOPBACK_HOSTS and not token:
        print(
            "ERROR: refusing to bind beyond localhost without a token. "
            "Set --token-file/ARMPI_GATEWAY_TOKEN, or keep --host 127.0.0.1 and use an SSH tunnel.",
            file=sys.stderr,
        )
        raise SystemExit(2)

    motion_config = None
    if args.enable_motion:
        if not token:
            print(
                "ERROR: --enable-motion requires a gateway token; refusing to "
                "expose a motion endpoint without authentication.",
                file=sys.stderr,
            )
            raise SystemExit(2)
        if not args.motion_config:
            print(
                "ERROR: --enable-motion requires --motion-config with the "
                "validated pulse limits.",
                file=sys.stderr,
            )
            raise SystemExit(2)
        try:
            motion_config = load_motion_config(args.motion_config)
        except MotionConfigError as exc:
            print(f"ERROR: invalid motion config: {exc}", file=sys.stderr)
            raise SystemExit(2) from exc

    stop_event = threading.Event()
    _install_signal_handlers(stop_event)

    aux_camera = None
    if not args.no_aux_camera and args.aux_camera_topic:
        aux_camera = RosImageCamera(
            args.aux_camera_topic,
            quality=args.jpeg_quality,
            log=lambda message: print(f"  aux:     {message}", flush=True),
        )
    elif not args.no_aux_camera:
        aux_camera = UvcCamera(
            args.aux_camera_device,
            width=args.aux_camera_width,
            height=args.aux_camera_height,
            fps=args.aux_camera_fps,
            quality=args.jpeg_quality,
            log=lambda message: print(f"  aux:     {message}", flush=True),
        )

    rclpy.init()
    try:
        node = GatewayNode(
            battery_topic=args.battery_topic,
            camera_topic=args.camera_topic,
            joint_state_topic=args.joint_state_topic,
            servo_state_topic=args.servo_state_topic,
            tf_topic=args.tf_topic,
            tf_static_topic=args.tf_static_topic,
            jpeg_quality=args.jpeg_quality,
            aux_camera=aux_camera,
            motion_enabled=bool(args.enable_motion),
            motion_config=motion_config,
        )
    except RuntimeError as exc:
        rclpy.shutdown()
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
    if aux_camera is not None:
        aux_camera.start()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="gateway-spin", daemon=True)
    spin_thread.start()

    server: GatewayHTTPServer | None = None
    try:
        server = GatewayHTTPServer((args.host, args.port), node, token=token)
        print(f"armpi_debug_gateway listening on http://{args.host}:{args.port}", flush=True)
        print(f"  battery: {args.battery_topic}")
        print(f"  camera:  {args.camera_topic}")
        if args.no_aux_camera:
            print("  aux:     DISABLED")
        else:
            print(
                f"  aux:     {args.aux_camera_device} "
                f"({args.aux_camera_width}x{args.aux_camera_height} "
                f"@ {args.aux_camera_fps:g} fps UVC)"
            )
        print(f"  joints:  {args.joint_state_topic}")
        print(f"  measured:{args.servo_state_topic}")
        print(f"  tf:      {args.tf_topic} + {args.tf_static_topic}")
        if node.motion_enabled:
            assert motion_config is not None
            print(
                "  motion:  ENABLED (supervised) "
                f"step<={motion_config.safety.max_step_rad} rad "
                f"single_joint={motion_config.safety.single_joint_only}"
            )
        else:
            print("  motion:  DISABLED (read-only gateway)")
        server.serve_until(stop_event)
    finally:
        if server is not None:
            server.server_close()
        if aux_camera is not None:
            aux_camera.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("armpi_debug_gateway: stopped", flush=True)


if __name__ == "__main__":
    main()
