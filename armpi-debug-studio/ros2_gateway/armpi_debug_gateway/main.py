"""Entry point for the read-only ArmPi Debug Gateway.

Run on the robot host, inside the ROS 2 environment that owns the vendor stack:

    source /opt/ros/humble/setup.bash
    source <vendor_ws>/install/setup.bash
    export ARMPI_GATEWAY_TOKEN=change-me
    python3 -m armpi_debug_gateway.main --host 127.0.0.1 --port 8765
"""

from __future__ import annotations

import argparse
import os
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .node import GatewayNode
from .server import GatewayHTTPServer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ArmPi Debug Studio read-only ROS 2 gateway")
    parser.add_argument("--battery-topic", default="/ros_robot_controller/battery")
    parser.add_argument("--camera-topic", default="/depth_cam/rgb/image_raw")
    parser.add_argument("--joint-state-topic", default="/controller_manager/joint_states")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--host", default="127.0.0.1", help="bind address; keep localhost unless using a private tailnet")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--token", default=os.getenv("ARMPI_GATEWAY_TOKEN"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.host not in {"127.0.0.1", "localhost", "::1"} and not args.token:
        print("WARNING: binding beyond localhost without ARMPI_GATEWAY_TOKEN is not allowed for shared networks.")
        print("Set ARMPI_GATEWAY_TOKEN or keep --host 127.0.0.1 and use an SSH tunnel.")

    rclpy.init()
    node = GatewayNode(
        battery_topic=args.battery_topic,
        camera_topic=args.camera_topic,
        joint_state_topic=args.joint_state_topic,
        jpeg_quality=args.jpeg_quality,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="gateway-spin", daemon=True)
    spin_thread.start()

    server = GatewayHTTPServer((args.host, args.port), node, token=args.token)
    print(f"armpi_debug_gateway listening on http://{args.host}:{args.port}")
    print(f"  battery: {args.battery_topic}")
    print(f"  camera:  {args.camera_topic}")
    print(f"  joints:  {args.joint_state_topic}")
    print("  motion:  DISABLED (read-only gateway)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
