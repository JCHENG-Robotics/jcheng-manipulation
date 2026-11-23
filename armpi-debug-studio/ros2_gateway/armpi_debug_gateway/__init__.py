"""Read-only ROS 2 gateway for ArmPi Debug Studio.

The gateway runs on the machine that owns the vendor ROS 2 stack. It exposes
battery, camera, and joint-command state over local HTTP. It does not implement
any motion write path.
"""

__version__ = "1.0.0"
