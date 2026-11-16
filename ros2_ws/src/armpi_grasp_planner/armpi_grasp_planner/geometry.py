"""
Pure-Python tool-frame geometry for the ArmPi 5-DoF grasp search.

Conventions
-----------
* Poses are expressed in ``base_link`` unless stated otherwise.
* The tool approach axis is the gripper ``+z`` axis. The finger-opening axis is
  the gripper ``y`` axis: the URDF finger joints rotate about ``x`` and are
  offset along ``y`` (see ``gripper.urdf.xacro``).
* ``pitch`` is the angle between the approach axis and straight down, in
  degrees. ``pitch=0`` is a perfectly vertical top-down approach. The real arm
  cannot quite reach it because of the joint limits; see
  ``docs/phase2/GRASP_CANDIDATE_SEARCH.md``.
* ``approach_yaw`` is the azimuth of the tilt direction, measured from
  ``base_link`` +x. It also sets the zero-roll finger direction
  ``h = (-sin(approach_yaw), cos(approach_yaw), 0)``.
* ``roll`` rotates the fingers about the approach axis away from ``h``.
* ``tcp_offset`` is the distance from ``gripper_link`` to the grasp tool point
  along ``+z``. The URDF ``end_effector_link`` is 0.0746 m past
  ``gripper_link``, so it is used as the nominal TCP.

These are project conventions for the constrained grasp family, not ROS
end-effector conventions.
"""

import math

EPS = 1e-12


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v):
    return math.sqrt(_dot(v, v))


def _normalize(v):
    length = _norm(v)
    if length < EPS:
        raise ValueError('cannot normalize a zero vector')
    return (v[0] / length, v[1] / length, v[2] / length)


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(v, factor):
    return (v[0] * factor, v[1] * factor, v[2] * factor)


def clamp(value, low, high):
    """Clamp *value* into the inclusive interval [low, high]."""
    return max(low, min(high, value))


def wrap_degrees(angle):
    """Wrap an angle in degrees into (-180, 180]."""
    wrapped = math.fmod(angle + 180.0, 360.0)
    if wrapped <= 0.0:
        wrapped += 360.0
    return wrapped - 180.0


def tool_z_from_angles(pitch_deg, approach_yaw_deg):
    """Return the unit tool approach axis for a pitch/azimuth pair."""
    pitch = math.radians(pitch_deg)
    yaw = math.radians(approach_yaw_deg)
    return (
        math.sin(pitch) * math.cos(yaw),
        math.sin(pitch) * math.sin(yaw),
        -math.cos(pitch),
    )


def tool_axes(pitch_deg, approach_yaw_deg, roll_deg):
    """
    Return the tool frame axes ``(x, y, z)`` in ``base_link``.

    ``z`` is the approach axis, ``y`` is the finger-opening axis after applying
    ``roll_deg`` about ``z``, and ``x = y x z``.
    """
    approach = tool_z_from_angles(pitch_deg, approach_yaw_deg)
    yaw = math.radians(approach_yaw_deg)
    reference = (-math.sin(yaw), math.cos(yaw), 0.0)
    reference = _normalize(
        _sub(reference, _scale(approach, _dot(reference, approach)))
    )
    lateral = _cross(approach, reference)
    roll = math.radians(roll_deg)
    fingers = _add(
        _scale(reference, math.cos(roll)),
        _scale(lateral, math.sin(roll)),
    )
    x_axis = _cross(fingers, approach)
    return x_axis, fingers, approach


def matrix_from_axes(x_axis, y_axis, z_axis):
    """Build a rotation matrix whose columns are the given axes."""
    return (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )


def rotation_from_tool_angles(pitch_deg, approach_yaw_deg, roll_deg):
    """Return the 3x3 tool rotation matrix for the candidate parameters."""
    return matrix_from_axes(*tool_axes(pitch_deg, approach_yaw_deg, roll_deg))


def quaternion_from_matrix(matrix):
    """Convert a 3x3 rotation matrix to a unit quaternion ``(x, y, z, w)``."""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < EPS:
        raise ValueError('cannot normalize a zero quaternion')
    return (x / norm, y / norm, z / norm, w / norm)


def matrix_from_quaternion(quaternion):
    """Convert a quaternion ``(x, y, z, w)`` to a 3x3 rotation matrix."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < EPS:
        raise ValueError('cannot normalize a zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def rotation_angle_deg(matrix_a, matrix_b):
    """
    Return the geodesic angle between two rotation matrices in degrees.

    The angle comes from ``atan2`` of the relative-rotation skew and trace
    rather than ``acos(trace)``, which loses precision near zero.
    """
    relative = []
    for row in range(3):
        relative_row = []
        for column in range(3):
            relative_row.append(
                sum(
                    matrix_a[index][row] * matrix_b[index][column]
                    for index in range(3)
                )
            )
        relative.append(tuple(relative_row))
    skew_x = relative[2][1] - relative[1][2]
    skew_y = relative[0][2] - relative[2][0]
    skew_z = relative[1][0] - relative[0][1]
    sine = 0.5 * math.sqrt(skew_x ** 2 + skew_y ** 2 + skew_z ** 2)
    cosine = 0.5 * (
        relative[0][0] + relative[1][1] + relative[2][2] - 1.0
    )
    return math.degrees(math.atan2(sine, cosine))


def decompose_tool_orientation(matrix):
    """
    Decompose a tool rotation into pitch, approach yaw and roll.

    The returned parameters reproduce ``matrix`` exactly when passed to
    :func:`rotation_from_tool_angles`, except that the approach yaw is
    unobservable at ``pitch == 0`` and is reported as zero.
    """
    approach = (matrix[0][2], matrix[1][2], matrix[2][2])
    pitch_deg = math.degrees(math.acos(clamp(-approach[2], -1.0, 1.0)))
    if math.hypot(approach[0], approach[1]) < EPS:
        approach_yaw_deg = 0.0
    else:
        approach_yaw_deg = math.degrees(math.atan2(approach[1], approach[0]))
    yaw = math.radians(approach_yaw_deg)
    reference = (-math.sin(yaw), math.cos(yaw), 0.0)
    lateral = _cross(approach, reference)
    fingers = (matrix[0][1], matrix[1][1], matrix[2][1])
    roll_deg = math.degrees(
        math.atan2(_dot(fingers, lateral), _dot(fingers, reference))
    )
    return {
        'pitch_deg': pitch_deg,
        'approach_yaw_deg': approach_yaw_deg,
        'roll_deg': roll_deg,
    }
