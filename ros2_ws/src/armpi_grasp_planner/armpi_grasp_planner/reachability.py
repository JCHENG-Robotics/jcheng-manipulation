"""
Reachability-aware grasp sampling for the ArmPi 5-DoF arm.

The Cartesian candidate family in :mod:`armpi_grasp_planner.candidates` asks
IK for 6D poses that may be outside a 5-DoF arm's reachable set. At a fixed
tool point the reachable orientations form a thin manifold, so a Cartesian
grid mostly fails. This module samples the joint space instead and keeps the
FK poses whose tool point is near the target. Every sampled pose is reachable
by construction; IK is then used only to verify or refine it.

The local FK reproduces MoveIt's ``/compute_fk`` exactly for this model (0.0 mm
per link at the checked configuration). The evaluator still verifies every
candidate with MoveIt, so a model mismatch cannot silently pass.

Requires ``PyKDL``, ``urdf_parser_py`` and ``xacro``.
"""

import math

import PyKDL
from urdf_parser_py.urdf import URDF
import xacro

from .candidates import GraspCandidate
from .geometry import (
    clamp,
    decompose_tool_orientation,
    quaternion_from_matrix,
)


def default_urdf_path():
    """Return the installed ArmPi description xacro path."""
    from ament_index_python.packages import get_package_share_directory

    return (
        get_package_share_directory('armpi_ultra_description')
        + '/urdf/armpi_ultra.xacro'
    )


def _axis_values(lower, upper, step):
    count = max(1, int(math.floor((upper - lower) / step)))
    return [lower + index * step for index in range(count + 1)]


def _distance(a, b):
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def _rotation_rows(kdl_rotation):
    return tuple(
        tuple(kdl_rotation[row, column] for column in range(3))
        for row in range(3)
    )


class ArmKinematics:
    """KDL forward kinematics for the ArmPi arm chain."""

    def __init__(self, urdf_path=None, base_link='base_link',
                 tip_link='end_effector_link'):
        self.urdf_path = urdf_path or default_urdf_path()
        self.base_link = base_link
        self.tip_link = tip_link
        robot = URDF.from_xml_string(xacro.process_file(self.urdf_path).toxml())
        joints = self._chain_joints(robot, base_link, tip_link)
        self.joint_names = []
        self.limits = {}
        chain = PyKDL.Chain()
        for joint in joints:
            xyz = list(joint.origin.xyz) if joint.origin else [0.0, 0.0, 0.0]
            rpy = list(joint.origin.rpy) if joint.origin else [0.0, 0.0, 0.0]
            rotation = PyKDL.Rotation.RPY(*rpy)
            translation = PyKDL.Vector(*xyz)
            if joint.type in ('revolute', 'continuous'):
                axis = rotation * PyKDL.Vector(*joint.axis)
                kdl_joint = PyKDL.Joint(
                    joint.name, translation, axis, PyKDL.Joint.RotAxis
                )
                # The origin translation is deliberately present in both the
                # joint and the tip frame; KDL composes it as
                # Trans(o) * Rot(q) * Trans(-o) * f_tip.
                tip_frame = PyKDL.Frame(rotation, translation)
                self.joint_names.append(joint.name)
                if joint.type == 'continuous':
                    self.limits[joint.name] = (-math.pi, math.pi)
                else:
                    self.limits[joint.name] = (
                        float(joint.limit.lower),
                        float(joint.limit.upper),
                    )
            elif joint.type == 'fixed':
                kdl_joint = PyKDL.Joint(PyKDL.Joint.Fixed)
                tip_frame = PyKDL.Frame(rotation, translation)
            else:
                raise ValueError(
                    'unsupported joint type {} on {}'.format(
                        joint.type, joint.name
                    )
                )
            chain.addSegment(
                PyKDL.Segment(joint.name, kdl_joint, tip_frame)
            )
        if not self.joint_names:
            raise ValueError('no actuated joints in the arm chain')
        self.chain = chain
        self.solver = PyKDL.ChainFkSolverPos_recursive(chain)

    @staticmethod
    def _chain_joints(robot, base_link, tip_link):
        parent = {joint.child: joint for joint in robot.joints}
        joints = []
        link = tip_link
        while link != base_link:
            if link not in parent:
                raise ValueError(
                    'link {} is not connected to {}'.format(link, base_link)
                )
            joint = parent[link]
            joints.append(joint)
            link = joint.parent
        joints.reverse()
        return joints

    def forward(self, joints):
        """Return the KDL TCP frame for the five arm joint values."""
        if len(joints) != len(self.joint_names):
            raise ValueError(
                'expected {} joint values, got {}'.format(
                    len(self.joint_names), len(joints)
                )
            )
        array = PyKDL.JntArray(len(self.joint_names))
        for index, value in enumerate(joints):
            array[index] = float(value)
        frame = PyKDL.Frame()
        if self.solver.JntToCart(array, frame) < 0:
            raise RuntimeError('KDL forward kinematics failed')
        return frame

    def sample_candidates(
        self,
        target,
        tcp_offset_m,
        radius_m=0.02,
        position_step_rad=0.2,
        roll_step_rad=0.3,
        max_pitch_deg=75.0,
        max_candidates=80,
        include_refined=True,
    ):
        """
        Sample FK-reachable grasps whose tool point is near *target*.

        ``target`` is the desired tool point in ``base_link``. Sampled poses
        that land within ``radius_m`` become ``joint_sample`` candidates.
        For each sampled orientation, a ``joint_sample_refined`` candidate is
        also generated at exactly ``target``; its IK may fail, and the sampled
        pose is kept as the fallback.
        """
        names = self.joint_names
        lower = [self.limits[name][0] for name in names]
        upper = [self.limits[name][1] for name in names]
        first_axes = [
            _axis_values(lower[index], upper[index], position_step_rad)
            for index in range(4)
        ]
        roll_values = _axis_values(lower[4], upper[4], roll_step_rad)
        refined = []
        sampled = []
        seen = set()
        for j1 in first_axes[0]:
            for j2 in first_axes[1]:
                for j3 in first_axes[2]:
                    for j4 in first_axes[3]:
                        base_joints = (j1, j2, j3, j4, 0.0)
                        frame = self.forward(base_joints)
                        tcp = (
                            frame.p.x(), frame.p.y(), frame.p.z()
                        )
                        offset = _distance(tcp, target)
                        if offset > radius_m:
                            continue
                        tool_z = (
                            frame.M[0, 2], frame.M[1, 2], frame.M[2, 2]
                        )
                        pitch = math.degrees(
                            math.acos(clamp(-tool_z[2], -1.0, 1.0))
                        )
                        if pitch > max_pitch_deg:
                            continue
                        for j5 in roll_values:
                            joints = (j1, j2, j3, j4, j5)
                            final = self.forward(joints)
                            tcp = (
                                final.p.x(), final.p.y(), final.p.z()
                            )
                            offset = _distance(tcp, target)
                            if offset > radius_m + 1e-6:
                                continue
                            tool_z = (
                                final.M[0, 2],
                                final.M[1, 2],
                                final.M[2, 2],
                            )
                            pitch = math.degrees(
                                math.acos(clamp(-tool_z[2], -1.0, 1.0))
                            )
                            if pitch > max_pitch_deg:
                                continue
                            rotation = _rotation_rows(final.M)
                            parameters = decompose_tool_orientation(rotation)
                            key = (
                                round(parameters['pitch_deg'], 2),
                                round(parameters['approach_yaw_deg'], 2),
                                round(parameters['roll_deg'], 2),
                                round(offset, 4),
                                round(tcp[0], 4),
                                round(tcp[1], 4),
                                round(tcp[2], 4),
                            )
                            if key in seen:
                                continue
                            seen.add(key)
                            quaternion = quaternion_from_matrix(rotation)
                            sampled.append(
                                GraspCandidate(
                                    candidate_id='js_{:04d}'.format(
                                        len(sampled)
                                    ),
                                    pitch_deg=parameters['pitch_deg'],
                                    approach_yaw_deg=parameters[
                                        'approach_yaw_deg'
                                    ],
                                    roll_deg=parameters['roll_deg'],
                                    standoff_m=0.0,
                                    tcp_position=tcp,
                                    tool_z=tool_z,
                                    gripper_position=tuple(
                                        tcp[index]
                                        - tcp_offset_m * tool_z[index]
                                        for index in range(3)
                                    ),
                                    orientation=quaternion,
                                    source='joint_sample',
                                    seed_joints=joints,
                                    target_offset_m=offset,
                                )
                            )
                            if not include_refined:
                                continue
                            exact_tcp = tuple(float(value) for value in target)
                            refined.append(
                                GraspCandidate(
                                    candidate_id='jsr_{:04d}'.format(
                                        len(refined)
                                    ),
                                    pitch_deg=parameters['pitch_deg'],
                                    approach_yaw_deg=parameters[
                                        'approach_yaw_deg'
                                    ],
                                    roll_deg=parameters['roll_deg'],
                                    standoff_m=0.0,
                                    tcp_position=exact_tcp,
                                    tool_z=tool_z,
                                    gripper_position=tuple(
                                        exact_tcp[index]
                                        - tcp_offset_m * tool_z[index]
                                        for index in range(3)
                                    ),
                                    orientation=quaternion,
                                    source='joint_sample_refined',
                                    seed_joints=joints,
                                    target_offset_m=0.0,
                                    refined=True,
                                )
                            )
        refined.sort(key=_sampled_sort_key)
        sampled.sort(key=_sampled_sort_key)
        limit = max(1, max_candidates // 2)
        selected = refined[:limit] + sampled[:limit]
        return selected[:max_candidates]


def _sampled_sort_key(candidate):
    return (
        candidate.target_offset_m,
        candidate.pitch_deg,
        abs(candidate.roll_deg),
        candidate.approach_yaw_deg,
    )
