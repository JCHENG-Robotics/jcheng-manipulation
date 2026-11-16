"""
Constrained grasp-candidate generation for the ArmPi 5-DoF arm.

The generator searches inside the task-space family described in
``docs/phase0/ARMPI_5DOF_CONSTRAINTS.md`` instead of proposing arbitrary 6D
tool poses:

    target position
    + approach pitch
    + approach azimuth
    + wrist roll
    + small stand-off along the approach axis

Each candidate is a ``gripper_link`` pose plus the corresponding TCP position,
so it can be sent to a full-pose IK solver. Candidate generation is pure
Python and does not import ROS.
"""

import dataclasses
import math

from .geometry import (
    matrix_from_axes,
    quaternion_from_matrix,
    tool_axes,
    wrap_degrees,
)

# ``end_effector_link`` sits 0.079 999 m above ``link5``; ``gripper_link`` is
# 0.005 438 8 m above ``link5`` (URDF fixed joints). The difference is the
# nominal grasp tool point used by the vendor model.
DEFAULT_TCP_OFFSET_M = 0.0745612

DEFAULT_PITCHES_DEG = (15.0, 30.0, 45.0, 60.0)
DEFAULT_APPROACH_YAWS_DEG = (0.0,)
DEFAULT_ROLLS_DEG = (-20.0, 0.0, 20.0)
DEFAULT_STANDOFFS_M = (0.0, 0.03)


@dataclasses.dataclass(frozen=True)
class GraspCandidate:
    """One candidate grasp in the constrained 5-DoF task space."""

    candidate_id: str
    pitch_deg: float
    approach_yaw_deg: float
    roll_deg: float
    standoff_m: float
    tcp_position: tuple
    tool_z: tuple
    gripper_position: tuple
    orientation: tuple
    is_exact_fk: bool = False
    source: str = 'cartesian'
    seed_joints: tuple = None
    target_offset_m: float = 0.0
    refined: bool = False

    def as_dict(self):
        """Return a JSON-friendly representation of the candidate."""
        return {
            'candidate_id': self.candidate_id,
            'pitch_deg': self.pitch_deg,
            'approach_yaw_deg': wrap_degrees(self.approach_yaw_deg),
            'roll_deg': self.roll_deg,
            'standoff_m': self.standoff_m,
            'is_exact_fk': self.is_exact_fk,
            'source': self.source,
            'refined': self.refined,
            'target_offset_m': self.target_offset_m,
            'sampled_joints': (
                list(self.seed_joints) if self.seed_joints else None
            ),
            'tcp_position': list(self.tcp_position),
            'tool_z': list(self.tool_z),
            'gripper_position': list(self.gripper_position),
            'gripper_orientation': list(self.orientation),
        }


def _unique(values):
    result = []
    seen = set()
    for value in values:
        key = round(float(value), 6)
        if key not in seen:
            seen.add(key)
            result.append(float(value))
    return result


def _candidate_id(pitch_deg, approach_yaw_deg, roll_deg, standoff_m):
    yaw = wrap_degrees(approach_yaw_deg)
    return 'p{p:06.2f}_a{y:+07.2f}_r{r:+07.2f}_s{s:03.0f}'.format(
        p=pitch_deg,
        y=yaw,
        r=roll_deg,
        s=standoff_m * 1000.0,
    )


def generate_candidates(
    grasp_point,
    pitches_deg=DEFAULT_PITCHES_DEG,
    approach_yaws_deg=DEFAULT_APPROACH_YAWS_DEG,
    rolls_deg=DEFAULT_ROLLS_DEG,
    standoffs_m=DEFAULT_STANDOFFS_M,
    tcp_offset_m=DEFAULT_TCP_OFFSET_M,
):
    """
    Generate the Cartesian product of the constrained candidate family.

    ``grasp_point`` is the desired tool point in ``base_link``. A positive
    stand-off moves the tool back along the approach axis, i.e. toward the
    pre-grasp pose. The returned ``gripper_position`` is the IK target for
    ``gripper_link``, which is ``tcp_offset_m`` behind the tool point.
    """
    candidates = []
    for pitch in _unique(pitches_deg):
        for approach_yaw in _unique(approach_yaws_deg):
            for roll in _unique(rolls_deg):
                for standoff in _unique(standoffs_m):
                    x_axis, y_axis, z_axis = tool_axes(
                        pitch, approach_yaw, roll
                    )
                    rotation = matrix_from_axes(x_axis, y_axis, z_axis)
                    quaternion = quaternion_from_matrix(rotation)
                    tcp = tuple(
                        grasp_point[index] - standoff * z_axis[index]
                        for index in range(3)
                    )
                    gripper = tuple(
                        tcp[index] - tcp_offset_m * z_axis[index]
                        for index in range(3)
                    )
                    candidates.append(
                        GraspCandidate(
                            candidate_id=_candidate_id(
                                pitch, approach_yaw, roll, standoff
                            ),
                            pitch_deg=pitch,
                            approach_yaw_deg=wrap_degrees(approach_yaw),
                            roll_deg=roll,
                            standoff_m=standoff,
                            tcp_position=tcp,
                            tool_z=z_axis,
                            gripper_position=gripper,
                            orientation=quaternion,
                        )
                    )
    return candidates


def mark_exact_fk(candidates, pitch_deg, approach_yaw_deg, roll_deg,
                  tolerance_deg=1e-6):
    """
    Mark the candidate that reproduces an FK pose exactly.

    The match is by candidate parameters (stand-off zero). Returns a new list
    with a single candidate flagged ``is_exact_fk`` when a match exists.
    """
    yaw = wrap_degrees(approach_yaw_deg)
    marked = []
    found = False
    for candidate in candidates:
        if (
            not found
            and abs(candidate.standoff_m) < 1e-9
            and abs(candidate.pitch_deg - pitch_deg) < tolerance_deg
            and abs(wrap_degrees(candidate.approach_yaw_deg - yaw))
            < tolerance_deg
            and abs(candidate.roll_deg - roll_deg) < tolerance_deg
        ):
            candidate = dataclasses.replace(candidate, is_exact_fk=True)
            found = True
        marked.append(candidate)
    return marked, found


def sort_candidates(candidates):
    """Sort for evaluation: exact FK first, then refined, then offset."""
    return sorted(
        candidates,
        key=lambda candidate: (
            not candidate.is_exact_fk,
            round(candidate.target_offset_m, 4),
            not candidate.refined,
            candidate.standoff_m,
            abs(candidate.pitch_deg),
            abs(candidate.roll_deg),
            candidate.approach_yaw_deg,
        ),
    )


def assess_sanity(
    candidate,
    max_pitch_deg=75.0,
    object_width_m=None,
    gripper_max_opening_m=None,
):
    """
    Check grasp-policy constraints that do not require IK.

    These checks are intentionally conservative: they reject side approaches
    steeper than the initial policy and, when both dimensions are supplied,
    objects wider than the gripper can open.
    """
    issues = []
    if candidate.pitch_deg > max_pitch_deg:
        issues.append(
            'approach pitch {:.1f} deg exceeds policy limit {:.1f} deg'.format(
                candidate.pitch_deg, max_pitch_deg
            )
        )
    if (
        object_width_m is not None
        and gripper_max_opening_m is not None
        and object_width_m > gripper_max_opening_m
    ):
        issues.append(
            'object width {:.4f} m exceeds gripper opening {:.4f} m'.format(
                object_width_m, gripper_max_opening_m
            )
        )
    if not math.isfinite(candidate.pitch_deg):
        issues.append('pitch is not finite')
    return {'ok': not issues, 'issues': issues}
