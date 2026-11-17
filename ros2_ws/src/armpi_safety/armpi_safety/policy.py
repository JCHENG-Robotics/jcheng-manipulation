"""
Conservative safety policy for planned ArmPi trajectories.

This module is pure Python: it does not import ROS message types, so the
checks can be unit-tested without a ROS runtime. The ROS node converts
``moveit_msgs/RobotTrajectory`` into the plain dictionary schema used here.

The policy is a **veto layer**, not a controller. It never sends a command and
never executes a trajectory. A future executor must call the safety service and
refuse to execute when ``approved`` is false.

The joint limits are the provisional URDF/MoveIt values. Mechanical limits have
not been measured on the real robot; see ``docs/phase2/JOINT_LIMITS.md``.
"""

import dataclasses
import math
import time

ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
EPS = 1e-9

DEFAULT_LIMITS = {
    'joint1': (-2.09, 2.09),
    'joint2': (-2.09, 2.09),
    'joint3': (-2.09, 0.0),
    'joint4': (-2.09, 2.09),
    'joint5': (-2.09, 2.09),
}


@dataclasses.dataclass(frozen=True)
class JointLimit:
    """Position and rate limits for one joint."""

    lower: float
    upper: float
    max_velocity: float
    max_acceleration: float


@dataclasses.dataclass
class SafetyConfig:
    """Configuration for the state watchdog and trajectory checks."""

    joint_limits: dict
    state_timeout_s: float = 0.5
    max_start_state_distance_rad: float = 0.08
    max_duration_s: float = 30.0
    max_points: int = 5000
    position_tolerance_rad: float = 1e-3
    velocity_tolerance_rad_s: float = 1e-3
    acceleration_tolerance_rad_s2: float = 1e-3
    workspace_enabled: bool = True
    workspace_required: bool = True
    workspace_frame: str = 'base_link'
    workspace_lower: tuple = (-0.05, -0.30, 0.00)
    workspace_upper: tuple = (0.45, 0.30, 0.55)

    @classmethod
    def from_dict(cls, config):
        """Build a configuration from a plain dictionary."""
        limits = {}
        raw_limits = config.get('joint_limits', {})
        for joint in ARM_JOINTS:
            raw = raw_limits.get(joint)
            if raw is None:
                lower, upper = DEFAULT_LIMITS[joint]
                raw = {
                    'lower': lower,
                    'upper': upper,
                    'max_velocity': 1.2,
                    'max_acceleration': 0.5,
                }
            limits[joint] = JointLimit(
                lower=float(raw['lower']),
                upper=float(raw['upper']),
                max_velocity=float(raw.get('max_velocity', 1.2)),
                max_acceleration=float(
                    raw.get('max_acceleration', 0.5)
                ),
            )
        state = config.get('state', {})
        start = config.get('start_state', {})
        trajectory = config.get('trajectory', {})
        workspace = config.get('workspace', {})
        lower = tuple(
            float(value)
            for value in workspace.get(
                'lower', (-0.05, -0.30, 0.00)
            )
        )
        upper = tuple(
            float(value)
            for value in workspace.get(
                'upper', (0.45, 0.30, 0.55)
            )
        )
        if len(lower) != 3 or len(upper) != 3:
            raise ValueError('workspace bounds must have 3 values')
        for low, high in zip(lower, upper):
            if low >= high:
                raise ValueError(
                    'workspace lower bound must be below upper bound'
                )
        return cls(
            joint_limits=limits,
            state_timeout_s=float(state.get('timeout_s', 0.5)),
            max_start_state_distance_rad=float(
                start.get('max_distance_rad', 0.08)
            ),
            max_duration_s=float(trajectory.get('max_duration_s', 30.0)),
            max_points=int(trajectory.get('max_points', 5000)),
            position_tolerance_rad=float(
                trajectory.get('position_tolerance_rad', 1e-3)
            ),
            velocity_tolerance_rad_s=float(
                trajectory.get('velocity_tolerance_rad_s', 1e-3)
            ),
            acceleration_tolerance_rad_s2=float(
                trajectory.get('acceleration_tolerance_rad_s2', 1e-3)
            ),
            workspace_enabled=bool(workspace.get('enabled', True)),
            workspace_required=bool(workspace.get('required', True)),
            workspace_frame=str(workspace.get('frame', 'base_link')),
            workspace_lower=lower,
            workspace_upper=upper,
        )


def load_config(path):
    """Load a safety configuration YAML file."""
    import yaml

    with open(path, 'r', encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    return SafetyConfig.from_dict(raw)


def _finite(value):
    return math.isfinite(float(value))


def _violation(check, message, **details):
    entry = {'check': check, 'message': message}
    entry.update(details)
    return entry


def _transpose_points(points, joint_names):
    """Return ``{joint: [value per point]}`` for the requested joints."""
    index = {name: position
             for position, name in enumerate(joint_names)}
    result = {}
    for joint in ARM_JOINTS:
        if joint in index:
            column = index[joint]
            result[joint] = [
                float(point['positions'][column]) for point in points
            ]
    return result


class SafetyMonitor:
    """State watchdog and trajectory veto with a latched stop flag."""

    def __init__(self, config, clock=time.monotonic):
        """Create a monitor for one robot configuration."""
        self.config = config
        self.clock = clock
        self.latest_positions = None
        self.latest_stamp = None
        self.stop_latched = False
        self.stop_reason = None
        self.last_validation = None

    def update_joint_state(self, names, positions, received_at=None):
        """Store the latest measured or broadcast joint state."""
        stamp = self.clock() if received_at is None else float(received_at)
        self.latest_positions = {
            str(name): float(position)
            for name, position in zip(names, positions)
        }
        self.latest_stamp = stamp

    def state_report(self, now=None):
        """Check state freshness, completeness and joint limits."""
        now = self.clock() if now is None else float(now)
        violations = []
        if self.latest_positions is None or self.latest_stamp is None:
            violations.append(
                _violation('state', 'no joint state has been received')
            )
            return self._state_result(now, violations, None)
        age = max(0.0, now - self.latest_stamp)
        if age > self.config.state_timeout_s:
            violations.append(
                _violation(
                    'state',
                    'joint state is stale: {:.3f} s > {:.3f} s'.format(
                        age, self.config.state_timeout_s
                    ),
                    age_s=age,
                    timeout_s=self.config.state_timeout_s,
                )
            )
        for joint in ARM_JOINTS:
            if joint not in self.latest_positions:
                violations.append(
                    _violation(
                        'state',
                        'joint state is missing {}'.format(joint),
                        joint=joint,
                    )
                )
                continue
            value = self.latest_positions[joint]
            if not _finite(value):
                violations.append(
                    _violation(
                        'state',
                        'joint state is not finite for {}'.format(joint),
                        joint=joint,
                        value=value,
                    )
                )
                continue
            limit = self.config.joint_limits[joint]
            if (
                value < limit.lower - self.config.position_tolerance_rad
                or value > limit.upper + self.config.position_tolerance_rad
            ):
                violations.append(
                    _violation(
                        'state',
                        '{} state {:.4f} is outside [{:.4f}, {:.4f}]'.format(
                            joint, value, limit.lower, limit.upper
                        ),
                        joint=joint,
                        value=value,
                        lower=limit.lower,
                        upper=limit.upper,
                    )
                )
        return self._state_result(now, violations, age)

    def _state_result(self, now, violations, age):
        valid = not violations
        # A real state fault latches the stop. The initial "no state yet"
        # condition does not latch, so a normal startup recovers when the
        # first joint-state message arrives.
        if violations and self.latest_positions is not None:
            if not self.stop_latched:
                self._latch('STATE_INVALID')
        return {
            'valid': valid,
            'age_s': age,
            'violations': violations,
            'stop': self.stop_latched or not valid,
            'stop_latched': self.stop_latched,
            'stop_reason': self.stop_reason,
            'positions': (
                dict(self.latest_positions)
                if self.latest_positions is not None
                else None
            ),
        }

    def status(self, now=None):
        """Return the combined watchdog status for publishing."""
        state = self.state_report(now=now)
        return {
            'schema_version': 1,
            'state': state,
            'last_validation': self.last_validation,
            'stop': state['stop'],
        }

    def reset_stop(self, now=None):
        """Clear a latched stop only when the current state is valid."""
        state = self.state_report(now=now)
        if not state['valid']:
            return {
                'ok': False,
                'message': 'current state is invalid; stop not cleared',
                'state': state,
            }
        self.stop_latched = False
        self.stop_reason = None
        return {
            'ok': True,
            'message': 'stop cleared',
            'state': self.state_report(now=now),
        }

    def _latch(self, code):
        self.stop_latched = True
        self.stop_reason = code

    def validate_trajectory(
        self,
        trajectory,
        now=None,
        tcp_fk=None,
        max_start_state_distance_rad=None,
    ):
        """
        Validate a plain-dictionary trajectory and return a report.

        ``trajectory`` has ``joint_names`` and ``points``; each point has
        ``positions``, optional ``velocities``/``accelerations`` and
        ``time_from_start_s``. ``tcp_fk(positions)`` returns the tool point in
        the workspace frame and is required when workspace checking is enabled
        and ``workspace_required`` is true.
        """
        now = self.clock() if now is None else float(now)
        violations = []
        state = self.state_report(now=now)
        for entry in state['violations']:
            violations.append(dict(entry))

        joint_names = list(trajectory.get('joint_names', []))
        points = list(trajectory.get('points', []))
        if not points:
            violations.append(
                _violation('trajectory', 'trajectory has no points')
            )
        if len(points) > self.config.max_points:
            violations.append(
                _violation(
                    'trajectory',
                    'trajectory has {} points > {}'.format(
                        len(points), self.config.max_points
                    ),
                    point_count=len(points),
                    max_points=self.config.max_points,
                )
            )
        for joint in ARM_JOINTS:
            if joint not in joint_names:
                violations.append(
                    _violation(
                        'trajectory',
                        'trajectory is missing {}'.format(joint),
                        joint=joint,
                    )
                )
        if not joint_names or not points:
            return self._finish_validation(now, violations, 0.0, 0, False)

        for index, point in enumerate(points):
            positions = point.get('positions', [])
            if len(positions) != len(joint_names):
                violations.append(
                    _violation(
                        'trajectory',
                        'point {} has {} positions for {} joints'.format(
                            index, len(positions), len(joint_names)
                        ),
                        point_index=index,
                    )
                )
                return self._finish_validation(
                    now, violations, 0.0, len(points), False
                )

        stamps = [
            float(point.get('time_from_start_s', 0.0)) for point in points
        ]
        duration = stamps[-1] - stamps[0]
        for index in range(1, len(stamps)):
            if stamps[index] <= stamps[index - 1] + EPS:
                violations.append(
                    _violation(
                        'timestamp',
                        'point {} time {:.6f} is not after {:.6f}'.format(
                            index, stamps[index], stamps[index - 1]
                        ),
                        point_index=index,
                    )
                )
        if duration > self.config.max_duration_s:
            violations.append(
                _violation(
                    'duration',
                    'duration {:.3f} s > {:.3f} s'.format(
                        duration, self.config.max_duration_s
                    ),
                    duration_s=duration,
                    max_duration_s=self.config.max_duration_s,
                )
            )

        columns = _transpose_points(points, joint_names)
        for joint in ARM_JOINTS:
            if joint not in columns:
                continue
            limit = self.config.joint_limits[joint]
            for index, value in enumerate(columns[joint]):
                if not _finite(value):
                    violations.append(
                        _violation(
                            'joint_limit',
                            '{} point {} is not finite'.format(joint, index),
                            joint=joint,
                            point_index=index,
                            value=value,
                        )
                    )
                    continue
                if (
                    value < limit.lower - self.config.position_tolerance_rad
                    or value
                    > limit.upper + self.config.position_tolerance_rad
                ):
                    violations.append(
                        _violation(
                            'joint_limit',
                            '{} point {} {:.4f} outside [{:.4f}, {:.4f}]'
                            .format(
                                joint, index, value,
                                limit.lower, limit.upper
                            ),
                            joint=joint,
                            point_index=index,
                            value=value,
                            lower=limit.lower,
                            upper=limit.upper,
                        )
                    )

        max_velocity = self._check_rates(
            'velocity', points, joint_names, stamps, violations
        )
        max_acceleration = self._check_rates(
            'acceleration', points, joint_names, stamps, violations
        )

        start_threshold = (
            self.config.max_start_state_distance_rad
            if max_start_state_distance_rad is None
            or max_start_state_distance_rad <= 0.0
            else float(max_start_state_distance_rad)
        )
        if state['valid'] and self.latest_positions is not None:
            start_positions = points[0]['positions']
            distance = 0.0
            for joint in ARM_JOINTS:
                if joint not in joint_names:
                    continue
                index = joint_names.index(joint)
                distance = max(
                    distance,
                    abs(
                        float(start_positions[index])
                        - float(self.latest_positions[joint])
                    ),
                )
            if distance > start_threshold:
                violations.append(
                    _violation(
                        'start_state',
                        'start state differs from latest state by '
                        '{:.4f} rad > {:.4f} rad'.format(
                            distance, start_threshold
                        ),
                        distance_rad=distance,
                        threshold_rad=start_threshold,
                    )
                )

        workspace_checked = False
        if self.config.workspace_enabled:
            if tcp_fk is None:
                if self.config.workspace_required:
                    violations.append(
                        _violation(
                            'workspace',
                            'workspace check is required but no FK is '
                            'available',
                        )
                    )
            else:
                workspace_checked = True
                for index, point in enumerate(points):
                    positions = {
                        name: float(value)
                        for name, value in zip(
                            joint_names, point['positions']
                        )
                    }
                    try:
                        tool_point = tcp_fk(positions)
                    except Exception as exc:  # noqa: BLE001 - FK failure veto
                        violations.append(
                            _violation(
                                'workspace',
                                'FK failed at point {}: {}'.format(
                                    index, exc
                                ),
                                point_index=index,
                            )
                        )
                        break
                    if tool_point is None:
                        violations.append(
                            _violation(
                                'workspace',
                                'FK returned no tool point at point {}'
                                .format(index),
                                point_index=index,
                            )
                        )
                        break
                    for axis, value in enumerate(tool_point):
                        low = self.config.workspace_lower[axis]
                        high = self.config.workspace_upper[axis]
                        if value < low or value > high:
                            violations.append(
                                _violation(
                                    'workspace',
                                    'tool point {} at point {} is {:.4f} '
                                    'outside [{:.4f}, {:.4f}]'.format(
                                        'xyz'[axis], index, value,
                                        low, high
                                    ),
                                    axis='xyz'[axis],
                                    point_index=index,
                                    value=float(value),
                                    lower=low,
                                    upper=high,
                                )
                            )
                            break

        return self._finish_validation(
            now, violations, duration, len(points), workspace_checked,
            max_velocity, max_acceleration,
        )

    def _check_rates(self, kind, points, joint_names, stamps, violations):
        """Check explicit or finite-difference velocity/acceleration."""
        explicit_key = (
            'velocities' if kind == 'velocity' else 'accelerations'
        )
        rates = self._rate_matrix(
            points, joint_names, stamps, explicit_key,
            self._velocity_matrix(points, joint_names, stamps)
            if kind == 'acceleration' else None,
        )
        max_seen = 0.0
        for joint in ARM_JOINTS:
            if joint not in rates:
                continue
            limit = self.config.joint_limits[joint]
            limit_value = (
                limit.max_velocity
                if kind == 'velocity'
                else limit.max_acceleration
            )
            tolerance = (
                self.config.velocity_tolerance_rad_s
                if kind == 'velocity'
                else self.config.acceleration_tolerance_rad_s2
            )
            for index, value in enumerate(rates[joint]):
                if value is None:
                    continue
                if not _finite(value):
                    violations.append(
                        _violation(
                            kind,
                            '{} {} at point {} is not finite'.format(
                                joint, kind, index
                            ),
                            joint=joint,
                            point_index=index,
                            value=value,
                            limit=limit_value,
                        )
                    )
                    continue
                max_seen = max(max_seen, abs(value))
                if abs(value) > limit_value + tolerance:
                    violations.append(
                        _violation(
                            kind,
                            '{} {} at point {} is {:.4f} > {:.4f}'.format(
                                joint, kind, index, value, limit_value
                            ),
                            joint=joint,
                            point_index=index,
                            value=value,
                            limit=limit_value,
                        )
                    )
        return max_seen

    def _rate_matrix(
        self, points, joint_names, stamps, explicit_key, source_matrix=None
    ):
        """Build per-joint rates, preferring explicit values over differences."""
        index = {name: position
                 for position, name in enumerate(joint_names)}
        matrix = {}
        for joint in ARM_JOINTS:
            if joint not in index:
                continue
            column = index[joint]
            explicit = [point.get(explicit_key) for point in points]
            complete = all(
                values is not None and len(values) == len(joint_names)
                for values in explicit
            )
            if complete:
                matrix[joint] = [
                    float(values[column]) for values in explicit
                ]
                continue
            if source_matrix is None:
                values = [0.0] * len(points)
                for point_index in range(1, len(points)):
                    delta_t = stamps[point_index] - stamps[point_index - 1]
                    if delta_t <= EPS:
                        values[point_index] = None
                        continue
                    values[point_index] = (
                        float(points[point_index]['positions'][column])
                        - float(points[point_index - 1]['positions'][column])
                    ) / delta_t
                matrix[joint] = values
            else:
                source = source_matrix.get(joint, [])
                values = [0.0] * len(points)
                for point_index in range(1, len(points)):
                    delta_t = stamps[point_index] - stamps[point_index - 1]
                    if (
                        delta_t <= EPS
                        or point_index >= len(source)
                        or source[point_index] is None
                        or source[point_index - 1] is None
                    ):
                        values[point_index] = None
                        continue
                    values[point_index] = (
                        source[point_index] - source[point_index - 1]
                    ) / delta_t
                matrix[joint] = values
        return matrix

    def _velocity_matrix(self, points, joint_names, stamps):
        """Return the velocity matrix used for acceleration differences."""
        return self._rate_matrix(
            points, joint_names, stamps, 'velocities'
        )

    def _finish_validation(
        self,
        now,
        violations,
        duration,
        point_count,
        workspace_checked,
        max_velocity=0.0,
        max_acceleration=0.0,
    ):
        approved = not violations
        code = 'OK' if approved else _first_code(violations)
        report = {
            'schema_version': 1,
            'approved': approved,
            'code': code,
            'violations': violations,
            'messages': [entry['message'] for entry in violations],
            'duration_s': duration,
            'point_count': point_count,
            'workspace_checked': workspace_checked,
            'workspace_frame': self.config.workspace_frame,
            'max_velocity_rad_s': max_velocity,
            'max_acceleration_rad_s2': max_acceleration,
            'checked_at_s': now,
        }
        self.last_validation = {
            'approved': approved,
            'code': code,
            'violation_count': len(violations),
            'checked_at_s': now,
        }
        return report


def _first_code(violations):
    """Return the most specific high-level failure code."""
    checks = {entry['check'] for entry in violations}
    for check, code in (
        ('state', 'STATE_INVALID'),
        ('joint_limit', 'JOINT_LIMIT'),
        ('velocity', 'VELOCITY_LIMIT'),
        ('acceleration', 'ACCELERATION_LIMIT'),
        ('workspace', 'WORKSPACE'),
        ('start_state', 'START_STATE'),
        ('duration', 'DURATION'),
        ('timestamp', 'TIMESTAMP'),
        ('trajectory', 'TRAJECTORY_INVALID'),
    ):
        if check in checks:
            return code
    return 'MULTIPLE'
