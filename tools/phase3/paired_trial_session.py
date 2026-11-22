#!/usr/bin/env python3
"""Run a paired baseline/intervention grasp session on the real robot.

The experiment unit is a matched pair (see ``docs/phase3/PAIRED_TRIALS.md``).
For every pair the operator places the same cube at the same floor fixture and
the session runs both methods in the randomized order of the plan:

``baseline``
    Target = the measured top-face point, no low-configuration correction.
    This is the method that failed passes 3-4 on 2026-11-19.
``intervention``
    Target = the measured point + the grasp-validated constant correction
    ``(+13.7, +3.9, +8.9) mm``.

Per trial the runner measures the cube at the view pose, fits the deep-family
joints (``grasp_joint_fit``), grasps, checks whether the floor is empty, and
for a held cube releases it back and measures the landing displacement. Every
trial is appended to the paired-trial JSONL through the validated recorder, so
the session is resumable.

Run on the laptop (needs ``sg docker`` for the Humble container and an SSH key
for the robot)::

    sg docker -c "python3 tools/phase3/paired_trial_session.py \
        --pairs 24 --seed 7 --fixture floor_square_A \
        --plan evidence/phase3/onsite_20261119/paired_trials_plan.json \
        --trials evidence/phase3/onsite_20261119/trials.jsonl"

Add ``--dry-run`` to exercise the control flow and the recorder with synthetic
measurements and no robot calls.
"""

import argparse
from datetime import datetime, timezone
import json
import os
import shlex
import subprocess
import sys
import time

import numpy as np

from _bootstrap import REPO, bootstrap  # noqa: E402
bootstrap()

import grasp_joint_fit  # noqa: E402
from armpi_analysis import paired_trials  # noqa: E402

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
DEFAULT_VIEW_POSE = [0.0, 0.2574, -1.18773, -1.70, -0.01676]
DEFAULT_CORRECTION_MM = [13.7, 3.9, 8.9]
DEFAULT_SSH_IDENTITY = '~/.ssh/id_ed25519_armpi'
#: Wider region for the home landing: the cube released at the start can sit
#: slightly outside the default view-pose region; a cube pushed out of the
#: grip box during the grasp is still absent and is recorded as NO_GRIP.
LANDING_WIDE_REGION = ('--x-range', '0.12,0.45', '--y-range', '0.22',
                       '--max-xy', '0.12')
#: The 2026-11-20 far-run home release landed 11.06 mm from the fitted target;
#: a landing farther than this from the start release target means the cube was
#: not actually carried home (a false grip), not a release repeatability tail.
HOME_RELEASE_MAX_MM = 60.0
#: Release gripper open: the vendor servo tracks the commanded interpolation at
#: about 100 pulses/s, so a full open from the grip (~550) to open (~183) needs
#: about 3 s. The tool's old 0.45 s default stopped at ~420 and the cube stayed
#: in the fingers (2026-11-21 p015), so the release would read as NO_GRIP.
RELEASE_OPEN_DURATION_S = 3.5
#: A release whose open register stays above this (open reads ~183) means the
#: fingers never cleared the cube; the trial is flagged for the operator.
RELEASE_OPEN_MAX_REGISTER = 230
ROBOT_TOOLS = ('bus_pulses.py', 'move_to_pose.py',
               'low_config_grasp_validate.py', 'fast_motion.py',
               'motion_profile.py', 'vendor_servo_map.py',
               'base_pulse_vector.py', 'ext_cam_capture.py')


def now_iso():
    """Return the current UTC time as an ISO-8601 string."""
    return datetime.now(timezone.utc).isoformat()


def run_command(command, timeout, what):
    """Run one command and fail loudly with its output on error."""
    started = time.time()
    completed = subprocess.run(
        command, capture_output=True, text=True, timeout=timeout)
    if completed.returncode != 0:
        raise RuntimeError(
            '{} failed ({} s, rc {}):\nstdout: {}\nstderr: {}'.format(
                what, round(time.time() - started, 1), completed.returncode,
                completed.stdout[-2000:], completed.stderr[-2000:]))
    return completed.stdout


class RobotLink:
    """The robot's vendor container over SSH.

    The robot hosts two keys: the interactive ``id_ed25519_armpi`` and the
    forced-command ``id_ed25519_armpi_clock`` used by the laptop clock-sync
    timer. If both are in the SSH agent, ``ssh pi@...`` may offer the clock
    key first and run the clock's ``get|set`` script instead of the command.
    The explicit identity below makes every robot command unambiguous.
    """

    def __init__(self, host, container, identity=None):
        self.host = host
        self.container = container
        self.identity = os.path.expanduser(
            identity or DEFAULT_SSH_IDENTITY)

    def ssh_prefix(self):
        """Return the ssh command prefix with the interactive identity."""
        return ['ssh', '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
                '-i', self.identity]

    def scp_prefix(self):
        """Return the scp command prefix with the interactive identity."""
        return ['scp', '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
                '-i', self.identity]

    def exec(self, arguments, timeout=90.0):
        """Run one command in the robot container and return stdout."""
        inner = 'source ~/.zshrc >/dev/null 2>&1; exec ' + ' '.join(
            shlex.quote(str(value)) for value in arguments)
        remote = 'sudo docker exec -u ubuntu {} /bin/zsh -c {}'.format(
            self.container, shlex.quote(inner))
        return run_command(
            self.ssh_prefix() + [self.host, remote], timeout,
            'robot command {}'.format(arguments[0]))

    def install_tools(self, local_tools):
        """Copy the container-side helper scripts into the container."""
        for name in ROBOT_TOOLS:
            local = os.path.join(local_tools, name)
            inner = ('rm -f /tmp/{0}; cat > /tmp/{0}; chmod 644 /tmp/{0}'
                     .format(name))
            remote = 'sudo docker exec -i -u root {} /bin/zsh -c {}'.format(
                self.container, shlex.quote(inner))
            with open(local, 'rb') as handle:
                completed = subprocess.run(
                    self.ssh_prefix() + [self.host, remote], stdin=handle,
                    capture_output=True, timeout=30)
            if completed.returncode != 0:
                raise RuntimeError('copy {} failed: {}'.format(
                    name, completed.stderr[-500:]))

    def pulses(self):
        """Read the five arm registers."""
        text = self.exec(['python3', '/tmp/bus_pulses.py'], timeout=40)
        values = [int(value) for value in text.strip().split(',') if value]
        if len(values) != 5:
            raise RuntimeError('unexpected pulse output: {!r}'.format(text))
        return values

    def base_pulse(self, vx, vy, duty, duration, allow_long=False):
        """Drive one open-loop base pulse and return its JSON report."""
        arguments = ['python3', '/tmp/base_pulse_vector.py',
                     '--vx={:.6f}'.format(float(vx)),
                     '--vy={:.6f}'.format(float(vy)),
                     '--duty={:.3f}'.format(float(duty)),
                     '--duration={:.4f}'.format(float(duration)), '--yes']
        if allow_long:
            arguments.append('--allow-long-pulse')
        text = self.exec(arguments, timeout=60)
        return json.loads(text[text.index('{'):])

    def move_to_pose(self, pose, tag='move'):
        """Move the arm to one pose and return the run report."""
        output = '/tmp/{}_{}.json'.format(tag, int(time.time()))
        self.exec(['python3', '/tmp/move_to_pose.py',
                   '--pose=' + ','.join('{:.6f}'.format(v) for v in pose),
                   '--output', output, '--i-have-clear-workspace'],
                  timeout=120)
        return self.read_json(output)

    def grasp(self, grasp_joints, view_pose, tag, yaw=0.0):
        """Grasp, close and rise to the view pose."""
        output = '/tmp/grasp_{}.json'.format(tag)
        self.exec(['python3', '/tmp/low_config_grasp_validate.py',
                   '--grasp=' + ','.join('{:.6f}'.format(v)
                                         for v in grasp_joints[:4]),
                   '--yaw={:.6f}'.format(float(yaw)),
                   '--verify-pose=' + ','.join('{:.6f}'.format(v)
                                               for v in view_pose),
                   '--output', output, '--i-have-clear-workspace'],
                  timeout=180)
        report = self.read_json(output)
        if not report.get('ok'):
            raise RuntimeError('grasp sequence failed: {}'.format(
                report.get('error')))
        return report

    def release(self, grasp_joints, view_pose, tag, yaw=0.0):
        """Lower a held cube back to the grasp pose and open."""
        output = '/tmp/release_{}.json'.format(tag)
        self.exec(['python3', '/tmp/low_config_grasp_validate.py',
                   '--grasp=' + ','.join('{:.6f}'.format(v)
                                         for v in grasp_joints[:4]),
                   '--yaw={:.6f}'.format(float(yaw)),
                   '--verify-pose=' + ','.join('{:.6f}'.format(v)
                                               for v in view_pose),
                   '--release-only', '--output', output,
                   '--release-open-duration={:.3f}'.format(
                       RELEASE_OPEN_DURATION_S),
                   '--i-have-clear-workspace'], timeout=180)
        report = self.read_json(output)
        if not report.get('ok'):
            raise RuntimeError('release sequence failed: {}'.format(
                report.get('error')))
        for step in report.get('steps') or []:
            if step.get('step') == 'open':
                report['release_open_register'] = step.get('register')
                break
        return report

    def read_json(self, path):
        """Read a JSON file from the container without copying it."""
        text = self.exec(['cat', path], timeout=20)
        return json.loads(text)


class HumbleLink:
    """The laptop's Humble container (segmentation)."""

    def __init__(self, container, hand_eye, repo):
        self.container = container
        self.hand_eye = hand_eye
        self.repo = repo

    def docker(self, arguments, timeout=90.0, check=True):
        """Run one command in the Humble container and return stdout."""
        command = 'docker exec {} {}'.format(
            self.container,
            ' '.join(shlex.quote(str(value)) for value in arguments))
        if check:
            return run_command(['sg', 'docker', '-c', command], timeout,
                               'container command {}'.format(arguments[0]))
        completed = subprocess.run(
            ['sg', 'docker', '-c', command], capture_output=True, text=True,
            timeout=timeout)
        return completed

    def install_hand_eye(self, source):
        """Copy the hand-eye calibration into the container."""
        run_command(['sg', 'docker', '-c',
                     'docker cp {} {}:{}'.format(
                         source, self.container, self.hand_eye)],
                    30, 'hand-eye copy')

    def segment(self, pulses, tag, extra_args=None):
        """Segment the cube at the current pose; return the JSON report.

        ``extra_args`` extends the segmenter command line (e.g. a wider
        ``--x-range`` for a cube that is still too far to grasp).
        """
        output = '/tmp/seg_{}.json'.format(tag)
        inner = (
            'source /opt/ros/humble/setup.bash; '
            'source /workspace/install/setup.bash 2>/dev/null; '
            'python3 /repo_tools/phase2/object_segment_rgbd.py '
            '--output {out} --hand-eye {he} --joints-pulses {pulses}'
        ).format(
            out=output,
            he=self.hand_eye,
            pulses=','.join(str(value) for value in pulses),
        )
        if extra_args:
            inner += ' ' + ' '.join(
                str(shlex.quote(value)) for value in extra_args)
        # The segmenter exits 1 when it finds no acceptable target, which is a
        # valid measurement result (a held cube leaves the floor empty).
        completed = self.docker(
            ['bash', '-lc', inner], timeout=90, check=False)
        try:
            text = self.docker(['cat', output], timeout=20)
        except RuntimeError:
            raise RuntimeError(
                'segmentation produced no report: {}'.format(
                    completed.stderr[-800:]))
        return json.loads(text)


def validate_target(target, fit, max_xy_error_mm, reach=(0.10, 0.30)):
    """Refuse an implausible target or a failed joint fit.

    A unit error in the correction (millimetres added to metres) produced a
    13.9 m target and a 14 m joint-fit residual on the first 2026-11-19
    attempt. This guard makes that class of bug stop before any motion.
    """
    target = np.asarray(target, dtype=np.float64)
    if not np.all(np.isfinite(target)):
        raise ValueError('target is not finite')
    radius = float(np.linalg.norm(target[:2]))
    if not reach[0] <= radius <= reach[1]:
        raise ValueError(
            'target radius {:.3f} m is outside {}-{} m'.format(
                radius, reach[0], reach[1]))
    if fit['xy_error_mm'] > max_xy_error_mm:
        raise ValueError(
            'joint fit xy error {:.2f} mm exceeds {:.2f} mm; refusing to '
            'move'.format(fit['xy_error_mm'], max_xy_error_mm))
    if fit['reach_limited']:
        raise ValueError('joint4 is at its low limit; target out of reach')


def placement_quality(report, min_height=0.030, max_height=0.050,
                      max_extent=0.050):
    """Return (ok, reason) for a pre-trial cube placement.

    The cube is 39.4 mm tall and was measured at 39.3-39.6 mm above the floor
    when flat. A taller cluster means it is on top of something (the 2026-11-19
    p019 baseline sat at 59.3 mm on a ~20 mm object), and a wider cluster means
    the cube merged with a neighbour (p023: 53.7 x 72.4 mm). Both are protocol
    violations and must be re-placed, not grasped.
    """
    selected = report.get('selected') or {}
    height = selected.get('height_above_floor_m')
    size = (selected.get('bounds') or {}).get('size')
    if height is None or size is None:
        return False, 'no height/size'
    if not min_height <= float(height) <= max_height:
        return False, 'height {:.1f} mm outside {:.0f}-{:.0f}'.format(
            float(height) * 1000.0, min_height * 1000.0, max_height * 1000.0)
    for axis, value in zip('xy', size[:2]):
        if not 0.030 <= float(value) <= max_extent:
            return False, '{} extent {:.1f} mm outside 30-{:.0f}'.format(
                axis, float(value) * 1000.0, max_extent * 1000.0)
    return True, 'ok'


def release_anchor(point, fit, moved_home):
    """Return the reference point for the release displacement.

    The site flow measures the put-back at the cube's original spot, so the
    anchor is the pre-grasp measurement. The home flow carries the cube back
    to the start and releases there, where the original floor spot no longer
    applies; the release precision is measured against the fitted TCP.
    """
    if moved_home:
        return np.asarray(fit['tcp'], dtype=np.float64)
    return np.asarray(point, dtype=np.float64)


def cube_box_present(report, point, radius=0.04, min_height=0.01):
    """Return True when a cluster sits in the box around the expected cube.

    The 2026-11-19 first attempt classified a grip as a miss because the
    segmenter counts every cluster above the floor, including objects far
    outside the grasp region (a 0.50 m background cluster). The grip test must
    ask about the cube's own box, which is the check the validated runs used
    ("0 points in the cube box").
    """
    point = np.asarray(point, dtype=np.float64)
    for cluster in report.get('segmentation', {}).get('cluster_details', []):
        centroid = np.asarray(cluster['bounds']['centroid'])
        if (abs(centroid[0] - point[0]) <= radius
                and abs(centroid[1] - point[1]) <= radius
                and float(cluster.get('height_m', 0.0)) >= min_height):
            return True
    return False


def cube_point(report):
    """Return the measured cube point, or None.

    A valid top face gives ``grasp_point_base``. A clipped view is rejected as
    a grasp target but keeps ``rejected_point_base``; the session uses it only
    for the landing measurement and records that caveat.
    """
    selected = report.get('selected') or {}
    if selected.get('grasp_point_base') is not None:
        return np.asarray(selected['grasp_point_base'], dtype=np.float64), True
    if selected.get('rejected_point_base') is not None:
        return np.asarray(selected['rejected_point_base'],
                          dtype=np.float64), False
    return None, False


class Session:
    """One paired-trial session."""

    def __init__(self, args):
        self.args = args
        correction_mm = np.asarray(
            [float(v) for v in args.correction.split(',')], dtype=np.float64)
        if correction_mm.size != 3:
            raise SystemExit('--correction needs three millimetres values')
        if np.max(np.abs(correction_mm)) > 50.0:
            raise SystemExit('--correction looks too large for millimetres')
        self.correction = correction_mm / 1000.0
        self.view_pose = [float(v) for v in args.view_pose.split(',')]
        self.plan = self._load_plan()
        self.done = self._load_done()
        self.robot = RobotLink(
            args.robot, args.robot_container, args.robot_identity)
        self.humble = HumbleLink(
            args.humble_container, args.hand_eye_container, REPO)
        self.counter = 0

    def _load_plan(self):
        path = self.args.plan
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as handle:
                return json.load(handle)
        pair_ids = ['p{:03d}'.format(index + 1)
                    for index in range(self.args.pairs)]
        plan = paired_trials.build_plan(
            pair_ids, fixtures=[self.args.fixture], seed=self.args.seed)
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.',
                    exist_ok=True)
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(plan, handle, indent=2)
        print('wrote the randomization plan: {}'.format(path))
        return plan

    def _load_done(self):
        if not os.path.exists(self.args.trials):
            return set()
        records = paired_trials.load_records(self.args.trials)
        return {(record['pair_id'], record['method']) for record in records}

    def prepare(self):
        """Install the container helpers and put the arm at the view pose."""
        if self.args.dry_run:
            return
        self.robot.install_tools(os.path.join(REPO, 'tools', 'phase2'))
        self.humble.install_hand_eye(os.path.join(
            REPO, self.args.hand_eye_host))
        print('正在移动到 view pose ...')
        self.robot.move_to_pose(self.view_pose, tag='view')
        print('手臂已在 view pose。')

    def measure(self, pulses, tag, retries=3):
        """Measure the cube, prompting the operator to re-place if needed."""
        for attempt in range(retries):
            report = self.humble.segment(pulses, tag)
            point, valid = cube_point(report)
            ok, reason = placement_quality(report)
            if valid and ok:
                return point, report
            if valid and not ok:
                print('放置不合格：{}'.format(reason))
            clusters = report.get('segmentation', {}).get('clusters', 0)
            print('测量不可用 '
                  '（success={}，clusters={}，{}）'.format(
                      report.get('success'), clusters,
                      report.get('rejection', {}).get('reason')))
            if attempt + 1 < retries and not self.args.dry_run:
                input('  请调整木块，然后按 Enter 重新测量 ... ')
                pulses = self.robot.pulses()
        return None, report

    def _before_release(self, entry, fit, grasp_report):
        """Hook: move the robot before the release; return True if moved.

        The base runner does not move (the validated site put-back). The
        free-placement runner returns the base home while holding the cube
        and releases there.
        """
        return False

    def trial(self, entry, prompt=True):
        """Run one trial and append its record.

        ``prompt=False`` is used by the site-diversity runner: the cube is
        already placed on the site and the arm is already aligned, so the
        per-trial placement prompt would be a no-op.
        """
        method = entry['method']
        tag = '{}_s{:02d}'.format(entry['pair_id'], entry['sequence'])
        self.counter += 1
        method_label = {
            'baseline': 'baseline（原始目标）',
            'intervention': 'intervention（常数修正）',
        }.get(method, method)
        print('\n=== [{}] {} / {} ({}) ==='.format(
            self.counter, entry['pair_id'], method_label,
            entry['object_pose_fixture']))
        if self.args.dry_run:
            return self._dry_trial(entry, tag)
        if prompt:
            input('  请把木块放到指定位置，然后按 Enter ... ')
        pre = getattr(self, '_pre_measure', None)
        if pre is not None:
            # The random-site runner just took this measurement at the same
            # arm pose for the approach gate; re-segmenting the identical
            # view only lengthens the gap before the grasp.
            self._pre_measure = None
            point, report = pre
            reused = True
        else:
            pulses = self.robot.pulses()
            point, report = self.measure(pulses, tag + '_pre')
            reused = False
        if point is None:
            raise RuntimeError(
                'no usable pre-trial measurement for {}'.format(tag))
        target = point + (self.correction if method == 'intervention'
                          else np.zeros(3))
        fit = grasp_joint_fit.fit(target)
        validate_target(target, fit, self.args.max_xy_error_mm)
        print('  木块  ({:+.4f}, {:+.4f}, {:+.4f}) m'.format(*point))
        print('  目标  ({:+.4f}, {:+.4f}, {:+.4f}) m'.format(*target))
        print('  关节 {}  xy 误差 {:.2f} mm'.format(
            ','.join('{:.4f}'.format(v) for v in fit['joints'][:4]),
            fit['xy_error_mm']))
        arrival = getattr(self, '_base_arrival_s', None)
        gap_s = None if arrival is None else time.monotonic() - arrival
        if gap_s is not None:
            print('  到达后 {:.1f} s 开始抓取{}'.format(
                gap_s, '（复用最终测量）' if reused else ''),
                flush=True)
        grasp_report = self.robot.grasp(fit['joints'], self.view_pose, tag)
        grip_pulses = [
            grasp_report['pulses_at_verify'][str(servo)]
            for servo in (6, 5, 4, 3, 2)
        ]
        grip_report = self.humble.segment(grip_pulses, tag + '_grip')
        clusters = grip_report.get('segmentation', {}).get('clusters', 0)
        in_box = cube_box_present(grip_report, point)
        held = not in_box
        notes = {
            'cube_point': [float(v) for v in point],
            'target': [float(v) for v in target],
            'joints': fit['joints'],
            'xy_error_mm': fit['xy_error_mm'],
            'residual_mm': fit['residual_mm'],
            'grip_clusters': clusters,
            'grip_cube_box': bool(in_box),
            'grip_success': grip_report.get('success'),
            'pre_seg_ok': report.get('success'),
            'pre_measure_reused': reused,
            'pre_grasp_gap_s': (None if gap_s is None
                                else round(gap_s, 2)),
        }
        notes['grip_rejection'] = grip_report.get('rejection')
        notes['grip_selected'] = bool(grip_report.get('selected'))
        # The site-diversity runner attaches its locate/approach record here.
        notes.update(getattr(self, 'extra_notes', None) or {})
        if not held:
            code = 'MISS'
            if clusters > 0:
                self._operator_note(
                    '木块框里还有点；请确认木块没有被夹起')
            self._record(entry, success=False, error_mm=None,
                         failure_code=code, notes=notes)
            return False
        self._operator_note('木块已夹住；手臂会把它带回起点放下')
        moved_home = self._before_release(entry, fit, grasp_report)
        release_report = self.robot.release(
            fit['joints'], self.view_pose, tag)
        open_register = release_report.get('release_open_register')
        notes['release_open_register'] = open_register
        if (open_register is not None
                and open_register > RELEASE_OPEN_MAX_REGISTER):
            self._operator_note(
                '释放时夹爪只开到 {}，可能还夹着木块；请检查'.format(
                    open_register))
        land_pulses = [
            release_report['pulses_at_verify'][str(servo)]
            for servo in (6, 5, 4, 3, 2)
        ]
        land_report = self.humble.segment(
            land_pulses, tag + '_land',
            extra_args=LANDING_WIDE_REGION if moved_home else None)
        landing, valid = cube_point(land_report)
        land_ok, land_reason = placement_quality(land_report)
        rejection_reason = (land_report.get('rejection') or {}).get('reason')
        partial_home = (moved_home and landing is not None and land_ok
                        and not valid)
        if moved_home and landing is None:
            # A held claim whose cube is absent from the start means the grip
            # did not actually hold (the cube was pushed out of the check box)
            # or the release failed to drop. Record a miss; do not stop the
            # whole session.
            notes['landing_missing'] = True
            notes['release_frame'] = 'home'
            self._operator_note('起点没有看到木块；按未夹住记录')
            self._record(entry, success=False, error_mm=None,
                         failure_code='NO_GRIP', notes=notes)
            return False
        if partial_home:
            # The start release often clips the top face at the image border.
            # The cluster is still cube-sized and cube-height (land_ok), so
            # the cube is at the start; accept the success with the clipped
            # centroid. The release error is a biased secondary value.
            notes['landing_partial'] = True
            notes['landing_rejection'] = rejection_reason
            self._operator_note(
                '起点顶面被裁切；按成功记录（释放误差为近似值）')
        elif landing is None or not valid or not land_ok:
            # p003 on 2026-11-19 landed near a 22 mm object and the segmenter
            # selected it; a landing must pass the same cube gates as a
            # pre-trial placement or the continuous error is meaningless.
            # Always print the segmenter's real rejection reason.
            reason = (rejection_reason or land_reason
                      if landing is not None else 'no point')
            raise RuntimeError(
                'landing measurement rejected ({}); the trial is not '
                'recorded, re-place the cube and re-run it'.format(reason))
        anchor = release_anchor(point, fit, moved_home)
        error_mm = float(np.linalg.norm((landing - anchor)[:2]) * 1000.0)
        notes['landing'] = [float(v) for v in landing]
        notes['landing_measured'] = True
        notes['landing_valid'] = bool(valid)
        notes['release_frame'] = 'home' if moved_home else 'site'
        notes['release_target'] = [float(v) for v in anchor]
        notes['landing_height_mm'] = float(
            (land_report.get('selected') or {}).get(
                'height_above_floor_m', 0.0)) * 1000.0
        if moved_home and error_mm > HOME_RELEASE_MAX_MM:
            # A cube on the floor near the start but far from the release
            # target is the operator's original spot, not the carried cube.
            notes['landing_far_mm'] = error_mm
            self._operator_note(
                '落点离起点释放目标 {:.0f} mm，按未夹住记录'.format(error_mm))
            self._record(entry, success=False, error_mm=None,
                         failure_code='NO_GRIP', notes=notes)
            return False
        self._record(entry, success=True, error_mm=error_mm,
                     failure_code=None, notes=notes)
        print('  已夹住并在{}放下；释放误差 {:.2f} mm'.format(
            '起点' if moved_home else '原地', error_mm))
        return True

    def _operator_note(self, text):
        """Print an operator instruction without blocking in dry-run."""
        print('  操作员：{}'.format(text))

    def _record(self, entry, success, error_mm, failure_code, notes):
        """Append one validated record."""
        record = {
            'pair_id': entry['pair_id'],
            'trial_order': entry['trial_order'],
            'method': entry['method'],
            'object_pose_fixture': entry['object_pose_fixture'],
            'continuous_error_mm': error_mm,
            'grasp_success': bool(success),
            'failure_code': failure_code,
            'recorded_at': now_iso(),
            'notes': json.dumps(notes, sort_keys=True),
        }
        paired_trials.append_record(self.args.trials, record, plan=self.plan)
        self.done.add((entry['pair_id'], entry['method']))
        print('  已记录 {} / {}：成功={} 误差={}'.format(
            entry['pair_id'], entry['method'], success,
            'n/a' if error_mm is None else '{:.2f} mm'.format(error_mm)))

    def _dry_trial(self, entry, tag):
        """Exercise the control flow without the robot."""
        point = np.asarray([0.2300, 0.0000, -0.0506])
        target = point + (self.correction if entry['method'] == 'intervention'
                          else np.zeros(3))
        fit = grasp_joint_fit.fit(target)
        notes = {
            'dry_run': True,
            'cube_point': point.tolist(),
            'target': target.tolist(),
            'joints': fit['joints'],
            'xy_error_mm': fit['xy_error_mm'],
            'residual_mm': fit['residual_mm'],
        }
        notes.update(getattr(self, 'extra_notes', None) or {})
        if entry['method'] == 'baseline':
            self._record(entry, success=False, error_mm=None,
                         failure_code='MISS', notes=notes)
            return False
        self._record(entry, success=True, error_mm=2.0,
                     failure_code=None, notes=notes)
        return True

    def run(self):
        """Run every pending trial in plan order."""
        self.prepare()
        pending = [entry for entry in self.plan['entries']
                   if (entry['pair_id'], entry['method']) not in self.done]
        print('{} pending trials of {}'.format(
            len(pending), len(self.plan['entries'])))
        if self.args.max_trials:
            pending = pending[:self.args.max_trials]
        for entry in pending:
            self.trial(entry)
        print('\n{} trials recorded in {}'.format(
            len(self.done), self.args.trials))


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pairs', type=int, default=24)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--fixture', default='floor_square_A')
    parser.add_argument(
        '--plan',
        default='evidence/phase3/onsite_20261119/paired_trials_plan.json')
    parser.add_argument(
        '--trials',
        default='evidence/phase3/onsite_20261119/trials.jsonl')
    parser.add_argument(
        '--correction',
        default=','.join(str(v) for v in DEFAULT_CORRECTION_MM),
        help='constant correction in millimetres for the intervention')
    parser.add_argument(
        '--view-pose', default=','.join(str(v) for v in DEFAULT_VIEW_POSE),
        help='five joints of the measurement pose')
    parser.add_argument(
        '--hand-eye-host',
        default='evidence/calibration/20261118/table_down/'
                'calibration_table_down_20261118.json',
        help='hand-eye calibration on the host')
    parser.add_argument(
        '--hand-eye-container', default='/tmp/hand_eye_table_down.json',
        help='hand-eye path inside the Humble container')
    parser.add_argument('--robot', default='pi@10.42.0.218')
    parser.add_argument('--robot-container', default='ArmPiUltra')
    parser.add_argument(
        '--robot-identity', default=DEFAULT_SSH_IDENTITY,
        help='interactive SSH key for the robot (not the clock-sync key)')
    parser.add_argument('--humble-container', default='armpi-humble')
    parser.add_argument('--max-trials', type=int, default=0)
    parser.add_argument(
        '--max-xy-error-mm', type=float, default=3.0,
        help='refuse to move when the joint fit xy error exceeds this')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='synthetic measurements, no robot or container calls')
    return parser


def main(argv=None):
    """Run the session."""
    args = build_parser().parse_args(argv)
    if not args.dry_run and not sys.stdin.isatty():
        raise SystemExit('the session needs an interactive operator prompt')
    session = Session(args)
    try:
        session.run()
    except KeyboardInterrupt:
        print('\n已中断；已完成的 trial 都已记录')
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
