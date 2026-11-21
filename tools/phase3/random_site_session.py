#!/usr/bin/env python3
"""Run the 24-pair trial with free operator placements.

This is the 2026-11-21 follow-up: the operator places the same 39.4 mm cube
**by eye at a new spot** in the forward workspace before every pair; nothing
is pre-taped, pre-drawn or measured. The robot locates the cube, decides
whether it is already in the grasp band (direct grasp) or needs the base to
drive toward it, and runs the pair's two randomized methods. The realized
point, not the operator's intent, is the recorded site.

Locator chain per sighting:

1. the validated view-pose wrist measurement with the wide region;
2. the chassis RealSense D435i, which is the only sensor that sees the far
   part of the workspace (0.46-0.55 m) from the home pose. Its depth top-face
   point is used only to drive the base into the wrist band; the final grasp
   target is always the strict wrist measurement. It is tried before the yaw
   sweep because one capture is shorter than the eight-step sweep;
3. a ``joint1`` yaw sweep (the 2026-11-19 out-of-view demo path) for the near
   wide bearings that the front camera cannot see.

The radius/bearing envelope in ``region.json`` is a guide to where the
sensors see the cube, not a gate. The operator does not measure; a point too
far/wide to locate is moved a little closer.

Run on the laptop::

    sg docker -c "python3 tools/phase3/random_site_session.py \
        --region evidence/phase3/onsite_20261121_random/region.json \
        --plan evidence/phase3/onsite_20261121_random/paired_trials_plan.json \
        --trials evidence/phase3/onsite_20261121_random/trials.jsonl"

``--precheck`` validates the locator chain (especially the front camera at
the far corners) without moving the base or recording a trial. ``--dry-run``
exercises the flow with synthetic points and no robot calls.
"""

import json
import math
import os
import subprocess
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'tools', 'phase2'))
sys.path.insert(0, os.path.join(REPO, 'tools', 'phase3'))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis'))

import paired_trial_session as base  # noqa: E402
import random_region  # noqa: E402
import site_diversity_session as sites  # noqa: E402
from armpi_analysis import paired_trials  # noqa: E402

DEFAULT_REGION = 'evidence/phase3/onsite_20261121_random/region.json'
DEFAULT_PLAN = ('evidence/phase3/onsite_20261121_random/'
                'paired_trials_plan.json')
DEFAULT_TRIALS = 'evidence/phase3/onsite_20261121_random/trials.jsonl'
DEFAULT_SEARCH_YAWS = '0.14,-0.14,0.28,-0.28,0.42,-0.42,0.56,-0.56'
DEFAULT_FRONT_WORK = '/tmp/armpi_random_front'
CALIB = 'evidence/calibration/20261120'
DEFAULT_FRONT_INTRINSICS = CALIB + '/uvc_color_intrinsics_refined.json'
DEFAULT_FRONT_EXTRINSIC = CALIB + '/front_camera_extrinsic.json'
DEFAULT_FRONT_DEPTH_TO_COLOR = CALIB + '/depth_to_color_extrinsics.json'
FRONT_MAX_AGE_S = 20.0
FRONT_MAX_SKEW_S = 1.0
LOCATOR_ZH = {
    'wrist_view': '腕相机（view pose）',
    'front_camera': '前相机（D435i 深度）',
    'wrist_yaw': '腕相机扫视',
    'none': '无',
}


class RandomSiteSession(sites.SiteSession):
    """A paired session with random placements and a front fallback."""

    def __init__(self, args):
        self.region_payload = random_region.load_region(args.region)
        self.sites = {
            fixture: {'site_id': fixture, 'random': True}
            for fixture in self.region_payload['fixtures']}
        self.extra_notes = {}
        self.approach = {}
        self.search_yaws = [
            float(value) for value in args.search_yaws.split(',') if value]
        self.base_tolerance = float(args.base_tolerance)
        self.max_j1_rad = float(args.max_j1_rad)
        envelope = [float(value) for value in args.x_envelope.split(',')
                    if value]
        if len(envelope) != 2 or envelope[0] >= envelope[1]:
            raise SystemExit('--x-envelope needs two increasing metres')
        self.x_envelope = (envelope[0], envelope[1])
        region = self.region_payload['region']
        radius_low = float(region['min_radius_m'])
        radius_high = float(region['max_radius_m'])
        bearing_low = math.radians(region['min_bearing_deg'])
        bearing_high = math.radians(region['max_bearing_deg'])
        corners = [
            (radius * math.cos(bearing), radius * math.sin(bearing))
            for radius in (radius_low, radius_high)
            for bearing in (bearing_low, bearing_high)]
        xs = [point[0] for point in corners]
        ys = [point[1] for point in corners]
        if bearing_low <= 0.0 <= bearing_high:
            xs.append(radius_high)
        self.front_x_range = '{:.3f},{:.3f}'.format(
            min(xs) - 0.015, max(xs) + 0.015)
        self.front_y_range = '{:.3f}'.format(
            max(abs(value) for value in ys) + 0.015)
        self.approaches_path = os.path.join(
            os.path.dirname(os.path.abspath(args.trials)),
            'approaches.jsonl')
        self.front_work = args.front_work
        self.front_evidence = os.path.join(
            os.path.dirname(os.path.abspath(args.trials)), 'front')
        if int(args.pairs) != len(self.sites):
            raise SystemExit(
                '--pairs {} does not match the {} random fixtures'.format(
                    args.pairs, len(self.sites)))
        base.Session.__init__(self, args)

    def _load_plan(self):
        """Load or pre-register the random-fixture paired plan."""
        path = self.args.plan
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as handle:
                plan = json.load(handle)
            fixtures = {entry['object_pose_fixture']
                        for entry in plan.get('entries', [])}
            if fixtures != set(self.sites):
                raise SystemExit(
                    'plan fixtures {} do not match the region {}'.format(
                        sorted(fixtures), sorted(self.sites)))
            expected = {
                'p{:03d}'.format(index + 1): fixture
                for index, fixture in enumerate(self.sites)}
            actual = {
                pair['pair_id']: pair['object_pose_fixture']
                for pair in plan.get('pairs', [])}
            if actual != expected:
                raise SystemExit(
                    'plan pair-to-fixture assignment does not match the '
                    'pre-registered pNNN <-> rNNN mapping')
            return plan
        pair_ids = ['p{:03d}'.format(index + 1)
                    for index in range(len(self.sites))]
        plan = paired_trials.build_plan(
            pair_ids, fixtures=list(self.sites), seed=self.args.seed)
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.',
                    exist_ok=True)
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(plan, handle, indent=2)
            handle.write('\n')
        print('wrote the random-fixture plan: {}'.format(path))
        return plan

    # -- locator chain ---------------------------------------------------
    def _front_capture(self, tag):
        """Capture one front frame and copy it to the laptop."""
        safe = ''.join(char if char.isalnum() or char in '._-' else '_'
                       for char in tag)
        remote = '/tmp/ext_cam_{}'.format(safe)
        host_remote = '/tmp/ext_cam_host_{}'.format(safe)
        local = os.path.join(self.front_work, safe)
        os.makedirs(self.front_work, exist_ok=True)
        subprocess.run(['rm', '-rf', local], check=False)
        try:
            self.robot.exec(['rm', '-rf', remote], timeout=20)
        except Exception:  # noqa: BLE001 - best effort
            pass
        try:
            self.robot.exec([
                'python3', '/tmp/ext_cam_capture.py',
                '--output', remote,
                '--max-skew', str(FRONT_MAX_SKEW_S),
                '--max-age', str(FRONT_MAX_AGE_S)], timeout=60)
        except Exception as error:  # noqa: BLE001 - reported by the caller
            print('  前相机抓取帧失败：{}'.format(error), flush=True)
            return None
        copy = ('sudo rm -rf {host}; '
                'sudo docker cp {container}:{remote} {host}').format(
                    host=host_remote, container=self.robot.container,
                    remote=remote)
        try:
            base.run_command(
                ['ssh', self.robot.host, copy], 60, 'front camera copy')
            base.run_command(
                ['scp', '-r',
                 '{}:{}'.format(self.robot.host, host_remote), local],
                60, 'front camera scp')
        except Exception as error:  # noqa: BLE001 - reported by the caller
            print('  前相机图像拷贝失败：{}'.format(error), flush=True)
            return None
        finally:
            try:
                self.robot.exec(['rm', '-rf', remote], timeout=20)
            except Exception:  # noqa: BLE001 - best effort
                pass
            try:
                base.run_command(
                    ['ssh', self.robot.host,
                     'sudo rm -rf {}'.format(host_remote)],
                    30, 'front host cleanup')
            except Exception:  # noqa: BLE001 - best effort
                pass
        return local

    def _front_measure(self, tag):
        """Measure the cube in base_link from the front camera."""
        local = self._front_capture(tag)
        if local is None:
            return None, None
        output = os.path.join(local, 'measure.json')
        command = [
            sys.executable,
            os.path.join(REPO, 'tools', 'phase2', 'ext_cam_measure.py'),
            '--capture', local,
            '--intrinsics', os.path.join(REPO, self.args.front_intrinsics),
            '--extrinsic', os.path.join(REPO, self.args.front_extrinsic),
            '--depth-to-color',
            os.path.join(REPO, self.args.front_depth_to_color),
            '--x-range', self.front_x_range,
            '--y-range', self.front_y_range,
            '--depth-cluster',
            '--output', output,
        ]
        try:
            base.run_command(command, 60, 'front camera measure')
        except Exception as error:  # noqa: BLE001 - reported by the caller
            print('  前相机测量失败：{}'.format(error), flush=True)
            return None, None
        with open(output, 'r', encoding='utf-8') as handle:
            report = json.load(handle)
        point = report.get('point_base')
        if point is None:
            return None, report
        self._save_front_report(tag, report)
        return np.asarray(point[:3], dtype=np.float64), report

    def _save_front_report(self, tag, report):
        """Keep the front-camera measurement JSON in the evidence directory."""
        safe = ''.join(char if char.isalnum() or char in '._-' else '_'
                       for char in tag)
        os.makedirs(self.front_evidence, exist_ok=True)
        path = os.path.join(self.front_evidence, safe + '.json')
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write('\n')

    def _front_locate(self, tag):
        """Return a front-camera point, or None."""
        for attempt in range(2):
            point, report = self._front_measure(
                '{}_{}'.format(tag, attempt))
            if point is not None:
                print('  前相机测得点 ({:+.3f}, {:+.3f}) m'.format(
                    point[0], point[1]), flush=True)
                return point, False, report
            print('  前相机第 {} 次没有测到点'.format(
                attempt + 1), flush=True)
        return None, False, None

    def _locate_initial(self, tag):
        """Wrist wide -> front camera -> yaw sweep."""
        point, valid, report = self._wide_measure(tag)
        if point is not None:
            return point, valid, report, 'wrist_view'
        # The front camera first: the far half of the region (0.46-0.55 m) is
        # invisible to the wrist at any yaw, and a front capture is shorter
        # than the eight-step yaw sweep. The yaw sweep still covers the near
        # wide bearings that the front camera cannot see.
        point, valid, report = self._front_locate('{}_front'.format(tag))
        if point is not None:
            return point, valid, report, 'front_camera'
        point, valid, report, yaw = self._search(tag)
        self.approach['search_yaw_rad'] = yaw
        return point, valid, report, (
            'wrist_yaw' if point is not None else 'none')

    def _locate_after(self, tag):
        """Wrist wide -> front camera -> yaw sweep after a base pulse."""
        point, valid, report = self._wide_measure(tag)
        if point is not None:
            return point, valid, report, 'wrist_view'
        point, valid, report = self._front_locate('{}_front'.format(tag))
        if point is not None:
            return point, valid, report, 'front_camera'
        point, valid, report, yaw = self._search(tag)
        self.approach['search_yaw_rad'] = yaw
        return point, valid, report, (
            'wrist_yaw' if point is not None else 'none')

    # -- region acceptance ----------------------------------------------
    def _region_check(self, point):
        """Return ``(radius, bearing, inside)`` for a base-frame point."""
        x, y = float(point[0]), float(point[1])
        radius = float(np.hypot(x, y))
        bearing = float(np.degrees(np.arctan2(y, x)))
        region = self.region_payload['region']
        radius_ok = (region['min_radius_m'] - 0.010
                     <= radius <= region['max_radius_m'] + 0.010)
        bearing_ok = (region['min_bearing_deg'] - 1.0
                      <= bearing <= region['max_bearing_deg'] + 1.0)
        return radius, bearing, radius_ok and bearing_ok

    def _validate_initial(self, point, report, site, tag, locator):
        """Report the realized placement; no measuring or re-place gate.

        The operator places the cube by eye. The locator only returns a point
        it actually found, so the realized point is inside the sensor
        coverage by construction; the region is descriptive, not enforced.
        """
        radius, bearing, inside = self._region_check(point)
        print('  已找到木块（{}）：测得点 x={:+.3f}, y={:+.3f} m，'
              '半径 {:.3f} m，方位 {:+.1f}°{}'.format(
                  LOCATOR_ZH.get(locator, locator), point[0], point[1],
                  radius, bearing,
                  '' if inside else '（超出参考区域，但已定位）'), flush=True)
        self.approach['locator'] = locator
        self.approach['realized_inside_region'] = bool(inside)
        return point

    def _dry_approach(self, site):
        """Build a synthetic random approach record for the dry run."""
        index = int(site['site_id'][1:])
        generator = np.random.default_rng(20261121 + index)
        region = self.region_payload['region']
        radius = generator.uniform(
            region['min_radius_m'] + 0.02, region['max_radius_m'] - 0.02)
        bearing = np.deg2rad(generator.uniform(
            region['min_bearing_deg'] + 2.0,
            region['max_bearing_deg'] - 2.0))
        point = np.asarray([
            radius * np.cos(bearing), radius * np.sin(bearing), -0.05])
        plan = sites.approach_plan(
            point, self.args.base_duty, self.base_tolerance)
        pulses = []
        if plan is not None:
            vx, vy, duration = plan
            pulses.append({
                'kind': 'approach',
                'vx': float(vx),
                'vy': float(vy),
                'duty': float(self.args.base_duty),
                'duration': float(duration),
                'nominal_travel_mm': round(
                    self.args.base_duty * duration / 0.182, 3),
                'point_before': [float(value) for value in point],
                'point_after': [0.2300, 0.0, -0.05],
            })
        self.approach['initial_point'] = [float(value) for value in point]
        self.approach['initial_valid'] = True
        self.approach['base_pulses'] = pulses
        self.approach['final_point'] = [0.2300, 0.0, -0.05]
        self.approach['final_j1_rad'] = 0.0
        self.approach['final_xy_error_mm'] = 0.0
        self.approach['final_envelope'] = {
            'x_m': 0.2300, 'j1_rad': 0.0, 'x_ok': True, 'j1_ok': True}
        self.approach['approach_wall_s'] = 0.0

    # -- session loops ---------------------------------------------------
    def run(self):
        """Run every pending pair with a fresh random placement."""
        self.prepare()
        pending = self._pending_pairs()
        print('{} 对未完成，共 {} 对'.format(
            len(pending), self.plan['n_pairs']))
        if self.args.max_pairs:
            pending = pending[:self.args.max_pairs]
        region = self.region_payload['region']
        print('放置方式：你随便放，不用量、不用贴点；尽量放在机器人前方'
              '（大约 0.25–0.55 m、方位约 {:+.0f}…{:+.0f}°，只是参考）。'
              '近的机器人直接夹，远的会先开过去再夹。'.format(
                  region['min_bearing_deg'], region['max_bearing_deg']))
        for index, (pair_id, entries, site) in enumerate(pending):
            self.extra_notes = {}
            self.approach = {}
            print('\n### 第 {} 对（{}/{}），随机点 {}'.format(
                pair_id, index + 1, len(pending), site['site_id']))
            if not self.args.dry_run:
                input('  请把木块随手放在机器人前方一个新位置'
                      '（不用量、不用贴点；不要每次都放同一个点），然后按 '
                      'Enter ... ')
            try:
                self.locate_and_approach(site, pair_id)
                for entry_index, entry in enumerate(entries):
                    if entry_index > 0 and not self.args.dry_run:
                        input('  请把木块放回同一个位置，然后按 '
                              'Enter ... ')
                    self.trial(entry, prompt=False)
            except Exception as exc:
                self.approach['error'] = str(exc)
                raise
            finally:
                try:
                    self.return_home(pair_id)
                finally:
                    self._write_approach(pair_id, site)

    def precheck(self):
        """Free near/far warm-up of the locator chain; no base motion."""
        if self.args.dry_run:
            raise SystemExit('--precheck 需要真实机器人')
        self.prepare()
        prompts = (
            '把木块随便放在近一点的位置（大约 0.3 m 内，不用量），'
            '然后按 Enter ... ',
            '把木块随便放在远一点的位置（尽量远，约 0.5 m，不用量），'
            '然后按 Enter ... ',
        )
        print('预检查（不用量、不用贴点）：先放一个近的、再放一个远的，'
              '看机器人能不能找到。底盘不动，也不记录 trial。')
        for index, prompt in enumerate(prompts, start=1):
            self.robot.move_to_pose(
                self.view_pose, tag='precheck_view_{}'.format(index))
            input('预检查 {}/{}：{}'.format(index, len(prompts), prompt))
            self.approach = {'site': {'site_id': 'precheck'}}
            point, valid, report, locator = self._locate_initial(
                'precheck_{}'.format(index))
            if point is None:
                print('预检查 {}/{}：没有找到木块。请把它放得离机器人'
                      '近一点/正一点，然后告诉我。'.format(
                          index, len(prompts)), flush=True)
                continue
            radius_m, bearing_deg, inside = self._region_check(point)
            print('预检查 {}/{}：定位方式={}；测得点 x={:+.3f}, y={:+.3f} m；'
                  '半径 {:.3f} m，方位 {:+.1f}°{}'.format(
                      index, len(prompts), LOCATOR_ZH.get(locator, locator),
                      point[0], point[1], radius_m, bearing_deg,
                      '' if inside else '（在参考范围外，但已定位）'),
                  flush=True)
        print('预检查结束。第 2 次如果是“前相机（D435i 深度）”，'
              '说明远场可用；把两行结果告诉 ZCode。')


def build_parser():
    """Return the argument parser."""
    parser = sites.build_parser()
    parser.description = __doc__
    parser.set_defaults(
        plan=DEFAULT_PLAN, trials=DEFAULT_TRIALS,
        fixture='random', seed=17)
    parser.add_argument(
        '--region', default=DEFAULT_REGION,
        help='pre-registered random region JSON')
    parser.add_argument(
        '--front-work', default=DEFAULT_FRONT_WORK,
        help='local temporary directory for the front-camera frames')
    parser.add_argument(
        '--front-intrinsics', default=DEFAULT_FRONT_INTRINSICS,
        help='refined UVC colour intrinsics')
    parser.add_argument(
        '--front-extrinsic', default=DEFAULT_FRONT_EXTRINSIC,
        help='calibrated base_link -> front optical extrinsic')
    parser.add_argument(
        '--front-depth-to-color', default=DEFAULT_FRONT_DEPTH_TO_COLOR,
        help='D435i depth -> colour extrinsic')
    parser.add_argument(
        '--precheck', action='store_true',
        help='exercise the locator chain only; no base motion or trial')
    return parser


def main(argv=None):
    """Run the random-placement session or the precheck."""
    args = build_parser().parse_args(argv)
    if not args.dry_run and not sys.stdin.isatty():
        raise SystemExit('这个 session 需要交互式终端（请在 tmux 或终端里运行）')
    session = RandomSiteSession(args)
    try:
        if args.precheck:
            session.precheck()
        else:
            session.run()
    except KeyboardInterrupt:
        print('\n已中断；已完成的 trial 都已记录')
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
