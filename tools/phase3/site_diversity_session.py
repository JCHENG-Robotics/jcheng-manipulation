#!/usr/bin/env python3
"""Run the 24-site paired grasp session on the real robot.

This is the follow-up experiment to the 2026-11-19 single-fixture paired trial
(``docs/phase3/PAIRED_TRIAL_DESIGN.md``). The 24 pairs each use their own
pre-registered floor site (``tools/phase3/site_layout.py``), so every pair
exercises the whole chain:

1. the operator places the cube on the site mark;
2. the wrist camera measures it with the wide region at the view pose;
3. a fallback ``joint1`` yaw sweep looks for it if the coarse home return left
   it outside the image;
4. closed-loop mecanum base pulses bring the cube into the validated
   0.226-0.234 m measurement band on the forward axis;
5. the arm runs the pair's two randomized methods (baseline / intervention)
   back to back at that site;
6. the base returns home by the reverse of the commanded translation, ready
   for the next site.

The grasp target and the outcome classifier are exactly the validated ones in
``paired_trial_session.py``; this runner only adds the locate/approach phase
and the home return. The records are appended through the same recorder, so
the session is resumable and an interrupted pair is re-located, not replayed
from a stale base pose.

Run on the laptop (needs ``sg docker`` for the Humble container and an SSH key
for the robot)::

    sg docker -c "python3 tools/phase3/site_diversity_session.py \
        --sites evidence/phase3/onsite_20261120_sites/site_layout.json \
        --plan evidence/phase3/onsite_20261120_sites/paired_trials_plan.json \
        --trials evidence/phase3/onsite_20261120_sites/trials.jsonl"

Add ``--dry-run`` to exercise the locate/approach/trial flow with synthetic
measurements and no robot calls.

Safety: the base moves are open-loop in translation (no odometry) and are
re-measured after every pulse; the arm stays at the view pose during the
approach. The operator keeps the power switch within reach; the robot has no
e-stop or brake.
"""

import json
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'tools', 'phase2'))
sys.path.insert(0, os.path.join(REPO, 'tools', 'phase3'))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis'))

import fast_motion_e2e  # noqa: E402
import grasp_joint_fit  # noqa: E402
import paired_trial_session as base  # noqa: E402
import site_layout  # noqa: E402
from armpi_analysis import paired_trials  # noqa: E402

DEFAULT_SITES = 'evidence/phase3/onsite_20261120_sites/site_layout.json'
DEFAULT_PLAN = 'evidence/phase3/onsite_20261120_sites/paired_trials_plan.json'
DEFAULT_TRIALS = 'evidence/phase3/onsite_20261120_sites/trials.jsonl'
DEFAULT_SEARCH_YAWS = '0.14,-0.14,0.28,-0.28,0.42,-0.42,0.56,-0.56'
WIDE_REGION = fast_motion_e2e.WIDE_REGION
BASE_COAST_S = 0.4
#: Base dead band (~0.15 s at duty 20 is ~15 mm). The closed-loop approach
#: stops once the cube is within this distance of (0.230, 0); the remaining
#: lateral offset is removed by the joint1 envelope guard below.
DEFAULT_BASE_TOLERANCE_M = 0.015
#: The prior single-fixture paired trial gripped 24/24 with measured
#: pre-grasp points at x 0.2169-0.2388 m and |joint1| <= 0.142 rad. These
#: bounds are the site session's empirical envelope, with a small margin.
DEFAULT_MAX_J1_RAD = 0.16
DEFAULT_X_ENVELOPE = (0.215, 0.240)


def nominal_travel_mm(pulse):
    """Return the pulse's nominal ground travel in millimetres."""
    value = pulse.get('nominal_travel_mm')
    if value is not None:
        return float(value)
    return float(pulse['duty']) * float(pulse['duration']) / 0.182


def approach_plan(point, duty, tolerance):
    """Return the next base pulse for the cube, or None when aligned."""
    return fast_motion_e2e.base_pulse_plan(
        point, duty=duty, tolerance=tolerance)


class SiteSession(base.Session):
    """A paired session in which every pair has its own floor site."""

    def __init__(self, args):
        self.layout = site_layout.load_layout(args.sites)
        self.sites = {
            site['site_id']: site for site in self.layout['sites']}
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
        self.approaches_path = os.path.join(
            os.path.dirname(os.path.abspath(args.trials)),
            'approaches.jsonl')
        if int(args.pairs) != len(self.sites):
            raise SystemExit(
                '--pairs {} does not match the {} sites in {}'.format(
                    args.pairs, len(self.sites), args.sites))
        super().__init__(args)

    def _load_plan(self):
        """Load or pre-register the 24-site paired plan."""
        path = self.args.plan
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as handle:
                plan = json.load(handle)
            fixtures = {entry['object_pose_fixture']
                        for entry in plan.get('entries', [])}
            if fixtures != set(self.sites):
                raise SystemExit(
                    'plan fixtures {} do not match the site layout {}'.format(
                        sorted(fixtures), sorted(self.sites)))
            expected = {
                'p{:03d}'.format(index + 1): site_id
                for index, site_id in enumerate(self.sites)}
            actual = {
                pair['pair_id']: pair['object_pose_fixture']
                for pair in plan.get('pairs', [])}
            if actual != expected:
                raise SystemExit(
                    'plan pair-to-site assignment does not match the '
                    'pre-registered pNNN <-> sNNN mapping')
            return plan
        pair_ids = ['p{:03d}'.format(index + 1)
                    for index in range(len(self.sites))]
        fixtures = list(self.sites)
        plan = paired_trials.build_plan(
            pair_ids, fixtures=fixtures, seed=self.args.seed)
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.',
                    exist_ok=True)
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(plan, handle, indent=2)
            handle.write('\n')
        print('wrote the 24-site randomization plan: {}'.format(path))
        return plan

    def _pending_pairs(self):
        """Return the plan's pending pairs in plan order."""
        pending = [
            entry for entry in self.plan['entries']
            if (entry['pair_id'], entry['method']) not in self.done]
        grouped = {}
        for entry in pending:
            grouped.setdefault(entry['pair_id'], []).append(entry)
        pairs = []
        for pair in self.plan['pairs']:
            pair_id = pair['pair_id']
            entries = grouped.get(pair_id)
            if not entries:
                continue
            site_id = entries[0]['object_pose_fixture']
            pairs.append((pair_id, entries, self.sites[site_id]))
        return pairs

    def _wide_measure(self, tag):
        """Measure any cube cluster with the wide region; no retry prompt."""
        pulses = self.robot.pulses()
        report = self.humble.segment(
            pulses, tag, extra_args=WIDE_REGION)
        point, valid = base.cube_point(report)
        return point, valid, report

    def _search(self, tag):
        """Sweep joint1 and return the first wide-region sighting."""
        report = None
        for yaw in self.search_yaws:
            pose = list(self.view_pose)
            pose[0] = float(yaw)
            self.robot.move_to_pose(
                pose, tag='{}_yaw'.format(tag))
            point, valid, report = self._wide_measure(
                '{}_y{:.2f}'.format(tag, yaw))
            print('  扫视 yaw {:+.2f} rad：{}'.format(
                yaw, '找到木块' if point is not None else '没有点'),
                flush=True)
            if point is not None:
                return point, valid, report, float(yaw)
        self.robot.move_to_pose(self.view_pose, tag='{}_view'.format(tag))
        return None, False, report, 0.0

    def _locate_initial(self, tag):
        """Locate the cube for the pair's first sighting.

        Returns ``(point, valid, report, locator)``. The base runner uses the
        wide view-pose measurement and the joint1 yaw sweep; a subclass may
        add a pose-independent front-camera fallback.
        """
        point, valid, report = self._wide_measure(tag)
        if point is not None:
            return point, valid, report, 'wrist_view'
        point, valid, report, yaw = self._search(tag)
        self.approach['search_yaw_rad'] = yaw
        return point, valid, report, (
            'wrist_yaw' if point is not None else 'none')

    def _locate_after(self, tag):
        """Locate the cube after a base pulse."""
        point, valid, report = self._wide_measure(tag)
        if point is not None:
            return point, valid, report, 'wrist_view'
        point, valid, report, yaw = self._search(tag)
        self.approach['search_yaw_rad'] = yaw
        return point, valid, report, (
            'wrist_yaw' if point is not None else 'none')

    def _validate_initial(self, point, report, site, tag, locator):
        """Hook: accept the first sighting (a subclass may reject a point)."""
        return point

    def _strict_measure(self, tag):
        """Measure the cube at the view pose with the placement gate."""
        pulses = self.robot.pulses()
        report = self.humble.segment(pulses, tag)
        point, valid = base.cube_point(report)
        ok, reason = base.placement_quality(report)
        if point is not None and valid and ok:
            return point, report
        print('  最终严格测量不可用（valid={}，{}）'.format(
            valid, reason), flush=True)
        return None, report

    def _pulse_and_measure(self, point, tag, kind, tolerance=None):
        """Drive one base pulse and re-measure the cube.

        Returns ``(point, record)`` where ``record`` is the pulse's JSON with
        the measured point before and after. ``None`` as the point means the
        cube was lost; the caller decides whether to search.
        """
        plan = approach_plan(
            point, self.args.base_duty,
            self.base_tolerance if tolerance is None else tolerance)
        if plan is None:
            return None, None
        vx, vy, duration = plan
        started = time.monotonic()
        pulse = self.robot.base_pulse(
            vx, vy, self.args.base_duty, duration)
        time.sleep(BASE_COAST_S)
        after, _valid, _report, locator = self._locate_after(
            '{}_after'.format(tag))
        record = {
            'kind': kind,
            'vx': float(vx),
            'vy': float(vy),
            'duty': float(self.args.base_duty),
            'duration': float(duration),
            'nominal_travel_mm': round(nominal_travel_mm(pulse), 3),
            'point_before': [float(value) for value in point],
            'point_after': (None if after is None
                            else [float(value) for value in after]),
            'locator': locator,
            'subscribers_matched': pulse.get('subscribers_matched'),
            'wall_s': round(time.monotonic() - started, 3),
        }
        print('  底盘 {} 脉冲 vx={:+.3f} vy={:+.3f} {:.2f} s'.format(
            kind, vx, vy, duration), flush=True)
        return after, record

    def locate_and_approach(self, site, pair_id):
        """Find the cube at the site and drive the base into the grasp band."""
        tag = '{}_loc'.format(pair_id)
        started = time.monotonic()
        self.approach = {
            'site': site,
            'locator': 'wrist_view',
            'search_yaw_rad': 0.0,
            'base_pulses': [],
            'final_point': None,
            'final_j1_rad': None,
        }
        if self.args.dry_run:
            self._dry_approach(site)
            self.extra_notes = {
                'site': site, 'approach': self.approach}
            return
        self.robot.move_to_pose(self.view_pose, tag=tag)
        point, valid, report, locator = self._locate_initial(tag)
        self.approach['locator'] = locator
        if point is None:
            raise RuntimeError(
                '随机点 {}：定位链没有找到木块'.format(
                    site['site_id']))
        point = self._validate_initial(point, report, site, tag, locator)
        self.approach['initial_point'] = [
            float(value) for value in point]
        self.approach['initial_valid'] = bool(valid)
        self.approach['initial_report_success'] = report.get('success')
        for index in range(self.args.base_pulses):
            point, record = self._pulse_and_measure(
                point, '{}_b{}'.format(tag, index), 'approach')
            if record is None:
                break
            self.approach['base_pulses'].append(record)
            if point is None:
                raise RuntimeError(
                    '随机点 {}：底盘脉冲 {} 后丢失木块'.format(
                        site['site_id'], index + 1))
        if self.approach['base_pulses']:
            print('  已通过 {} 次底盘脉冲把木块带入夹取带。'.format(
                len(self.approach['base_pulses'])), flush=True)
        else:
            print('  木块已在夹取带内，直接抓取。', flush=True)
        # The final grasp needs the validated joint1 ~ 0 view.
        if abs(self.approach['search_yaw_rad']) > 1e-9:
            self.robot.move_to_pose(
                self.view_pose, tag='{}_view'.format(tag))
        point, report = self._strict_measure('{}_final'.format(tag))
        if point is None:
            point, report = self.measure(
                self.robot.pulses(), '{}_final'.format(tag))
        if point is None:
            raise RuntimeError(
                '随机点 {}：没有可用的最终测量'.format(
                    site['site_id']))
        # Empirical envelope guard. The prior 24/24 single-fixture intervention
        # ran over measured pre-grasp points x 0.2169-0.2388 m, |y| <= 24.5 mm
        # and |joint1| <= 0.142 rad, so the site approach must stay inside
        # those bounds (with a 1-2 mm margin) before the target is used.
        envelope = None
        for trim in range(self.args.envelope_trims):
            target = point + self.correction
            fit = grasp_joint_fit.fit(target)
            j1 = float(fit['joints'][0])
            x = float(point[0])
            envelope = {
                'x_m': x,
                'j1_rad': j1,
                'x_ok': self.x_envelope[0] <= x <= self.x_envelope[1],
                'j1_ok': abs(j1) <= self.max_j1_rad,
            }
            if envelope['x_ok'] and envelope['j1_ok']:
                break
            print('  最终点 x={:.4f} m，joint1={:.3f} rad，超出 '
                  'x {}..{} / |j1| ≤ {:.2f}；做一次底盘微调'.format(
                      x, j1, self.x_envelope[0], self.x_envelope[1],
                      self.max_j1_rad), flush=True)
            after, record = self._pulse_and_measure(
                point, '{}_trim_{}'.format(tag, trim), 'envelope_trim',
                tolerance=0.005)
            if record is None or after is None:
                break
            self.approach['base_pulses'].append(record)
            point, report = self._strict_measure(
                '{}_final_{}'.format(tag, trim))
            if point is None:
                break
        target = point + self.correction
        fit = grasp_joint_fit.fit(target)
        self.approach['final_point'] = [float(value) for value in point]
        self.approach['final_j1_rad'] = float(fit['joints'][0])
        self.approach['final_xy_error_mm'] = float(fit['xy_error_mm'])
        self.approach['final_envelope'] = envelope
        if envelope is None or not (envelope['x_ok'] and envelope['j1_ok']):
            raise RuntimeError(
                '随机点 {}：最终点 x={:.4f} m，joint1={:.3f} rad '
                '超出 24/24 经验包络，拒绝抓取'.format(
                    site['site_id'], float(point[0]),
                    float(fit['joints'][0])))
        self.approach['approach_wall_s'] = round(
            time.monotonic() - started, 3)
        self.extra_notes = {'site': site, 'approach': self.approach}

    def _dry_approach(self, site):
        """Build a synthetic approach record for the dry run."""
        point = np.asarray(
            [site['x_m'], site['y_m'], -0.05], dtype=np.float64)
        plan = approach_plan(
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

    def _net_base_travel(self):
        """Return the commanded planar base displacement of the approach.

        The travel uses the **same** ``duty * 4.9 mm/s`` model that sized the
        outbound pulses, not the pulse report's ``duty / 0.182`` nominal
        (109.9 mm/s at duty 20 vs 98 mm/s). Reversing with the plan's model
        makes the base's unknown speed scale cancel: the return duration is
        the same as the outbound duration, so the real displacement is undone
        even when the floor is faster or slower than 98 mm/s. The report's
        nominal value is kept in the record for transparency.

        A pulse with no matched subscriber was dropped by the vendor watchdog
        and is excluded; a pulse whose cube measurement was lost still moved
        the base and must be undone.
        """
        net = np.zeros(2)
        for record in self.approach.get('base_pulses', []):
            matched = record.get('subscribers_matched')
            if matched is not None and int(matched) < 1:
                continue
            travel = (
                float(record['duty']) * float(record['duration'])
                * fast_motion_e2e.BASE_SPEED_PER_DUTY / 1000.0)
            net += travel * np.asarray(
                [record['vx'], record['vy']], dtype=np.float64)
        return net

    def return_home(self, pair_id):
        """Drive back along the reverse of the commanded approach."""
        if self.approach.get('return_home', {}).get('done'):
            return
        started = time.monotonic()
        net = self._net_base_travel()
        distance = float(np.linalg.norm(net))
        self.approach['return_home'] = {
            'net_travel_m': [float(value) for value in net],
            'distance_m': distance,
            'pulses': [],
            'wall_s': 0.0,
            'done': False,
        }
        if distance < 0.005:
            self.approach['return_home']['done'] = True
            return
        if self.args.dry_run:
            self.approach['return_home']['done'] = True
            return
        direction = -net / distance
        remaining = distance
        while remaining > 1e-6:
            duration = min(
                remaining * 1000.0
                / (self.args.base_duty
                   * fast_motion_e2e.BASE_SPEED_PER_DUTY),
                3.0)
            pulse = self.robot.base_pulse(
                float(direction[0]), float(direction[1]),
                self.args.base_duty, duration)
            self.approach['return_home']['pulses'].append({
                'vx': float(direction[0]),
                'vy': float(direction[1]),
                'duration': float(duration),
                'nominal_travel_mm': round(nominal_travel_mm(pulse), 3),
                'subscribers_matched': pulse.get('subscribers_matched'),
            })
            print('  回程脉冲 vx={:+.3f} vy={:+.3f} {:.2f} s'.format(
                direction[0], direction[1], duration), flush=True)
            remaining -= (duration * self.args.base_duty
                          * fast_motion_e2e.BASE_SPEED_PER_DUTY / 1000.0)
            time.sleep(BASE_COAST_S)
        self.approach['return_home']['wall_s'] = round(
            time.monotonic() - started, 3)
        self.approach['return_home']['done'] = True

    def _before_release(self, entry, fit, grasp_report):
        """Return the base home while holding the cube; release at home."""
        self.return_home(entry['pair_id'])
        if self.approach.get('return_home') is not None:
            self.approach['return_home']['while_holding'] = True
        print('  夹着木块回到起点；将在起点放下。', flush=True)
        return True

    def _write_approach(self, pair_id, site, entry=None):
        """Append the locate/approach/return sidecar record."""
        record = {
            'pair_id': pair_id,
            'site': site,
            'approach': self.approach,
            'recorded_at': base.now_iso(),
        }
        if entry is not None:
            record['trial_order'] = int(entry['trial_order'])
            record['method'] = entry['method']
        os.makedirs(
            os.path.dirname(self.approaches_path) or '.', exist_ok=True)
        with open(self.approaches_path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, sort_keys=True) + '\n')

    def run(self):
        """Run every pending pair, site by site."""
        self.prepare()
        pending = self._pending_pairs()
        print('{} pending pairs of {}'.format(
            len(pending), self.plan['n_pairs']))
        if self.args.max_pairs:
            pending = pending[:self.args.max_pairs]
        for index, (pair_id, entries, site) in enumerate(pending):
            self.extra_notes = {}
            self.approach = {}
            print('\n### pair {} ({}/{}) at site {}: radius {:.2f} m, '
                  'bearing {:+.0f} deg, mark ({:.3f}, {:+.3f}) m'.format(
                      pair_id, index + 1, len(pending), site['site_id'],
                      site['radius_m'], site['bearing_deg'],
                      site['x_m'], site['y_m']))
            if not self.args.dry_run:
                input('  place the cube on site {} and press '
                      'Enter ... '.format(site['site_id']))
            try:
                self.locate_and_approach(site, pair_id)
                for entry_index, entry in enumerate(entries):
                    if entry_index > 0 and not self.args.dry_run:
                        input('  re-place the cube on site {} and press '
                              'Enter ... '.format(site['site_id']))
                    self.trial(entry, prompt=False)
            except Exception as exc:
                self.approach['error'] = str(exc)
                raise
            finally:
                # Always undo the base approach, including after an aborted
                # pair. Otherwise a resumed session would start its locate
                # from the previous site and the reverse-home bookkeeping
                # would have no net to undo.
                try:
                    self.return_home(pair_id)
                finally:
                    self._write_approach(pair_id, site)
        print('\n{} trials recorded in {}'.format(
            len(self.done), self.args.trials))


def build_parser():
    """Return the argument parser."""
    parser = base.build_parser()
    parser.description = __doc__
    parser.set_defaults(
        plan=DEFAULT_PLAN, trials=DEFAULT_TRIALS,
        fixture='site_diversity', seed=17)
    parser.add_argument(
        '--sites', default=DEFAULT_SITES,
        help='pre-registered site layout JSON')
    parser.add_argument(
        '--base-duty', type=float,
        default=fast_motion_e2e.BASE_DUTY,
        help='base vector duty for the closed-loop approach')
    parser.add_argument(
        '--base-pulses', type=int, default=5,
        help='maximum closed-loop base pulses per site')
    parser.add_argument(
        '--base-tolerance', type=float,
        default=DEFAULT_BASE_TOLERANCE_M,
        help='stop the base approach inside this distance of (0.230, 0) m')
    parser.add_argument(
        '--search-yaws', default=DEFAULT_SEARCH_YAWS,
        help='joint1 yaw fallback sweep in radians')
    parser.add_argument(
        '--max-j1-rad', type=float, default=DEFAULT_MAX_J1_RAD,
        help='refuse a final point outside this |joint1| envelope')
    parser.add_argument(
        '--x-envelope', default='{:.3f},{:.3f}'.format(*DEFAULT_X_ENVELOPE),
        help='refuse a final point outside this measured x envelope')
    parser.add_argument(
        '--envelope-trims', type=int, default=2,
        help='base pulses allowed to bring the final point inside')
    parser.add_argument(
        '--max-pairs', type=int, default=0,
        help='limit the session to the first N pending pairs')
    return parser


def main(argv=None):
    """Run the site-diversity session."""
    args = build_parser().parse_args(argv)
    if not args.dry_run and not sys.stdin.isatty():
        raise SystemExit('the session needs an interactive operator prompt')
    session = SiteSession(args)
    try:
        session.run()
    except KeyboardInterrupt:
        print('\ninterrupted; completed trials are already recorded')
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
