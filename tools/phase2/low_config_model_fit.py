#!/usr/bin/env python3
"""Fit physical model-error hypotheses to the low-configuration tag survey.

Run on the laptop with the Jazzy workspace sourced (PyKDL/urdf_parser_py/xacro
are used by ``ArmKinematics``)::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/low_config_model_fit.py \
        --samples evidence/calibration/20261119/low_config_error/\
dataset_low_tag_20261119.json \
        --calibration evidence/calibration/20261118/table_down/\
calibration_table_down_20261118.json \
        --json-out /tmp/low_config_model_fit.json

Why this tool exists
--------------------
The 2026-11-19 tag survey measured the model error between the tilted
table-down view and the low (grasp) family as one constant vector
``(+11.6, +4.0, +8.5) mm`` with a per-pose spread of 0.4-1.0 mm. A constant is
a measurement, not a physical model: it says nothing about configurations that
were not surveyed. Three physical hypotheses can produce (part of) that
vector:

``link4_translation``
    A fixed translation ``dt`` in the ``link4`` frame. The base-frame error
    then rotates with ``link4``: ``d(q) = R_link4(q) @ dt``.
``link4_se3``
    A fixed six-DoF transform in the ``link4`` frame (translation and small
    rotation).
``joint_offsets4``
    Angle offsets ``dq1..dq4`` on the measured joints. The base-frame error is
    ``FK(q + dq) - FK(q)``, which changes shape with the configuration.

The tag is a fixed point in ``base_link``. For every pose the uncorrected
camera prediction ``T_link4(q) @ T_link4_camera @ T_cam_tag`` is one sample of
the model error; a correct hypothesis makes all samples agree on one point.
The fit therefore minimises the deviation of the corrected predictions from
their own mean (the common tag point is a free nuisance parameter). This makes
the comparison a *consistency* test, not a fit to an external truth.

Outputs
-------
- per-model fitted parameters, residual RMS/max, AIC and leave-one-out error;
- the correction each model implies at ``G_grasp`` relative to the reference
  poses, in the same convention as the empirical ``(+13.7, +3.9, +8.9)``;
- for each model, the distance between that prediction and the empirical
  vector.

The result on the 2026-11-19 survey was negative: every correction model fits
the tags, but at the configurations the robot actually ran, all of them
predict a correction 10.6-15.0 mm away from the one four passing grasps used.
See ``evidence/calibration/20261119/low_config_error/model_selection.md``.
The constant base-frame offset is the supported model inside the tested
envelope.

Sign convention
---------------
``p_pred`` is the tag centre predicted with the correction applied; the common
tag truth is ``P``, so the reported survey error is ``p_pred - P``. The
correction a grasp adds to the view measurement is the difference of the
per-pose errors, ``e = (p_pred(grasp) - P) - (p_pred(view) - P)``. This tool
reports ``e`` in that convention, so a positive ``e`` means the view
measurement must be pushed in the positive base-axis direction.
"""

import argparse
from datetime import datetime, timezone
import json
import math

import numpy as np

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
LINKS = {'link4': 'link4', 'tcp': 'end_effector_link'}
# Empirical correction measured on 2026-11-19 (tag-referenced x/y, z from the
# tag and confirmed by the grasp). Reported for comparison only.
EMPIRICAL_CORRECTION_MM = (13.7, 3.9, 8.9)


def make_transform(rotation, translation):
    """Build a 4x4 homogeneous transform."""
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation, dtype=np.float64)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def rotation_from_vector(vector):
    """Build a 3x3 rotation from an axis-angle vector (Rodrigues, rad)."""
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (np.eye(3) + math.sin(angle) * skew
            + (1.0 - math.cos(angle)) * (skew @ skew))


def transform_from_rvec_tvec(rvec, tvec):
    """Build T from an OpenCV rotation vector and translation."""
    return make_transform(rotation_from_vector(rvec), tvec)


def damped_least_squares(function, start, bound, max_iterations=200):
    """Minimise ``function(x)`` inside a symmetric box with Gauss-Newton.

    The problem has at most six parameters and a smooth residual, so a damped
    Gauss-Newton with a numerical Jacobian is enough and keeps the tool free of
    a SciPy dependency (the pure-Python CI installs numpy only).
    """
    scale = np.asarray(bound, dtype=np.float64)
    lower = -scale
    upper = scale
    x = np.clip(np.asarray(start, dtype=np.float64), lower, upper)
    residual = function(x)
    cost = float(np.sum(residual ** 2))
    damping = 1e-3
    for _ in range(max_iterations):
        step_size = 1e-7 * max(1.0, float(np.max(np.abs(x))))
        jacobian = np.empty((residual.size, x.size))
        for index in range(x.size):
            delta = np.zeros_like(x)
            delta[index] = step_size
            jacobian[:, index] = (
                function(np.clip(x + delta, lower, upper))
                - function(np.clip(x - delta, lower, upper))
            ) / (2.0 * step_size)
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ residual
        diagonal = np.diag(np.diag(normal)) + 1e-12 * np.eye(x.size)
        try:
            proposal = np.linalg.solve(normal + damping * diagonal, -gradient)
        except np.linalg.LinAlgError:
            break
        candidate = np.clip(x + proposal, lower, upper)
        candidate_residual = function(candidate)
        candidate_cost = float(np.sum(candidate_residual ** 2))
        if candidate_cost < cost:
            improvement = cost - candidate_cost
            x = candidate
            residual = candidate_residual
            cost = candidate_cost
            damping = max(damping * 0.3, 1e-12)
            if improvement <= 1e-14 * max(1.0, cost):
                break
        else:
            damping *= 10.0
            if damping > 1e12:
                break
    return x, cost


def rotation_angle_deg(rotation):
    """Return the geodesic angle of a rotation matrix in degrees."""
    cosine = (np.trace(rotation) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


class Kinematics:
    """Cached KDL forward kinematics for link4 and the TCP."""

    def __init__(self):
        from armpi_grasp_planner.reachability import ArmKinematics

        self._link4 = ArmKinematics(tip_link=LINKS['link4'])
        self._tcp = ArmKinematics(tip_link=LINKS['tcp'])
        self.joint_names = list(self._link4.joint_names)

    def pose(self, joints, link):
        kinematics = self._link4 if link == 'link4' else self._tcp
        values = [float(value) for value in joints[:5]]
        frame = kinematics.forward(values[:len(kinematics.joint_names)])
        rotation = np.array([
            [frame.M[row, column] for column in range(3)]
            for row in range(3)
        ])
        return make_transform(
            rotation, [frame.p.x(), frame.p.y(), frame.p.z()])


def load_hand_eye(path):
    """Load T_link4_camera from a calibration report (TSAI preferred)."""
    with open(path, 'r', encoding='utf-8') as handle:
        report = json.load(handle)
    hand_eye = report.get('hand_eye', report)
    methods = hand_eye.get('methods', {})
    if 'TSAI' in methods:
        return np.asarray(methods['TSAI']['T_link4_camera'], dtype=np.float64)
    return np.asarray(hand_eye['T_link4_camera'], dtype=np.float64)


def load_samples(path):
    """Load the survey samples and keep the fields the fit needs."""
    with open(path, 'r', encoding='utf-8') as handle:
        dataset = json.load(handle)
    samples = []
    for sample in dataset.get('samples', dataset):
        joints = sample.get('measured_joints') or sample.get('joints')
        if not joints or sample.get('tag') is None:
            continue
        samples.append({
            'name': sample['name'],
            'joints': [float(value) for value in joints[:5]],
            'T_cam_tag': transform_from_rvec_tvec(
                sample['tag']['rvec'], sample['tag']['tvec']),
            'rms_px': float(sample['tag'].get('rms_px', 0.0)),
        })
    if not samples:
        raise SystemExit('no usable tag samples in {}'.format(path))
    return samples


MODELS = {
    # name: (parameter count, bound)
    'none': (0, None),
    'link4_translation': (3, 0.05),
    'link4_se3': (6, 0.05),
    'joint_offsets4': (4, 0.15),
}


def corrected_link4(model, params, joints, kinematics):
    """Return T_base_link4 after applying the model correction."""
    base = kinematics.pose(joints, 'link4')
    if model == 'none':
        return base
    if model == 'link4_translation':
        return base @ make_transform(np.eye(3), params[:3])
    if model == 'link4_se3':
        return base @ make_transform(
            rotation_from_vector(params[:3]), params[3:6])
    if model == 'joint_offsets4':
        shifted = list(joints[:5])
        for index in range(4):
            shifted[index] += float(params[index])
        return kinematics.pose(shifted, 'link4')
    raise ValueError('unknown model {}'.format(model))


def tcp_correction(model, params, joints, kinematics):
    """Return the base-frame TCP displacement of the model correction."""
    base = kinematics.pose(joints, 'tcp')
    if model == 'none':
        return np.zeros(3)
    if model == 'link4_translation':
        link4 = kinematics.pose(joints, 'link4')
        return link4[:3, :3] @ np.asarray(params[:3], dtype=np.float64)
    if model == 'link4_se3':
        link4 = kinematics.pose(joints, 'link4')
        correction = make_transform(
            rotation_from_vector(params[:3]), params[3:6])
        tcp_in_link4 = np.linalg.inv(link4) @ base
        moved = link4 @ correction @ tcp_in_link4
        return moved[:3, 3] - base[:3, 3]
    if model == 'joint_offsets4':
        shifted = list(joints[:5])
        for index in range(4):
            shifted[index] += float(params[index])
        return kinematics.pose(shifted, 'tcp')[:3, 3] - base[:3, 3]
    raise ValueError('unknown model {}'.format(model))


def predict_positions(model, params, samples, t_link4_camera, kinematics):
    """Return the corrected tag centre for every sample in base_link."""
    positions = []
    for sample in samples:
        t_link4 = corrected_link4(
            model, params, sample['joints'], kinematics)
        transform = t_link4 @ t_link4_camera @ sample['T_cam_tag']
        positions.append(transform[:3, 3])
    return np.asarray(positions, dtype=np.float64)


def residual_vector(params, model, samples, t_link4_camera, kinematics):
    """Return the corrected predictions minus their own mean.

    The common tag point is a free nuisance parameter. For fixed model
    parameters its least-squares estimate is the mean of the predictions, so
    this profile residual is the exact joint optimum over (params, P) and
    avoids a gauge that would make a pure translation unidentifiable.
    """
    positions = predict_positions(
        model, params, samples, t_link4_camera, kinematics)
    return (positions - positions.mean(axis=0)).reshape(-1)


def fit_model(model, samples, t_link4_camera, kinematics):
    """Fit one model by minimising the tag-centre spread."""
    count, bound = MODELS[model]
    if count == 0:
        return {
            'model': model,
            'params': [],
            'cost': float(np.sum(residual_vector(
                np.zeros(0), model, samples, t_link4_camera,
                kinematics) ** 2)),
        }
    solution, cost = damped_least_squares(
        lambda candidate: residual_vector(
            candidate, model, samples, t_link4_camera, kinematics),
        np.zeros(count),
        bound,
    )
    return {
        'model': model,
        'params': [float(value) for value in solution],
        'cost': float(cost),
        'success': True,
    }


def residual_metrics(model, params, samples, t_link4_camera, kinematics):
    """Return per-pose residual norms in mm plus summary statistics."""
    positions = predict_positions(
        model, params, samples, t_link4_camera, kinematics)
    residuals = positions - positions.mean(axis=0)
    norms = np.linalg.norm(residuals, axis=1) * 1000.0
    return {
        'per_pose_mm': {
            sample['name']: float(norm)
            for sample, norm in zip(samples, norms)
        },
        'rms_mm': float(np.sqrt(np.mean(norms ** 2))),
        'max_mm': float(np.max(norms)),
        'std_mm': [float(value * 1000.0) for value in residuals.std(axis=0)],
    }


def leave_one_out(model, samples, t_link4_camera, kinematics):
    """Fit on all but one pose and predict the held-out pose's error.

    The training predictions estimate the common tag point; the held-out pose
    is scored against it. This measures whether the model's *shape* generalises
    to a configuration it was not fitted on, which is the property that allows
    the correction to replace a per-grasp constant.
    """
    errors = {}
    for index, held_out in enumerate(samples):
        train = [sample for position, sample in enumerate(samples)
                 if position != index]
        fitted = fit_model(model, train, t_link4_camera, kinematics)
        params = np.asarray(fitted['params'], dtype=np.float64)
        train_pred = predict_positions(
            model, params, train, t_link4_camera, kinematics)
        held_pred = predict_positions(
            model, params, [held_out], t_link4_camera, kinematics)[0]
        error = held_pred - train_pred.mean(axis=0)
        errors[held_out['name']] = {
            'error_mm': [float(value * 1000.0) for value in error],
            'norm_mm': float(np.linalg.norm(error) * 1000.0),
        }
    norms = [entry['norm_mm'] for entry in errors.values()]
    return {
        'per_pose_mm': errors,
        'rms_mm': float(np.sqrt(np.mean(np.square(norms)))),
        'max_mm': float(np.max(norms)),
    }


def reference_names(samples):
    """Return the pose names used as the view reference."""
    names = [sample['name'] for sample in samples
             if sample['name'].startswith('R_')]
    if not names:
        names = [sample['name'] for sample in samples[:2]]
    return names


def implied_correction_mm(model, params, samples, t_link4_camera, kinematics):
    """Return the grasp correction implied by the model (mm).

    ``e = (p_pred(grasp) - P) - (p_pred(view) - P)`` where ``P`` is common, so
    the mean of the two reference poses stands in for the view. The camera
    error uses the real ``G_grasp`` observation; the TCP error uses the same
    pose's measured joints, so both refer to the configuration that was
    actually grasped.
    """
    references = [sample for sample in samples
                  if sample['name'] in reference_names(samples)]
    grasp = next(
        (sample for sample in samples if sample['name'] == 'G_grasp'), None)
    if grasp is None:
        grasp = next(
            (sample for sample in samples
             if sample['name'].startswith('G_')), None)
    if grasp is None:
        raise SystemExit('no G_grasp pose in the survey')
    positions = predict_positions(
        model, params, samples, t_link4_camera, kinematics)
    names = [sample['name'] for sample in samples]
    reference_positions = positions[[names.index(sample['name'])
                                     for sample in references]]
    camera_error = (positions[names.index(grasp['name'])]
                    - reference_positions.mean(axis=0))
    tcp_grasp = tcp_correction(model, params, grasp['joints'], kinematics)
    reference_tcp = [
        tcp_correction(model, params, sample['joints'], kinematics)
        for sample in references
    ]
    # The pipeline commands TCP_model(q_grasp) = C_view + e. With
    # TCP_true = TCP_model + d, the command that lands on the truth is
    # C_view + d(view) - d(grasp), so e is the negative of the grasp-side
    # displacement relative to the view.
    tcp_error = np.mean(np.asarray(reference_tcp), axis=0) - tcp_grasp
    empirical = np.asarray(EMPIRICAL_CORRECTION_MM, dtype=np.float64)
    return {
        'grasp_sample': grasp['name'],
        'camera_mm': [float(value * 1000.0) for value in camera_error],
        'tcp_mm': [float(value * 1000.0) for value in tcp_error],
        'tcp_vs_empirical_mm': [
            float(value) for value in
            (np.asarray(tcp_error, dtype=np.float64) * 1000.0 - empirical)
        ],
        'tcp_vs_empirical_norm_mm': float(np.linalg.norm(
            np.asarray(tcp_error, dtype=np.float64) * 1000.0 - empirical)),
    }


def aic(cost, residual_count, parameter_count):
    """Return the AIC of a least-squares fit."""
    if cost <= 0.0:
        return float('-inf')
    return float(residual_count * math.log(cost / residual_count)
                 + 2 * parameter_count)


def analyse(samples, t_link4_camera, kinematics):
    """Fit every model and assemble the comparison report."""
    residual_count = 3 * len(samples)
    reports = {}
    for model in MODELS:
        fitted = fit_model(model, samples, t_link4_camera, kinematics)
        params = np.asarray(fitted['params'], dtype=np.float64)
        metrics = residual_metrics(
            model, params, samples, t_link4_camera, kinematics)
        loo = leave_one_out(model, samples, t_link4_camera, kinematics)
        correction = implied_correction_mm(
            model, params, samples, t_link4_camera, kinematics)
        reports[model] = {
            'model': model,
            'parameter_count': MODELS[model][0],
            'params': fitted['params'],
            'success': fitted.get('success', True),
            'cost': fitted['cost'],
            'aic': aic(fitted['cost'], residual_count,
                       MODELS[model][0] + 3),
            'residual': metrics,
            'leave_one_out': loo,
            'implied_correction': correction,
            'implied_correction_norm_mm': float(np.linalg.norm(
                correction['tcp_mm'])),
        }
    return reports


def summary_table(reports):
    """Return a Markdown comparison table."""
    lines = [
        '| Model | Params | Residual RMS (mm) | Residual max (mm) | '
        'LOO RMS (mm) | LOO max (mm) | Implied TCP correction (mm) |',
        '|---|---:|---:|---:|---:|---:|---|',
    ]
    for name, report in reports.items():
        correction = ', '.join(
            '{:+.1f}'.format(value)
            for value in report['implied_correction']['tcp_mm'])
        lines.append(
            '| `{}` | {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | ({}) |'.format(
                name,
                report['parameter_count'],
                report['residual']['rms_mm'],
                report['residual']['max_mm'],
                report['leave_one_out']['rms_mm'],
                report['leave_one_out']['max_mm'],
                correction,
            ))
    return '\n'.join(lines)


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(
        description='Fit physical model-error hypotheses to the tag survey.')
    parser.add_argument(
        '--samples', required=True,
        help='tag survey samples JSON (dataset_low_tag_*.json)')
    parser.add_argument(
        '--calibration', required=True,
        help='hand-eye calibration report (T_link4_camera)')
    parser.add_argument('--json-out', help='write the full report JSON')
    parser.add_argument('--markdown-out', help='write the comparison table')
    return parser


def main(argv=None):
    """Run the fit and report."""
    args = build_parser().parse_args(argv)
    samples = load_samples(args.samples)
    t_link4_camera = load_hand_eye(args.calibration)
    kinematics = Kinematics()
    reports = analyse(samples, t_link4_camera, kinematics)
    table = summary_table(reports)
    print(table)
    print()
    print('Empirical correction applied on 2026-11-19: '
          '({:+.1f}, {:+.1f}, {:+.1f}) mm'.format(*EMPIRICAL_CORRECTION_MM))
    ranked = sorted(reports.values(), key=lambda report: report['aic'])
    margin = ranked[1]['aic'] - ranked[0]['aic'] if len(ranked) > 1 else 0.0
    print('Lowest AIC: {} (delta to the next: {:.2f})'.format(
        ranked[0]['model'], margin))
    if args.markdown_out:
        with open(args.markdown_out, 'w', encoding='utf-8') as handle:
            handle.write(table + '\n')
    if args.json_out:
        payload = {
            'generated_utc': datetime.now(timezone.utc).isoformat(),
            'samples': args.samples,
            'calibration': args.calibration,
            'empirical_correction_mm': list(EMPIRICAL_CORRECTION_MM),
            'reports': reports,
        }
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2)
        print('Wrote {}'.format(args.json_out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
