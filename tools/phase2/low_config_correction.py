#!/usr/bin/env python3
"""Compute the low-configuration grasp correction from a fitted model.

The grasp pipeline measures the object at a view pose ``q_view`` and commands
the arm to ``q_grasp``. The fitted model says the true TCP is
``TCP_model(q) + d(q)``, so the command that lands on the measured point is::

    TCP_model(q_grasp) = C_view + d(q_view) - d(grasp)

This tool returns ``e = d(q_view) - d(q_grasp)`` in millimetres, i.e. the
vector the pipeline adds to the view measurement. With a fitted
``link4_translation`` model the correction rotates with ``link4`` and therefore
follows a yawed grasp, unlike the constant vector measured on one family.

Run on the laptop with the Jazzy workspace sourced::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/low_config_correction.py \
        --report evidence/calibration/20261119/low_config_error/\
report_low_config_model_fit.json \
        --model link4_translation \
        --view-joints "0,0.2574,-1.18773,-1.70,-0.01676" \
        --grasp-joints "-0.0263,-0.6618,-1.8012,-0.294,0"
"""

import argparse
import json

import numpy as np

import low_config_model_fit as model_fit


def load_model(report_path, model, fitted_params=None):
    """Return the model name and parameters from a fit report."""
    with open(report_path, 'r', encoding='utf-8') as handle:
        report = json.load(handle)
    parameters = None
    if fitted_params is not None:
        parameters = [float(value) for value in fitted_params.split(',')
                      if value.strip()]
    elif model in report.get('reports', {}):
        parameters = report['reports'][model]['params']
    elif model in report and 'params' in report[model]:
        parameters = report[model]['params']
    if parameters is None:
        raise SystemExit(
            'no fitted parameters for {} in {}'.format(model, report_path))
    return model, np.asarray(parameters, dtype=np.float64)


def correction_mm(model, params, view_joints, grasp_joints, kinematics):
    """Return the correction added to the view measurement, in mm."""
    view = model_fit.tcp_correction(model, params, view_joints, kinematics)
    grasp = model_fit.tcp_correction(model, params, grasp_joints, kinematics)
    return (view - grasp) * 1000.0


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(
        description='Compute the low-configuration grasp correction.')
    parser.add_argument('--report', required=True,
                        help='fit report JSON from low_config_model_fit.py')
    parser.add_argument('--model', default='link4_translation',
                        choices=sorted(model_fit.MODELS),
                        help='fitted model to evaluate')
    parser.add_argument('--params', help='override the fitted parameters')
    parser.add_argument('--view-joints', required=True,
                        help='five comma-separated joints at the view pose')
    parser.add_argument('--grasp-joints', required=True,
                        help='five comma-separated joints at the grasp pose')
    parser.add_argument('--json-out', help='write the result as JSON')
    return parser


def main(argv=None):
    """Run the correction computation."""
    args = build_parser().parse_args(argv)
    view = [float(value) for value in args.view_joints.split(',')]
    grasp = [float(value) for value in args.grasp_joints.split(',')]
    if len(view) < 5 or len(grasp) < 5:
        raise SystemExit('both joint lists need five values')
    model, params = load_model(args.report, args.model, args.params)
    kinematics = model_fit.Kinematics()
    correction = correction_mm(model, params, view, grasp, kinematics)
    print('model: {}'.format(model))
    print('correction: ({:+.2f}, {:+.2f}, {:+.2f}) mm  norm {:.2f} mm'.format(
        correction[0], correction[1], correction[2],
        float(np.linalg.norm(correction))))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump({
                'model': model,
                'params': [float(value) for value in params],
                'view_joints': view,
                'grasp_joints': grasp,
                'correction_mm': [float(value) for value in correction],
            }, handle, indent=2)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
