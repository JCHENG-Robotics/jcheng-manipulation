"""Tests for the low-configuration model-error fit (numpy only, no ROS)."""

import math

import numpy as np

import low_config_model_fit as fit


class FakeKinematics:
    """A toy 4-joint arm with a yaw base and three pitch joints."""

    def pose(self, joints, link):
        q = [float(value) for value in joints]
        yaw = q[0]
        pitch = q[1] + q[2] + q[3]
        rotation = fit.rotation_from_vector([0.0, pitch, 0.0]) @ \
            fit.rotation_from_vector([0.0, 0.0, yaw])
        if link == 'link4':
            reach = 0.20 + 0.03 * math.cos(q[2])
            point = [reach * math.cos(q[1]), 0.02 * q[3],
                     reach * math.sin(q[1])]
        elif link == 'tcp':
            link4 = self.pose(q, 'link4')
            point = (link4 @ np.array([0.04, 0.0, 0.0, 1.0]))[:3]
        else:
            raise ValueError('unknown link {}'.format(link))
        return fit.make_transform(rotation, point)


POSES = [
    [0.00, 0.30, -1.20, -1.60],
    [0.08, 0.30, -1.20, -1.60],
    [0.00, -0.66, -1.80, -0.39],
    [0.00, -0.56, -1.75, -0.55],
    [0.10, -0.66, -1.80, -0.39],
    [-0.10, -0.66, -1.80, -0.39],
    [0.00, -0.66, -1.75, -0.39],
    [0.00, -0.66, -1.80, -0.29],
]


def synthetic_samples(delta, kinematics):
    """Build samples from a known link4-frame translation.

    The tag is one fixed point in ``base_link``; the camera sees it through the
    true link4 pose, so the only inconsistency in the data is the correction.
    """
    camera = fit.make_transform(np.eye(3), [0.05, 0.02, 0.03])
    world_tag = fit.make_transform(
        fit.rotation_from_vector([0.01, -0.02, 0.05]), [0.28, 0.01, 0.02])
    samples = []
    for index, pose in enumerate(POSES):
        link4 = kinematics.pose(pose, 'link4')
        true_link4 = link4 @ fit.make_transform(np.eye(3), delta)
        t_cam_tag = np.linalg.inv(true_link4 @ camera) @ world_tag
        samples.append({
            'name': 'R_ref{}'.format(index) if index < 2
                    else 'G_{}'.format(index),
            'joints': pose,
            'T_cam_tag': t_cam_tag,
        })
    return samples, camera


def test_fit_recovers_a_link4_translation():
    """A known link4-frame translation is recovered from the tag spread."""
    kinematics = FakeKinematics()
    delta = np.array([0.010, -0.005, 0.020])
    samples, camera = synthetic_samples(delta, kinematics)
    fitted = fit.fit_model('link4_translation', samples, camera, kinematics)
    assert np.allclose(fitted['params'], delta, atol=2e-4)
    metrics = fit.residual_metrics(
        'link4_translation', fitted['params'], samples, camera, kinematics)
    assert metrics['max_mm'] < 0.05


def test_uncorrected_model_has_spread_and_fit_improves_it():
    """The uncorrected model is inconsistent and the fit reduces the spread."""
    kinematics = FakeKinematics()
    delta = np.array([0.010, -0.005, 0.020])
    samples, camera = synthetic_samples(delta, kinematics)
    plain = fit.residual_metrics('none', np.zeros(0), samples, camera,
                                 kinematics)
    fitted = fit.fit_model('link4_translation', samples, camera, kinematics)
    corrected = fit.residual_metrics(
        'link4_translation', fitted['params'], samples, camera, kinematics)
    assert plain['rms_mm'] > 1.0
    assert corrected['rms_mm'] < 0.01


def test_implied_correction_sign_matches_the_grasp_pipeline():
    """The implied correction is R(view)*d - R(grasp)*d, added to the view."""
    kinematics = FakeKinematics()
    delta = np.array([0.012, 0.0, 0.018])
    samples, camera = synthetic_samples(delta, kinematics)
    fitted = fit.fit_model('link4_translation', samples, camera, kinematics)
    correction = fit.implied_correction_mm(
        'link4_translation', fitted['params'], samples, camera, kinematics)
    reference = np.mean(
        [kinematics.pose(POSES[index], 'link4')[:3, :3] for index in (0, 1)],
        axis=0)
    grasp = kinematics.pose(POSES[2], 'link4')[:3, :3]
    expected = (reference - grasp) @ delta * 1000.0
    assert np.allclose(correction['tcp_mm'], expected, atol=0.05)


def test_link4_translation_is_identified_uniquely():
    """Restarts from different points all reach the same translation."""
    kinematics = FakeKinematics()
    delta = np.array([0.008, 0.004, -0.015])
    samples, camera = synthetic_samples(delta, kinematics)
    rng = np.random.default_rng(3)
    solutions = []
    for _ in range(5):
        start = rng.uniform(-0.02, 0.02, 3)
        solution, _ = fit.damped_least_squares(
            lambda candidate: fit.residual_vector(
                candidate, 'link4_translation', samples, camera, kinematics),
            start, 0.05)
        solutions.append(solution)
    assert np.allclose(np.asarray(solutions), delta, atol=2e-4)


def test_aic_prefers_a_smaller_cost():
    """The AIC helper is monotone in the residual cost."""
    assert fit.aic(1e-6, 30, 3) < fit.aic(1e-3, 30, 3)
