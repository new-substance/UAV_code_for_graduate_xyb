"""Fixed-input checks of the forest design equations; never a mission simulator.

All numeric inputs are illustrative algebra fixtures, not a radio/aircraft
profile. See 论文总纲 sections 3.2-3.7, 5.2 and 6.1-6.5.
"""
from __future__ import annotations

import json
import math

import numpy as np


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def exp_rotation(v: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(v))
    k = skew(v)
    if angle < 1e-12:
        return np.eye(3) + k + 0.5 * k @ k
    return (np.eye(3) + math.sin(angle) / angle * k
            + (1.0 - math.cos(angle)) / angle**2 * k @ k)


def reference_checks() -> dict:
    p = np.array([3.0, -2.0, 4.0])
    a = np.array([-5.0, 7.0, 0.0])
    lever = np.array([0.2, -0.1, 0.08])
    rotation = exp_rotation(np.array([0.2, -0.1, 0.3]))
    difference = p + rotation @ lever - a
    direction = difference / np.linalg.norm(difference)
    analytic = np.concatenate((direction, -direction @ rotation @ skew(lever), -direction))

    def distance(error: np.ndarray) -> float:
        antenna = p + error[:3] + rotation @ exp_rotation(error[3:6]) @ lever
        return float(np.linalg.norm(antenna - a - error[6:]))

    step = 1e-6
    finite = np.zeros(9)
    for j in range(9):
        perturbation = np.eye(9)[j] * step
        finite[j] = (distance(perturbation) - distance(-perturbation)) / (2 * step)
    error = float(np.max(np.abs(analytic - finite)))
    assert error < 2e-8
    shift = np.array([25.0, -13.0, 7.0])
    shifted = np.linalg.norm(p + shift + rotation @ lever - a - shift)
    gauge_difference = abs(float(shifted - np.linalg.norm(difference)))
    assert gauge_difference < 1e-12

    # A shared primitive produces correlation between position and range error.
    mapping = np.array([[1.0, 0.2, 0, 0, 0], [0, 0, 0.3, 0, 0],
                        [0, 0, 0, 0.4, 0], [0.6, 0, 0, 0, 0.5]])
    joint = mapping @ mapping.T
    jacobian = np.concatenate((direction, [1.0]))
    full = float(jacobian @ joint @ jacobian)
    dropped = float(direction @ joint[:3, :3] @ direction + joint[3, 3])
    expanded = dropped + 2 * float(direction @ joint[:3, 3])
    assert abs(full - expanded) < 1e-12 and abs(full - dropped) > 1e-3
    return {"jacobian_max_abs_difference": error,
            "common_translation_range_difference": gauge_difference,
            "joint_covariance_expansion_difference": abs(full - expanded),
            "omitting_cross_term_changes_variance": abs(full - dropped)}


def wake_checks() -> dict:
    period_bound, window, packet, spacing = 10.0, 3.0, 0.4, 1.2
    count = math.ceil((period_bound + window - packet) / spacing) + 1
    starts = np.arange(count) * spacing
    duration = float(starts[-1] + packet)
    assert window >= spacing + packet and duration >= period_bound + window
    phases = np.linspace(0, period_bound, 2001)
    for phase in phases:
        assert np.any((starts >= phase - 1e-12)
                      & (starts + packet <= phase + window + 1e-12))

    # Too short a receive window can miss every whole packet despite overlap.
    short_window, phase = 0.6, 0.1
    heard = False
    for start in np.arange(phase, duration, period_bound):
        heard |= bool(np.any((starts >= start)
                             & (starts + packet <= start + short_window)))
    assert not heard
    return {"checked_phases": len(phases), "packet_count": count,
            "rounded_train_meets_bound": True,
            "short_window_counterexample_misses_all_packets": True,
            "scope": "timing_opportunity_only_not_channel_or_decode_success"}


def decision_checks() -> dict:
    deadline, reference_reserve = 10.0, 3.0
    tempting_service = 8.0
    assert tempting_service <= deadline
    assert tempting_service + reference_reserve > deadline
    assert 6.0 + reference_reserve <= deadline
    # Counts refer to disjoint current node phases, never delivered observations.
    candidates = {1: (0, 1, 1, 0, 0, 0.0, -4.0, -1),
                  2: (0, 2, 0, 0, 0, 0.0, -5.0, -2)}
    selected = max(candidates, key=candidates.__getitem__)
    assert selected == 2
    return {"end_before_reference_deadline_is_insufficient": True,
            "next_reference_reservation_rejects_counterexample": True,
            "multi_node_key_selects_candidate": selected,
            "scope": "finite_design_examples_not_route_performance"}


def first_model_checks() -> dict:
    # R01-a/b: arbitrary clock phases cancel; rate mismatch does not.
    speed, distance, reply = 299792458.0, 37.0, 0.008
    alpha_a, alpha_b = 1.00002, 0.99997
    t1 = 0.01
    t2 = t1 + distance / speed
    t3 = t2 + reply
    t4 = t3 + distance / speed
    clock_a = lambda t: alpha_a * t + 0.3
    clock_b = lambda t: alpha_b * t - 0.7
    da, db = clock_a(t4) - clock_a(t1), clock_b(t3) - clock_b(t2)
    corrected = speed * (da - alpha_a / alpha_b * db) / (2 * alpha_a)
    uncorrected = speed * (da - db) / (2 * alpha_a)
    clock_error = abs(corrected - distance)
    assert clock_error < 1e-5 and abs(uncorrected - distance) > 1

    # S01/N02: position geometry and mirror ambiguity are distinct checks.
    anchors = np.array([[-20, -20, 0], [20, -20, 0],
                        [20, 20, 0], [-20, 20, 0]], dtype=float)
    position = np.array([1.0, 2.0, 15.0])

    def geometry(p: np.ndarray, a: np.ndarray) -> np.ndarray:
        vectors = p - a
        return vectors / np.linalg.norm(vectors, axis=1)[:, None]

    plane = position * np.array([1.0, 1.0, 0.0])
    line = np.array([[-20, 0, 0], [0, 0, 0], [20, 0, 0], [40, 0, 0]])
    assert np.linalg.matrix_rank(geometry(position, anchors)) == 3
    assert np.linalg.matrix_rank(geometry(plane, anchors)) == 2
    assert np.linalg.matrix_rank(geometry(position, line)) == 2
    mirrored = position * np.array([1.0, 1.0, -1.0])
    mirror_error = float(np.max(np.abs(
        np.linalg.norm(position - anchors, axis=1)
        - np.linalg.norm(mirrored - anchors, axis=1))))
    assert mirror_error < 1e-12

    # L01-a: horizontal motion changes terrain height through its gradient.
    terrain_gradient = np.array([0.08, -0.04])
    x = np.array([1.0, 2.0])
    antenna = np.array([8.0, -5.0, 12.0])

    def slant(horizontal: np.ndarray) -> float:
        node = np.r_[horizontal, terrain_gradient @ horizontal + 0.5]
        return float(np.linalg.norm(antenna - node))

    node = np.r_[x, terrain_gradient @ x + 0.5]
    direction = (antenna - node) / slant(x)
    analytic = -direction[:2] - direction[2] * terrain_gradient
    step = 1e-6
    finite = np.array([(slant(x + step * e) - slant(x - step * e)) / (2 * step)
                       for e in np.eye(2)])
    terrain_error = float(np.max(np.abs(analytic - finite)))
    assert terrain_error < 2e-8

    # V01-b: propagate fitted and held-out errors from one joint primitive.
    design = np.array([[1, 0], [0, 1], [-0.8, 0.6], [0.6, 0.8]], dtype=float)
    verify = np.array([[0.6, -0.8]])
    covariance = np.diag([1, 1.2, 0.8, 1.1, 0.9]) + 1.2 * np.ones((5, 5))
    cff, cfv = covariance[:4, :4], covariance[:4, 4:]
    weighted = np.linalg.solve(cff, design)
    gain = np.linalg.solve(design.T @ weighted, weighted.T)
    propagated = (covariance[4:, 4:] + verify @ gain @ cff @ gain.T @ verify.T
                  - cfv.T @ gain.T @ verify.T - verify @ gain @ cfv)
    mapping = np.c_[-verify @ gain, np.ones((1, 1))]
    direct = mapping @ covariance @ mapping.T
    verify_error = float(np.max(np.abs(propagated - direct)))
    assert verify_error < 1e-12 and propagated.item() > 0

    # N02-b: correlated state/noise conditioning equals direct propagation.
    prior, h, cross, noise = 2.0, 0.6, 0.7, 1.0
    innovation = h * prior * h + 2 * h * cross + noise
    kalman = (prior * h + cross) / innovation
    posterior = prior - (prior * h + cross)**2 / innovation
    joint = np.array([[prior, cross], [cross, noise]])
    error_map = np.array([1 - kalman * h, -kalman])
    correlation_error = abs(float(error_map @ joint @ error_map) - posterior)
    assert correlation_error < 1e-12 and posterior > 0

    # A common survey translation is invisible to relative ranging.
    floor = 4.0 * np.ones((2, 2))
    prior_matrix = floor + np.diag([1.0, 0.5])
    measurement = np.array([[1.0, -1.0]])
    innovation = (measurement @ prior_matrix @ measurement.T).item() + 0.2
    posterior_matrix = (prior_matrix
                        - prior_matrix @ measurement.T @ measurement @ prior_matrix / innovation)
    assert np.min(np.linalg.eigvalsh(posterior_matrix - floor)) > -1e-12
    return {"affine_clock_distance_error_m": clock_error,
            "uncompensated_clock_error_exceeds_one_m": True,
            "position_geometry_ranks": [3, 2, 2],
            "above_below_mirror_range_difference": mirror_error,
            "terrain_jacobian_max_abs_difference": terrain_error,
            "correlated_gls_heldout_variance_difference": verify_error,
            "correlated_update_covariance_difference": correlation_error,
            "common_survey_translation_floor_retained": True,
            "scope": "fixed_algebra_not_full_ins_observability_or_forest_validation"}


def run_checks() -> dict:
    return {"all_passed": True, "new_missions": 0,
            "scope": "fixed_input_algebra_not_forest_navigation_or_protocol_validation",
            "reference": reference_checks(), "wakeup": wake_checks(),
            "decision": decision_checks(), "first_model": first_model_checks()}


if __name__ == "__main__":
    print(json.dumps(run_checks(), ensure_ascii=False))
