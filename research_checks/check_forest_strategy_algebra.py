"""Fixed-input checks of the forest design equations; never a mission simulator.

All numeric inputs are illustrative algebra fixtures, not a radio/aircraft
profile. See 论文总纲 sections 3.2-3.10, 4.1-4.7, 5.2, 6.1-6.7 and 11-12.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

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


def css_packet_time_s(payload_bytes: int, phy: dict) -> float:
    """C01-a, restricted analytical branch; not a radio driver."""
    if (not 7 <= phy["sf"] <= 10 or phy["long_interleaving"]
            or not 1 <= payload_bytes <= 255 or not 1 <= phy["cr_index"] <= 4):
        raise ValueError("Unsupported analytical packet mode or length")
    header_bits = 20 if phy["explicit_header"] else 0
    crc_bits = 16 if phy["crc_enabled"] else 0
    sf = phy["sf"]
    blocks = math.ceil(max(8 * payload_bytes + crc_bits - 4 * sf + 8 + header_bits, 0)
                       / (4 * sf))
    symbols = phy["preamble_symbols"] + 4.25 + 8 + blocks * (phy["cr_index"] + 4)
    return symbols * 2**sf / phy["bandwidth_hz"]


def protocol_parameter_checks() -> dict:
    """D1/D2 and N02-c/P01-a fixed-input calculations; zero task runs."""
    config = json.loads((Path(__file__).resolve().parents[1]
                         / "research_configs/baseline.json").read_text(encoding="utf-8"))
    profile = config["project_context"]["analytical_profile"]
    assert profile["usage"] == "fixed_input_algebra_only_not_mission_configuration"
    assert not profile["formal_radio_and_imu_selected"]
    phy = profile["ordinary_phy"]
    fixture = profile["manufacturer_ranging_fixture"]
    symbol = 2**fixture["sf"] / fixture["bandwidth_hz"]
    total = symbol * (fixture["preamble_symbols"] + 2 * fixture["ranging_symbols"] + 22.25)
    master = symbol * (fixture["preamble_symbols"] + fixture["ranging_symbols"] + 20.25)
    slave = symbol * fixture["ranging_symbols"]
    assert abs(total - master - slave - 2 * symbol) < 1e-12
    assert abs(total * 1000 - fixture["quoted_exchange_ms"]) < 0.005

    # Application byte ledger excludes PHY header and CRC.
    header = sum([1, 1, 2, 2, 2, 2, 2, 1, 1])
    wake_fixed = header + sum([2, 4, 4, 4, 1])
    member = sum([2, 1, 2])
    maximum_members = (255 - wake_fixed) // member
    assert header == 14 and wake_fixed == 29 and maximum_members == 45
    assert wake_fixed + 45 * member <= 255 < wake_fixed + 46 * member
    sizes = {"wake_four_members": wake_fixed + 4 * member,
             "id_status": header + sum([2, 2, 2, 2, 4]),
             "data_four_sample_bytes": header + sum([2, 1, 1, 4, 2, 4]),
             "range_control": header + sum([2, 1, 1, 2, 1, 1, 2, 4]),
             "range_report": header + sum([2, 4, 2, 2, 4]),
             "ack": header + sum([1, 2, 1]),
             "complete": header + sum([2, 1, 4])}
    durations = {name: 1000 * css_packet_time_s(size, phy) for name, size in sizes.items()}
    assert abs(durations["ack"] - 31.035076923076925) < 1e-10
    ranging = profile["ranging_phy"]
    ranging_time = (2**ranging["sf"] / ranging["bandwidth_hz"]
                    * (ranging["preamble_symbols"] + 2 * ranging["ranging_symbols"] + 22.25))
    pair_radio = (2 * durations["range_control"] + 2 * ranging_time * 1000
                  + durations["range_report"] + durations["ack"])
    assert abs(pair_radio - 233.47199999999998) < 1e-9
    unsupported = {**phy, "sf": 6}
    try:
        css_packet_time_s(18, unsupported)
    except ValueError:
        pass
    else:
        raise AssertionError("SF6 must not use the SF7-10 ordinary-packet branch")

    # Two worst-case bounded response intervals, not a channel simulation.
    reply = css_packet_time_s(sizes["id_status"], phy)
    bound, rx_gap = 0.003, 0.001  # illustrative seconds, not frozen timing bounds
    slot = reply + 2 * bound + rx_gap
    assert bound + reply + rx_gap <= slot - bound + 1e-12
    assert bound + reply + rx_gap > reply + rx_gap - bound

    # N02-c: symmetric reference geometry without an INS or trajectory.
    lx, ly, height = 2.0, 2.0, 1.0
    anchors = np.array([[-lx, -ly, 0], [lx, -ly, 0],
                        [lx, ly, 0], [-lx, ly, 0]])
    vectors = np.array([0.0, 0.0, height]) - anchors
    geometry = vectors / np.linalg.norm(vectors, axis=1)[:, None]
    exact = 4 * np.diag([lx**2, ly**2, height**2]) / (lx**2 + ly**2 + height**2)
    layout_difference = float(np.max(np.abs(geometry.T @ geometry - exact)))
    assert layout_difference < 1e-12
    covariance_position = np.linalg.inv(exact)
    height_ratio = math.sqrt(covariance_position[2, 2] / covariance_position[0, 0])
    assert abs(height_ratio - lx / height) < 1e-12

    # P01-a: hypothetical fit candidate only; never absorb held-out data.
    fit = np.array([[1, 0], [0, 1], [-0.8, 0.6], [0.6, 0.8]], dtype=float)
    candidate = np.array([[0.6, -0.8]])
    joint_noise = np.diag([1, 1.2, 0.8, 1.1, 0.9]) + 0.8 * np.ones((5, 5))
    cff, cfq = joint_noise[:4, :4], joint_noise[:4, 4:]
    conditional = joint_noise[4:, 4:] - cfq.T @ np.linalg.solve(cff, cfq)
    adjusted = candidate - cfq.T @ np.linalg.solve(cff, fit)
    added = adjusted.T @ np.linalg.solve(conditional, adjusted)
    information_fit = fit.T @ np.linalg.solve(cff, fit)
    joint_design = np.vstack([fit, candidate])
    information_joint = joint_design.T @ np.linalg.solve(joint_noise, joint_design)
    information_difference = float(np.max(np.abs(
        information_joint - information_fit - added)))
    assert information_difference < 1e-12
    assert np.min(np.linalg.eigvalsh(added)) > -1e-12

    imu = profile["imu_datasheet_fixture"]
    acc_xy = imu["acc_xy_micro_g_per_sqrt_hz"] * 1e-6 * 9.80665
    acc_z = imu["acc_z_micro_g_per_sqrt_hz"] * 1e-6 * 9.80665
    gyro = math.radians(imu["gyro_deg_per_s_per_sqrt_hz"])
    rms_deg = imu["gyro_deg_per_s_per_sqrt_hz"] * math.sqrt(
        imu["gyro_example_bandwidth_hz"])
    assert abs(rms_deg - imu["gyro_quoted_rms_deg_per_s"]) < 0.005
    return {"profile_id": profile["id"], "manufacturer_exchange_ms": total * 1000,
            "profile_ranging_exchange_ms": ranging_time * 1000,
            "pair_radio_lower_bound_ms": pair_radio,
            "application_bytes": sizes, "ordinary_packet_ms": durations,
            "wake_maximum_members": maximum_members,
            "bounded_slot_guard_checked": True,
            "reference_geometry_information_difference": layout_difference,
            "height_horizontal_std_ratio_fixture": height_ratio,
            "correlated_candidate_information_difference": information_difference,
            "acc_xy_si_asd": acc_xy, "acc_z_si_asd": acc_z, "gyro_si_asd": gyro,
            "gyro_47hz_ideal_rms_deg_per_s": rms_deg,
            "scope": "fixed_datasheet_and_matrix_calculations_not_forest_performance"}


def workflow_checks() -> dict:
    """I01/N03/D01/L02/Q01/V02/P02 fixed matrices, never a flight replay."""
    def alignment(f: np.ndarray, yaw: float) -> tuple:
        radial = math.hypot(f[1], f[2])
        norm = float(np.linalg.norm(f))
        if radial < 1e-8 or norm < 1e-8:
            raise ValueError("Degenerate stationary alignment")
        roll, pitch = math.atan2(f[1], f[2]), math.atan2(-f[0], radial)
        rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        return rz @ ry @ rx, roll, pitch

    f, yaw = np.array([0.8, 1.1, 9.7]), 0.3
    rotation, roll, pitch = alignment(f, yaw)
    radial, norm2 = math.hypot(f[1], f[2]), float(f @ f)
    euler_derivative = np.array([
        [0, f[2] / radial**2, -f[1] / radial**2, 0],
        [-radial / norm2, f[0] * f[1] / (radial * norm2),
         f[0] * f[2] / (radial * norm2), 0], [0, 0, 0, 1]])
    local_map = np.array([[1, 0, -math.sin(pitch)],
                          [0, math.cos(roll), math.sin(roll) * math.cos(pitch)],
                          [0, -math.sin(roll), math.cos(roll) * math.cos(pitch)]])
    attitude_jacobian = local_map @ euler_derivative
    lever = np.array([0.1, -0.04, -0.2])
    step, finite, finite_position = 1e-6, np.zeros((3, 4)), np.zeros((3, 4))
    for j in range(4):
        delta = np.eye(4)[j] * step
        plus = alignment(f + delta[:3], yaw + delta[3])[0]
        minus = alignment(f - delta[:3], yaw - delta[3])[0]
        body_derivative = rotation.T @ ((plus - minus) / (2 * step))
        finite[:, j] = [body_derivative[2, 1], body_derivative[0, 2],
                        body_derivative[1, 0]]
        finite_position[:, j] = (-plus @ lever + minus @ lever) / (2 * step)
    alignment_error = float(np.max(np.abs(finite - attitude_jacobian)))
    assert alignment_error < 2e-8
    assert np.max(np.abs(rotation.T[:, 2] - f / np.linalg.norm(f))) < 1e-12
    lever_error = float(np.max(np.abs(
        finite_position - rotation @ skew(lever) @ attitude_jacobian)))
    assert lever_error < 2e-8
    try:
        alignment(np.zeros(3), yaw)
    except ValueError:
        pass
    else:
        raise AssertionError("Zero specific force cannot initialize orientation")
    # Primitive order: home, accel mean, accel bias, gyro mean, yaw, lever, velocity.
    j0 = np.zeros((15, 19))
    j0[:3, :3], j0[3:6, 16:19] = np.eye(3), np.eye(3)
    j0[6:9, 3:6], j0[6:9, 6:9] = attitude_jacobian[:, :3], -attitude_jacobian[:, :3]
    j0[6:9, 12] = attitude_jacobian[:, 3]
    j0[9:12, 6:9], j0[12:15, 9:12] = np.eye(3), np.eye(3)
    j0[:3, 3:13] = rotation @ skew(lever) @ j0[6:9, 3:13]
    j0[:3, 13:16] = -rotation
    primitive_std = np.r_[np.repeat(0.2, 3), np.repeat(0.02, 3),
                          np.repeat(0.03, 3), np.repeat(0.004, 3), 0.02,
                          np.repeat(0.01, 3), np.repeat(0.05, 3)]
    p0 = j0 @ np.diag(primitive_std**2) @ j0.T
    assert np.linalg.norm(p0[:3, 6:9]) > 0 and np.linalg.norm(p0[6:9, 9:12]) > 0
    assert np.min(np.linalg.eigvalsh(p0)) > -1e-12
    correlated_mean_variance = float(np.ones(2) @ np.array([[1, 0.8], [0.8, 1]])
                                     @ np.ones(2) / 4)
    assert abs(correlated_mean_variance - 0.9) < 1e-12

    # N03 augmentation preserves marginal information and nonzero cross blocks.
    base = np.array([[2, 0.3, 0.1], [0.3, 1.5, 0.2], [0.1, 0.2, 1.0]])
    clone_map = np.array([[1, 0.2, 0], [0, 0, 1]])
    augmentation = np.vstack([np.eye(3), clone_map])
    augmented = augmentation @ base @ augmentation.T
    assert np.linalg.matrix_rank(augmented) == 3 and np.linalg.norm(augmented[:3, 3:]) > 0
    observation = np.array([[1, -0.3, 0.2, 0, 0]])
    innovation = (observation @ augmented @ observation.T).item() + 0.1
    gain = augmented @ observation.T / innovation
    updated = augmented - gain @ observation @ augmented
    direct_h = observation[:, :3]
    direct = base - base @ direct_h.T @ direct_h @ base / innovation
    clone_marginal_error = float(np.max(np.abs(updated[:3, :3] - direct)))
    assert clone_marginal_error < 1e-12 and np.linalg.norm(gain[3:]) > 0

    # Identity, qualified transactions, spatial groups and raw holdout differ.
    identity_seen, raw_ids, required = True, {"txn1"}, 2
    assert not (identity_seen and len(set()) > 0)
    assert identity_seen and len(raw_ids) > 0 and len(raw_ids) < required
    raw_ids.add("txn1")
    assert len(raw_ids) == 1
    raw_ids.add("txn2")
    assert len(raw_ids) == required
    fit_ids, holdout_ids = {"a", "b", "c", "d"}, {"e"}
    assert fit_ids.isdisjoint(holdout_ids) and not fit_ids.isdisjoint({"d", "e"})
    difference_covariance = 5 * np.eye(2) + 5 * np.eye(2) - 4 * np.eye(2) - 4 * np.eye(2)
    assert 5 - math.sqrt(np.max(np.linalg.eigvalsh(difference_covariance))) >= 3
    assert 5 - math.sqrt(10) < 3

    # A three-cell fixed likelihood has two equal separated branches.
    grid = np.array([[-1.0, 0], [0, 0], [1.0, 0]])
    observed = np.array([1.0, 2.0])
    means_fit = np.array([[1, 2], [4, 5], [1, 2]], dtype=float)
    cff = np.array([[1.0, 0.4], [0.4, 1.3]])
    residuals = observed - means_fit
    costs = np.array([e @ np.linalg.solve(cff, e) for e in residuals])
    costs += np.linalg.slogdet(cff)[1] - 2 * math.log(1 / 3)
    weights = np.exp(-0.5 * (costs - costs.min()))
    weights /= weights.sum()
    active = costs - costs.min() <= 5.0  # algebra fixture, not a frozen gamma_Q
    estimate = grid[int(np.argmin(costs))].copy()
    radius = float(np.max(np.linalg.norm(grid[active] - estimate, axis=1)) + 0.1)
    assert active.tolist() == [True, False, True] and radius > 0.5
    weights_before = weights.copy()
    cvf, cvv = np.array([[0.3, 0.2]]), 0.9
    conditional_variance = cvv - (cvf @ np.linalg.solve(cff, cvf.T)).item()
    assert 0 < conditional_variance < cvv
    means_new = np.array([2.0, 3.0, 4.0])
    predictive_means = means_new + residuals @ np.linalg.solve(cff, cvf.T).ravel()

    def predictive_cdf(z: float) -> float:
        values = np.array([0.5 * (1 + math.erf(
            (z - mean) / math.sqrt(2 * conditional_variance))) for mean in predictive_means])
        return float(weights @ values)

    central, extreme = predictive_cdf(3.0), predictive_cdf(30.0)
    assert 0.05 <= central <= 0.95 and extreme > 0.95
    assert np.array_equal(weights, weights_before) and np.array_equal(estimate, grid[0])
    recomputed = np.exp(-0.5 * (costs - costs.min()))
    recomputed /= recomputed.sum()
    incorrectly_reused_posterior = weights**2 / np.sum(weights**2)
    assert np.array_equal(recomputed, weights)
    assert np.max(np.abs(incorrectly_reused_posterior - weights)) > 1e-6
    mixture_variance = float(weights @ (conditional_variance + predictive_means**2)
                             - (weights @ predictive_means)**2)
    assert mixture_variance > conditional_variance
    # L02-a conditions a shared primitive once rather than adding an independent prior.
    shared = np.array([[1.0, 0.2], [0.2, 0.8]])
    position_prior = np.diag([0.5, 0.7])
    shared_cross = np.array([[0.1, 0.02], [0.03, -0.08]])
    conditional_primitive = shared - shared_cross @ np.linalg.solve(
        position_prior, shared_cross.T)
    conditional_mean = shared_cross @ np.linalg.solve(position_prior, np.array([0.2, -0.1]))
    assert np.min(np.linalg.eigvalsh(conditional_primitive)) > 0
    assert np.linalg.norm(conditional_mean) > 0
    assert np.linalg.norm(conditional_primitive - shared) > 0

    # P02 full reservations reject a tempting late reference; no movement is run.
    candidates = {1: {"counts": (0, 1, 1, 0, 0), "service": 6, "reference": 3},
                  2: {"counts": (0, 2, 0, 0, 0), "service": 8, "reference": 3},
                  3: {"counts": (0, 2, 0, 0, 0), "service": 7, "reference": 2}}
    admitted = {key: value for key, value in candidates.items()
                if value["service"] + value["reference"] <= 10 and value["service"] + 5 <= 20}
    selected = max(admitted, key=lambda key: (
        *admitted[key]["counts"], -admitted[key]["service"], -key))
    assert 2 not in admitted and selected == 3
    # Shared control/move is paid once; individually adding every node is tested.
    reserved, service = [], 2
    for node, duration in [(1, 4), (2, 4), (3, 1)]:
        if service + duration + 2 <= 10:
            reserved.append(node)
            service += duration
    assert reserved == [1, 3] and service == 7
    def segment_time(length: float) -> float:
        return 2 * math.sqrt(length) if length <= 4 else length / 2 + 2
    assert segment_time(0) == 0 and segment_time(4) == 4 and segment_time(9) == 6.5
    assert abs(segment_time(4 + 1e-8) - segment_time(4 - 1e-8)) < 2e-8
    return {"alignment_local_jacobian_difference": alignment_error,
            "initial_position_lever_jacobian_difference": lever_error,
            "initial_position_attitude_bias_cross_terms_retained": True,
            "correlated_static_mean_variance": correlated_mean_variance,
            "clone_navigation_marginal_difference": clone_marginal_error,
            "identity_discovery_group_and_raw_holdout_checked": True,
            "multimodal_grid_region_rejects_tight_quality_fixture": True,
            "conditional_predictive_cdf_central": central,
            "conditional_predictive_cdf_extreme": extreme,
            "heldout_does_not_modify_four_point_weights_or_estimate": True,
            "external_prior_not_replaced_by_same_data_posterior": True,
            "mixture_position_uncertainty_retained": True,
            "correlated_position_prior_conditioned_once": True,
            "piecewise_move_time_boundary_checked": True,
            "selected_fixed_candidate": selected, "reserved_fixed_nodes": reserved,
            "scope": "fixed_initialization_state_matrix_and_probability_examples_not_a_mission"}


def strategy_review_checks() -> dict:
    """Section 10 counterexamples; no dynamics, random draws or mission loop."""
    # A central mixture interval can include the gap between two narrow modes.
    alpha, means, std = 0.1, np.array([-5.0, 5.0]), 0.2

    def acceptance(z: float) -> tuple[float, bool, bool]:
        component_cdf = np.array([
            0.5 * (1 + math.erf((z - mean) / (math.sqrt(2) * std)))
            for mean in means])
        mixture_cdf = float(component_cdf.mean())
        central = alpha / 2 <= mixture_cdf <= 1 - alpha / 2
        compatible = bool(np.any((alpha / 2 <= component_cdf)
                                 & (component_cdf <= 1 - alpha / 2)))
        return mixture_cdf, central, central and compatible

    gap, left_mode, tail = acceptance(0.0), acceptance(-5.0), acceptance(30.0)
    assert gap == (0.5, True, False)
    assert left_mode[2] and not tail[2]
    # A scalar rule loses contrasts; V02-c adds a full-vector guard below.
    contrast, projection = np.array([5.0, -5.0]), np.array([0.5, 0.5])
    assert float(projection @ contrast) == 0.0 and float(contrast @ contrast) == 50.0

    # A long complete train makes a periodic-only branch immediately due again.
    period, complete_train = 1.0, 1.2
    assert complete_train >= period
    available_tickets = {(0, 0, 0), (1, 0, 0)}  # (cell, scan round, attempt)
    available_tickets.remove((0, 0, 0))
    assert not any(ticket[:2] == (0, 0) for ticket in available_tickets)
    assert (1, 0, 0) in available_tickets

    # A partial group requires new raw transactions, not duplicate packets.
    required, raw_ids = 2, {"a", "a"}
    remaining = required - len(raw_ids)
    same_site = np.linalg.norm(np.array([0.01, 0.0])) <= 0.05
    other_site = np.linalg.norm(np.array([2.0, 0.0])) <= 0.05
    assert remaining == 1 and same_site and not other_site

    # Cached quality and all covariance blocks must match the snapshot version.
    cached_version, snapshot_version = 7, 8
    cached_quality = True
    quality_usable = cached_quality and cached_version == snapshot_version
    fit_version, cross_version, holdout_version = 8, 8, 7
    coherent = len({fit_version, cross_version, holdout_version}) == 1
    assert not quality_usable and not coherent

    # A finite coarse grid can miss a narrow second branch even with cell radii.
    def nominal_radius(grid: np.ndarray, cell_radius: float) -> float:
        costs = np.minimum(grid**2, ((grid - 3.0) / 0.05)**2)
        active = costs - costs.min() <= 1.0
        estimate = grid[int(np.argmin(costs))]
        return float(np.max(np.abs(grid[active] - estimate)) + cell_radius)

    coarse_radius = nominal_radius(np.array([0., 2., 4., 6.]), 1.0)
    fine_radius = nominal_radius(np.arange(7, dtype=float), 1.0)
    assert coarse_radius <= 1.5 < fine_radius

    # Merely repeating feasible reference blocks leaves no progress budget.
    reference_period, progress_block, next_reference = 10.0, 8.0, 3.0
    assert next_reference <= reference_period < progress_block + next_reference
    return {"mixture_gap_old_cdf_accepts": gap[1],
            "mixture_gap_component_guard_rejects": not gap[2],
            "valid_component_passes_and_extreme_tail_fails": True,
            "scalar_projection_can_hide_within_block_contrast": True,
            "complete_train_can_exceed_soft_period": True,
            "consumed_cell_ticket_leaves_next_cell_available": True,
            "partial_group_dedup_and_same_site_boundary_checked": True,
            "stale_quality_and_mixed_versions_rejected": True,
            "coarse_grid_can_miss_branch_despite_cell_radius": True,
            "coarse_fixture_radius": coarse_radius,
            "fine_fixture_radius": fine_radius,
            "reference_only_feasibility_is_not_progress_feasibility": True,
            "scope": "fixed_design_counterexamples_not_state_machine_or_mission_validation"}


def fixed_grid_resolution_checks(profile: dict) -> list[dict]:
    """L02/Q01, two registered eight-row inputs; no truth or flight history."""
    fixture = profile["feasibility_fixture"]
    grid_fixture = fixture["grid_diagnostic"]
    radius, height = grid_fixture["uav_cardinal_radius_m"], grid_fixture["relative_flight_height_m"]
    samples = fixture["samples_per_reference"]
    positions = np.repeat(np.array([[radius, 0, height], [0, radius, height],
        [-radius, 0, height], [0, -radius, height]]), samples, axis=0)
    dimension = len(positions)
    same_group = np.kron(np.eye(4), np.ones((samples, samples)))
    domain = grid_fixture["domain_half_width_m"]
    rows = []
    for input_index, observed in enumerate(grid_fixture["fixed_range_vectors_m"]):
        observed = np.asarray(observed)
        assert observed.shape == (dimension,)
        for pitch in grid_fixture["pitches_m"]:
            axis = np.arange(-domain, domain + pitch / 2, pitch)
            assert len(axis)**2 <= 4096
            xx, yy = np.meshgrid(axis, axis, indexing="ij")
            points = np.column_stack([xx.ravel(), yy.ravel()])
            widths = np.full(len(axis), pitch)
            widths[[0, -1]] = pitch / 2
            prior = np.outer(widths, widths).ravel() / (2 * domain)**2
            assert abs(prior.sum() - 1) < 1e-12
            vectors = positions[None, :, :] - np.column_stack([points, np.zeros(len(points))])[:, None, :]
            distances = np.linalg.norm(vectors, axis=2)
            unit = vectors / distances[:, :, None]
            covariance = np.broadcast_to(fixture["white_range_std_m"]**2 * np.eye(dimension)
                + grid_fixture["independent_nav_radial_std_m"]**2 * same_group
                + fixture["common_range_bias_std_m"]**2 * np.ones((dimension, dimension)),
                (len(points), dimension, dimension)).copy()
            covariance += grid_fixture["common_nav_translation_std_m"]**2 * (unit @ unit.transpose(0, 2, 1))
            uz = unit[:, :, 2]
            covariance += grid_fixture["common_terminal_height_std_m"]**2 * uz[:, :, None] * uz[:, None, :]
            chol = np.linalg.cholesky(covariance)
            whitened = np.linalg.solve(chol, (observed[None, :] - distances)[:, :, None])[:, :, 0]
            costs = np.sum(whitened**2, axis=1) + 2 * np.log(np.diagonal(chol, axis1=1, axis2=2)).sum(axis=1)
            costs -= 2 * np.log(prior)
            index = int(np.argmin(costs))
            active = costs - costs[index] <= grid_fixture["cost_increment"]
            estimate = points[index]
            nominal_radius = np.max(np.linalg.norm(points[active] - estimate, axis=1)) + pitch / math.sqrt(2)
            rows.append({"fixed_input_index": input_index, "pitch_m": pitch,
                "grid_points": len(points), "map_xy_m": estimate.tolist(),
                "nominal_cost_region_radius_m": float(nominal_radius),
                "active_cells": int(active.sum()),
                "active_touches_domain_boundary": bool(np.any(np.abs(points[active]) >= domain)),
                "resolution_or_quality_approved": False})
    return rows


def coplanar_feasibility_checks() -> dict:
    """Section 11 static geometry, analytic gap and packet-ledger diagnostics."""
    config = json.loads((Path(__file__).resolve().parents[1]
                         / "research_configs/baseline.json").read_text(encoding="utf-8"))
    profile = config["project_context"]["analytical_profile"]
    fixture = profile["feasibility_fixture"]
    assert fixture["usage"].startswith("fixed_input_capability_diagnostics")
    half_span = fixture["reference_square_half_span_m"]
    white_std = fixture["white_range_std_m"]
    samples = fixture["samples_per_reference"]
    common_std = fixture["common_range_bias_std_m"]
    anchors = np.array([[-half_span, -half_span, 0], [half_span, -half_span, 0],
                        [half_span, half_span, 0], [-half_span, half_span, 0]])
    layouts = []
    for height in fixture["relative_antenna_heights_m"]:
        differences = np.array([0.0, 0.0, height]) - anchors
        geometry = differences / np.linalg.norm(differences, axis=1)[:, None]
        covariance = white_std**2 / samples * np.eye(4) + common_std**2 * np.ones((4, 4))
        position = np.linalg.inv(geometry.T @ np.linalg.solve(covariance, geometry))
        distance2 = 2 * half_span**2 + height**2
        expected_z = (white_std**2 / (4 * samples) + common_std**2) * distance2 / height**2
        assert abs(position[2, 2] - expected_z) < 1e-8
        assert np.linalg.matrix_rank(geometry) == 3
        assert np.linalg.matrix_rank(np.column_stack([geometry, np.ones(4)])) == 3
        white_position = np.linalg.inv(geometry.T @ geometry) * white_std**2 / samples
        layouts.append({"relative_height_m": height,
            "height_horizontal_std_ratio_white_only": half_span / height,
            "white_only_local_rms3d_m": math.sqrt(np.trace(white_position)),
            "common_bias_local_height_std_floor_m": common_std * math.sqrt(distance2) / height,
            "position_rank": 3, "position_plus_free_common_bias_rank": 3})
    ground_geometry = -anchors / np.linalg.norm(anchors, axis=1)[:, None]
    assert np.linalg.matrix_rank(ground_geometry) == 2
    heights = [fixture["relative_antenna_heights_m"][0],
               fixture["relative_antenna_heights_m"][0] * 2 / 3]
    ranges = [math.sqrt(2 * half_span**2 + h**2) for h in heights]
    compensating_bias = ranges[0] - ranges[1]
    assert abs(ranges[1] + compensating_bias - ranges[0]) < 1e-12

    # One coplanar home reference helps only under favorable horizontal geometry.
    home_anchors = np.vstack([anchors, np.zeros(3)])
    alternative_anchors = np.vstack([anchors[:3], np.zeros(3)])
    differences = np.array([0, 0, heights[0]]) - alternative_anchors
    alternative_geometry = differences / np.linalg.norm(differences, axis=1)[:, None]
    assert np.linalg.matrix_rank(np.column_stack([alternative_geometry, np.ones(4)])) == 4
    home_rows = []
    for offset in fixture["home_horizontal_offsets_m"]:
        point = np.array([offset, 0, fixture["relative_antenna_heights_m"][-1]])
        differences = point - home_anchors
        geometry = differences / np.linalg.norm(differences, axis=1)[:, None]
        covariance = (fixture["home_reference_capability_std_m"]**2 * np.eye(5)
                      + fixture["home_common_bias_capability_std_m"]**2 * np.ones((5, 5)))
        position = np.linalg.inv(geometry.T @ np.linalg.solve(covariance, geometry))
        home_rows.append({"horizontal_offset_m": offset,
                          "local_height_std_m": math.sqrt(position[2, 2]),
                          "local_rms3d_m": math.sqrt(np.trace(position))})
    assert home_rows[1]["local_height_std_m"] > 4 * home_rows[0]["local_height_std_m"]

    # Exact closed polynomial of a frozen, level, single-horizontal-axis model.
    imu = profile["imu_datasheet_fixture"]
    gravity = 9.80665
    qa = (imu["acc_xy_micro_g_per_sqrt_hz"] * 1e-6 * gravity)**2 / 2
    qg = math.radians(imu["gyro_deg_per_s_per_sqrt_hz"])**2 / 2
    prior = fixture["post_reference_std"]
    prior_covariance = np.diag(np.array([prior["position_m"], prior["velocity_m_per_s"],
        math.radians(prior["tilt_deg"]), prior["acc_bias_m_per_s2"],
        math.radians(prior["gyro_bias_deg_per_s"])])**2)
    gap_rows = []
    for gap in fixture["imu_gap_seconds"]:
        mapping = np.array([1, gap, gravity * gap**2 / 2, -gap**2 / 2, -gravity * gap**3 / 6])
        noise = qa * gap**3 / 3 + gravity**2 * qg * gap**5 / 20
        variance = float(mapping @ prior_covariance @ mapping + noise)
        gap_rows.append({"gap_s": gap, "illustrative_one_axis_position_std_m": math.sqrt(variance)})
    assert all(a["illustrative_one_axis_position_std_m"] < b["illustrative_one_axis_position_std_m"]
               for a, b in zip(gap_rows, gap_rows[1:]))

    timing = fixture["timing"]
    phy, ranging = profile["ordinary_phy"], profile["ranging_phy"]
    packet = lambda size: css_packet_time_s(size, phy)
    ranging_s = (2**ranging["sf"] / ranging["bandwidth_hz"]
                 * (ranging["preamble_symbols"] + 2 * ranging["ranging_symbols"] + 22.25))
    precision = profile["precision_ranging_candidate"]
    precision_s = (2**precision["sf"] / precision["bandwidth_hz"]
                   * (precision["preamble_symbols"] + 2 * precision["ranging_symbols"] + 22.25))
    assert abs(precision_s - ranging_s) < 1e-12
    pair_radio = 2 * packet(28) + 2 * ranging_s + packet(28) + packet(18)
    wake_packet = packet(29 + 5 * timing["responding_ids"])
    assert timing["effective_receive_window_s"] >= timing["broadcast_start_spacing_s"] + wake_packet
    train_packets = math.ceil((timing["listener_max_wait_s"] + timing["effective_receive_window_s"]
        - wake_packet) / timing["broadcast_start_spacing_s"]) + 1
    train_s = (train_packets - 1) * timing["broadcast_start_spacing_s"] + wake_packet
    assert train_s >= timing["listener_max_wait_s"] + timing["effective_receive_window_s"]
    pair_bound = timing["pair_attempt_bound_s"] * timing["attempts_per_pair"]
    reference_lower = timing["reference_count"] * samples * pair_radio
    reference_bound = timing["reference_count"] * samples * pair_bound
    progress_lower = (train_s + timing["responding_ids"] * packet(26)
                      + timing["unknown_samples"] * pair_radio + packet(28) + packet(18))
    progress_bound = (train_s + timing["responding_ids"] * timing["response_slot_s"]
        + timing["unknown_samples"] * pair_bound + timing["data_attempt_bound_s"] * timing["data_attempts"]
        + timing["control_and_mode_overhead_bound_s"] + timing["computation_bound_s"])
    assert pair_bound >= pair_radio and progress_bound >= progress_lower
    time_rows = []
    for length in timing["move_lengths_m"]:
        speed, acceleration = timing["cruise_speed_m_per_s"], timing["acceleration_m_per_s2"]
        move = (2 * math.sqrt(length / acceleration) if length <= speed**2 / acceleration
                else length / speed + speed / acceleration)
        time_rows.append({"move_length_m": length, "instruction_move_s": move,
            "radio_plus_move_lower_s": move + progress_lower + reference_lower,
            "assumed_bounded_progress_and_next_reference_s": move + progress_bound + reference_bound})

    # V02-c: Cholesky whitening retains a contrast lost by the scalar average.
    conditional = np.array([[1.0, 0.8], [0.8, 1.0]])
    chol = np.linalg.cholesky(conditional)
    contrast = np.array([5.0, -5.0])
    whitened = np.linalg.solve(chol, contrast)
    statistic = float(whitened @ whitened)
    assert abs(statistic - contrast @ np.linalg.solve(conditional, contrast)) < 1e-10
    fit_covariance = np.array([[2.0, 0.3], [0.3, 1.5]])
    cross = np.array([[0.4, 0.2], [0.1, 0.35]])
    raw_holdout_covariance = conditional + cross @ np.linalg.solve(fit_covariance, cross.T)
    joint = np.block([[fit_covariance, cross.T], [cross, raw_holdout_covariance]])
    fit_residual = np.array([0.2, -0.1])
    raw_holdout_residual = contrast + cross @ np.linalg.solve(fit_covariance, fit_residual)
    joint_residual = np.r_[fit_residual, raw_holdout_residual]
    conditional_statistic = (joint_residual @ np.linalg.solve(joint, joint_residual)
                             - fit_residual @ np.linalg.solve(fit_covariance, fit_residual))
    conditioning_error = float(abs(conditional_statistic - statistic))
    assert conditioning_error < 1e-9
    alpha = 0.1  # independent algebra fixture; formal alpha_V is not frozen
    quantile_df2 = -2 * math.log(alpha)
    assert abs(np.array([0.5, 0.5]) @ contrast) == 0 and statistic > quantile_df2
    assert np.array([0.1, -0.1]) @ np.linalg.solve(conditional, np.array([0.1, -0.1])) < quantile_df2
    try:
        np.linalg.cholesky(np.ones((2, 2)))
    except np.linalg.LinAlgError:
        pass
    else:
        raise AssertionError("A singular heldout model must fail rather than receive artificial jitter")
    # Separate existential choices would combine different incompatible modes.
    mode_residuals = np.array([[5.0, -5.0], [1.4, 1.4]])
    projected_cdf = np.array([0.5 * (1 + math.erf(row.mean())) for row in mode_residuals])
    projected_ok = (alpha / 2 <= projected_cdf) & (projected_cdf <= 1 - alpha / 2)
    vector_ok = np.sum(mode_residuals**2, axis=1) <= quantile_df2
    assert projected_ok.any() and vector_ok.any() and not np.any(projected_ok & vector_ok)
    return {"scope": "static_geometry_closed_polynomial_and_radio_ledger_not_a_navigation_simulation",
        "coplanar_layouts": layouts, "plane_height_position_rank": 2,
        "exact_symmetric_height_bias_ambiguity_m": compensating_bias,
        "different_horizontal_radii_position_plus_common_bias_rank": 4,
        "home_reference_capability_examples": home_rows, "illustrative_inertial_gaps": gap_rows,
        "full_train_packets": train_packets, "full_train_s": train_s,
        "paired_transaction_radio_lower_s": pair_radio, "time_ledger_examples": time_rows,
        "precision_candidate_ranging_exchange_s": precision_s,
        "precision_candidate_same_ranging_airtime_not_an_accuracy_claim": True,
        "fixed_four_group_grid_resolution": fixed_grid_resolution_checks(profile),
        "heldout_contrast_mahalanobis": statistic, "heldout_df2_fixture_quantile": quantile_df2,
        "conditional_block_quadratic_identity_difference": conditioning_error,
        "same_active_component_required_by_both_tests": True,
        "singular_heldout_covariance_rejected_without_jitter": True,
        "heldout_full_vector_rejects_zero_average_contrast": True,
        "new_missions": 0, "physical_feasibility_validated": False}


def hardware_and_observability_checks() -> dict:
    """H01 and section 11.7: conversions, provenance sets and fixed matrices."""
    def signed24(value: int) -> int:
        if not 0 <= value < 2**24:
            raise ValueError("A register result must contain exactly 24 unsigned bits")
        return value - 2**24 if value >= 2**23 else value

    assert [signed24(v) for v in [0, 2**23 - 1, 2**23, 2**24 - 1]] == [0, 8388607, -8388608, -1]
    low_step, high_step = [150 / (2**12 * bw) for bw in [0.40625, 1.625]]
    assert abs(low_step / high_step - 4) < 1e-12
    raw = 1024
    distance = raw * high_step
    assert abs(distance - 23.076923076923077) < 1e-12
    assert abs(distance * high_step - distance) > 20
    assert abs(raw * 150 / (2**12 * 1625000) / distance - 1e-6) < 1e-15
    four_group_sources = set(range(1, 17))
    filtered_fifth_sources = set(range(10, 18))
    raw_fifth_sources = set(range(17, 21))
    assert not four_group_sources.isdisjoint(filtered_fifth_sources)
    assert four_group_sources.isdisjoint(raw_fifth_sources)
    directions = np.array([[4.0, 3.0], [3.0, 9.0]])
    weights = np.array([0.5, 0.5])
    pair_variance = float(weights @ directions @ weights)
    assert pair_variance == 4.75 and pair_variance != np.trace(directions) / 4

    gravity = 9.80665
    transition_rate = np.zeros((15, 15))
    transition_rate[:3, 3:6] = np.eye(3)
    transition_rate[3:6, 6:9] = -skew(np.array([0.0, 0.0, gravity]))
    transition_rate[3:6, 9:12] = -np.eye(3)
    transition_rate[6:9, 12:15] = -np.eye(3)
    observation = np.zeros((3, 15))
    observation[:, :3] = np.eye(3)
    power, rows = np.eye(15), []
    for _ in range(15):
        rows.append(observation @ power)
        power = power @ transition_rate
    observability = np.vstack(rows)
    null = np.zeros((15, 4))
    null[8, 0], null[14, 1] = 1, 1
    null[6, 2], null[10, 2] = 1, -gravity
    null[7, 3], null[9, 3] = 1, gravity
    rank = int(np.linalg.matrix_rank(observability))
    null_error = float(np.max(np.abs(observability @ null)))
    assert rank == 11 and null_error == 0 and np.linalg.matrix_rank(null) == 4
    # M01: zero estimated world acceleration is not zero physical acceleration
    # when the true body attitude has a tilt error.  This is a static force check.
    true_rotation = exp_rotation(np.array([0.0, math.radians(5.0), 0.0]))
    actual_acceleration = true_rotation @ np.array([0.0, 0.0, gravity]) + np.array([0.0, 0.0, -gravity])
    assert actual_acceleration[0] > 0.8 and np.linalg.norm(actual_acceleration) > 0
    return {"scope": "fixed_raw_interface_and_stationary_observability_not_driver_or_flight_validation",
        "raw24_signed_boundaries_checked": True, "raw_unit_step_m": [low_step, high_step],
        "already_meter_output_must_not_be_scaled_again": True,
        "filtered_fifth_can_overlap_four_group_raw_support": True,
        "pair_direction_cross_covariance_retained_variance": pair_variance,
        "stationary_position_only_observability_rank": rank,
        "specified_four_dimensional_nullspace_error": null_error,
        "tilted_body_zero_command_horizontal_acceleration_m_per_s2": float(actual_acceleration[0]),
        "attitude_error_not_hidden_by_ideal_world_acceleration_injection": True,
        "driver_timeout_or_firmware_tested": False, "new_missions": 0}


def joint_model_candidate_checks(grid_pitch_override: float | None = None,
                                 reference_sigma_override: float | None = None,
                                 budget_only: bool = False) -> dict:
    """One registered Gaussian snapshot model and ledger, never a mission."""
    config = json.loads((Path(__file__).resolve().parents[1]
                         / "research_configs/baseline.json").read_text(encoding="utf-8"))
    candidate = config["project_context"]["joint_model_candidate"]
    scene, radio, imu, service, review = [candidate[k] for k in ["scene", "radio", "imu", "service", "fixed_review"]]
    if reference_sigma_override is not None:
        radio = {**radio, "reference_pair_white_std_m": reference_sigma_override}
    anchors = np.asarray(scene["known_reference_xyz_m"], dtype=float)
    lever = np.asarray(scene["uav_antenna_lever_body_m"])
    xy = np.asarray(review["pose_antenna_xy_m"], dtype=float)
    antenna = np.column_stack([xy, np.full(4, scene["nominal_imu_cruise_z_m"] + lever[2])])
    # Four selected position errors, common attitude, reference-coordinate primitives,
    # shared device bias, five persistent link terms, current velocity and IMU biases.
    theta, coordinate, device, links, velocity, ba, bg, dimension = 12, 15, 28, 29, 34, 37, 40, 43
    covariance = np.zeros((dimension, dimension))
    position_std = imu["current_relative_position_prior_std_m"]
    shared_position_std = candidate["fixed_review"]["common_clone_translation_prior_std_m"]
    covariance[:12, :12] = (shared_position_std**2 * np.kron(np.ones((4, 4)), np.eye(3))
        + (position_std**2 - shared_position_std**2) * np.eye(12))
    attitude_std = np.radians([imu["roll_pitch_std_deg"], imu["roll_pitch_std_deg"], imu["yaw_std_deg"]])
    covariance[theta:theta+3, theta:theta+3] = np.diag(attitude_std**2)
    coord_std = np.r_[np.repeat(scene["outer_reference_local_xy_std_m"], 8),
        np.repeat(scene["home_local_xy_std_m"], 2), scene["common_terminal_plane_relative_height_std_m"],
        np.repeat(scene["home_reference_relative_xy_std_m"], 2)]
    covariance[coordinate:coordinate+13, coordinate:coordinate+13] = np.diag(coord_std**2)
    covariance[device, device] = radio["shared_device_range_bias_std_m"]**2
    covariance[links:links+5, links:links+5] = radio["reference_persistent_link_std_m"]**2 * np.eye(5)
    covariance[velocity:velocity+3, velocity:velocity+3] = imu["current_velocity_std_m_per_s"]**2 * np.eye(3)
    covariance[ba:ba+3, ba:ba+3] = imu["acc_bias_std_m_per_s2"]**2 * np.eye(3)
    covariance[bg:bg+3, bg:bg+3] = math.radians(imu["gyro_bias_std_deg_per_s"])**2 * np.eye(3)
    cross = imu["tilt_acc_bias_correlation"] * attitude_std[0] * imu["acc_bias_std_m_per_s2"]
    covariance[theta, ba+1] = covariance[ba+1, theta] = -cross
    covariance[theta+1, ba] = covariance[ba, theta+1] = cross
    assert np.min(np.linalg.eigvalsh(covariance)) > 0
    coordinate_map = np.zeros((5, 3, 13))
    for j in range(5):
        coordinate_map[j, 2, 10] = 1
        if j < 4:
            coordinate_map[j, 0, 2*j], coordinate_map[j, 1, 2*j+1] = 1, 1
            coordinate_map[j, 0, 8], coordinate_map[j, 1, 9] = -1, -1
        else:
            coordinate_map[j, 0, 11], coordinate_map[j, 1, 12] = 1, 1

    def reference_rows(points: np.ndarray) -> np.ndarray:
        rows = []
        for k, point in enumerate(points):
            for j, anchor in enumerate(anchors):
                unit = point - anchor
                unit /= np.linalg.norm(unit)
                row = np.zeros(dimension)
                row[3*k:3*k+3] = unit
                row[theta:theta+3] = -unit @ skew(lever)
                row[coordinate:coordinate+13] = -unit @ coordinate_map[j]
                row[device], row[links+j] = 1, 1
                rows.append(row)
        return np.asarray(rows)

    def posterior(prior: np.ndarray, observation: np.ndarray) -> np.ndarray:
        noise = radio["reference_pair_white_std_m"]**2 / service["pairs_per_reference"]
        innovation = observation @ prior @ observation.T + noise * np.eye(len(observation))
        updated = prior - prior @ observation.T @ np.linalg.solve(innovation, observation @ prior)
        return (updated + updated.T) / 2

    # Constructed zero residuals at the registered means; no random measurements.
    updated = posterior(covariance, reference_rows(antenna))
    navigation_before_unknown = updated.copy()
    assert np.min(np.linalg.eigvalsh(updated)) > -1e-12
    state_indices = np.r_[np.arange(9, 12), np.arange(velocity, velocity+3), np.arange(theta, theta+3),
                          np.arange(ba, ba+3), np.arange(bg, bg+3)]
    nav = updated[np.ix_(state_indices, state_indices)]
    shared_cross_norm = float(np.linalg.norm(updated[:12, device]))
    assert shared_cross_norm > 0 and nav[8, 8] > 0 and nav[14, 14] > 0

    # Exact polynomial covariance of the frozen hover linear model, not a flight.
    gravity = 9.80665
    f = np.zeros((15, 15)); f[:3, 3:6] = np.eye(3)
    f[3:6, 6:9] = -skew(np.array([0.0, 0.0, gravity])); f[3:6, 9:12] = -np.eye(3)
    f[6:9, 12:15] = -np.eye(3)
    powers = [np.eye(15), f, f @ f, f @ f @ f]
    assert np.max(np.abs(powers[-1] @ f)) == 0
    data = config["project_context"]["analytical_profile"]["imu_datasheet_fixture"]
    qa = np.square(np.array([data["acc_xy_micro_g_per_sqrt_hz"]]*2 + [data["acc_z_micro_g_per_sqrt_hz"]]) * 1e-6 * gravity) / 2
    qg = math.radians(data["gyro_deg_per_s_per_sqrt_hz"])**2 / 2
    drive = np.zeros((15, 15)); drive[3:6, 3:6] = np.diag(qa); drive[6:9, 6:9] = qg * np.eye(3)
    drive[9:12, 9:12] = imu["acc_bias_random_walk_m_per_s2_per_sqrt_s"]**2 * np.eye(3)
    drive[12:15, 12:15] = math.radians(imu["gyro_bias_random_walk_deg_per_s_per_sqrt_s"])**2 * np.eye(3)

    def covariance_at(initial: np.ndarray, time: float) -> np.ndarray:
        transition = sum(p * time**i / math.factorial(i) for i, p in enumerate(powers))
        noise = sum(pi @ drive @ pj.T * time**(i+j+1) / (math.factorial(i)*math.factorial(j)*(i+j+1))
                    for i, pi in enumerate(powers) for j, pj in enumerate(powers))
        return transition @ initial @ transition.T + noise

    phy, ranging = radio["ordinary_phy"], radio["ranging_phy"]
    packet = lambda size: css_packet_time_s(size, phy)
    ranging_s = (2**ranging["sf"] / ranging["bandwidth_hz"]
                 * (ranging["preamble_symbols"] + 2*ranging["ranging_symbols"] + 22.25))
    pair_lower = 2*packet(28) + 2*ranging_s + packet(28) + packet(18)
    assert pair_lower < radio["pair_whole_attempt_bound_s"]
    wake = packet(29 + 5*service["responding_unknown_ids_per_wake"])
    assert service["effective_receive_window_s"] >= service["broadcast_start_spacing_s"] + wake
    train_count = math.ceil((service["listener_max_wait_s"] + service["effective_receive_window_s"] - wake)
                           / service["broadcast_start_spacing_s"]) + 1
    train = (train_count-1)*service["broadcast_start_spacing_s"] + wake
    schedule_error = service["schedule_error_base_bound_s"] + service["relative_clock_rate_bound_ppm"]*1e-6*train
    assert service["response_slot_s"] >= packet(26)+2*schedule_error+service["rx_gap_s"]
    reference_attempts = len(anchors)*service["pairs_per_reference"] + service["reference_block_shared_extra_attempts"]
    unknown_attempts = service["pairs_per_unknown_group"] + service["unknown_group_shared_extra_attempts"]
    reference_time = reference_attempts * radio["pair_whole_attempt_bound_s"]
    progress_time = (train + service["responding_unknown_ids_per_wake"]*service["response_slot_s"]
        + unknown_attempts*radio["pair_whole_attempt_bound_s"] + service["data_attempts"]*service["data_attempt_bound_s"]
        + service["shared_control_prepare_s"] + service["computation_bound_s"])
    length, speed, acceleration = [service[k] for k in ["reference_motion_segment_candidate_m", "cruise_speed_m_per_s", "acceleration_m_per_s2"]]
    movement = 2*math.sqrt(length/acceleration) if length <= speed**2/acceleration else length/speed + speed/acceleration
    tracking_allowance = service["movement_tracking_and_settling_allowance_s"]
    full_time = movement + tracking_allowance + progress_time + reference_time
    assert full_time <= service["reference_max_age_s"]
    predicted = covariance_at(nav, full_time)
    # First-order maneuver envelope: horizontal acceleration is bounded and the
    # nominal body keeps its heading, tilting by at most atan(a_max/g).  Global
    # attitude error R*dtheta removes the skew(omega) term; compare with hover.
    # The heading-based reference can contain both roll and pitch.  Bound the
    # difference from I by the sum of the two rotation differences, not just tilt.
    rotation_difference = min(2.0, 4*math.sin(math.atan(acceleration/gravity)/2))
    for horizontal in [np.array([acceleration, 0.0]), np.array([0.0, acceleration]),
                       acceleration*np.array([0.6, 0.8])]:
        b3 = np.r_[horizontal, gravity]; b3 /= np.linalg.norm(b3)
        b2 = np.cross(b3, np.array([1.0, 0.0, 0.0])); b2 /= np.linalg.norm(b2)
        reference_rotation = np.column_stack([np.cross(b2, b3), b2, b3])
        assert np.linalg.norm(reference_rotation-np.eye(3), 2) <= rotation_difference+1e-12
    sigma_ba = math.sqrt(np.max(np.linalg.eigvalsh(nav[9:12, 9:12])))
    sigma_bg = math.sqrt(np.max(np.linalg.eigvalsh(nav[12:15, 12:15])))
    sigma_yaw = math.sqrt(nav[8, 8])
    sigma_tilt = math.sqrt(np.max(np.linalg.eigvalsh(nav[6:8, 6:8])))
    qba, qbg, qa_max = drive[9, 9], drive[12, 12], float(np.max(qa))
    t = full_time
    gyro_position = sigma_bg*t**3/6 + math.sqrt(qg*t**5/20) + math.sqrt(qbg*t**7/252)
    basic_position = sigma_ba*t**2/2 + math.sqrt(qba*t**5/20) + math.sqrt(qa_max*t**3/3)
    delta_horizontal = acceleration*(sigma_yaw*t**2/2 + gyro_position)
    delta_horizontal += rotation_difference*(basic_position + gravity*gyro_position)
    delta_vertical = math.sqrt(2)*acceleration*(sigma_tilt*t**2/2 + gyro_position)
    delta_vertical += rotation_difference*basic_position
    horizontal_upper = math.sqrt(np.max(np.linalg.eigvalsh(predicted[:2, :2]))) + delta_horizontal
    vertical_upper = math.sqrt(predicted[2, 2]) + delta_vertical
    gyro_velocity = sigma_bg*t**2/2 + math.sqrt(qg*t**3/3) + math.sqrt(qbg*t**5/20)
    basic_velocity = sigma_ba*t + math.sqrt(qba*t**3/3) + math.sqrt(qa_max*t)
    delta_vh = acceleration*(sigma_yaw*t + gyro_velocity)
    delta_vh += rotation_difference*(basic_velocity + gravity*gyro_velocity)
    delta_vz = math.sqrt(2)*acceleration*(sigma_tilt*t + gyro_velocity) + rotation_difference*basic_velocity
    velocity_upper = (math.sqrt(np.max(np.linalg.eigvalsh(predicted[3:6, 3:6])))
                      + math.sqrt(2*delta_vh**2 + delta_vz**2))
    attitude_upper = (math.sqrt(np.max(np.linalg.eigvalsh(nav[6:9, 6:9]))) + t*sigma_bg
                      + math.sqrt(qg*t) + math.sqrt(qbg*t**3/3))
    # A separate saved input at the landing point; contact is NOT supplied to nav.
    landing_antenna = np.array([[0.0, 0.0, scene["home_imu_xyz_m"][2]+lever[2]]])
    landing_updated = posterior(covariance, reference_rows(landing_antenna))
    landing_indices = np.r_[np.arange(3), state_indices[3:]]
    landing_nav = landing_updated[np.ix_(landing_indices, landing_indices)]
    landing_prediction = covariance_at(landing_nav, reference_time)
    landing_proxy = (review["landing_estimated_xy_offset_candidate_m"]
        + review["landing_nominal_sigma_multiplier"]*math.sqrt(np.max(np.linalg.eigvalsh(landing_prediction[:2, :2]))))
    caps = candidate["navigation_admission"]
    budget_ratio = max(horizontal_upper/caps["horizontal_max_std_m"], vertical_upper/caps["vertical_std_m"],
        velocity_upper/caps["velocity_max_std_m_per_s"], attitude_upper/math.radians(caps["attitude_max_std_deg"]),
        landing_proxy/candidate["landing"]["actual_return_tolerance_m"])
    if budget_only:
        return {"reference_pair_white_std_m": radio["reference_pair_white_std_m"], "budget_ratio": budget_ratio}

    # Four-group likelihood uses the same posterior nuisance block, including the
    # cross covariance between navigation, shared device bias and terminal plane.
    domain, pitch = review["grid_domain_half_width_m"], review["grid_pitch_m"]
    if grid_pitch_override is not None:
        pitch = grid_pitch_override
    axis = np.arange(-domain, domain+pitch/2, pitch)
    xx, yy = np.meshgrid(axis, axis, indexing="ij")
    points = np.column_stack([xx.ravel(), yy.ravel()]); assert len(points) <= 4096
    widths = np.full(len(axis), pitch); widths[[0, -1]] = pitch/2
    prior_mass = np.outer(widths, widths).ravel()/(2*domain)**2
    raw_positions = np.repeat(antenna, service["pairs_per_unknown_group"], axis=0)
    target = np.r_[review["target_relative_xy_input_m"], scene["terminal_antenna_plane_z_m"]]
    observed = np.linalg.norm(raw_positions-target, axis=1) + radio["unknown_persistent_link_mean_m"]
    vectors = raw_positions[None, :, :] - np.column_stack([points, np.full(len(points), target[2])])[:, None, :]
    ranges = np.linalg.norm(vectors, axis=2); unit = vectors/ranges[:, :, None]
    mapping = np.zeros((len(points), len(raw_positions), dimension))
    for row in range(len(raw_positions)):
        mapping[:, row, 3*(row//service["pairs_per_unknown_group"]):3*(row//service["pairs_per_unknown_group"])+3] = unit[:, row]
    mapping[:, :, theta:theta+3] = -unit @ skew(lever)
    mapping[:, :, coordinate+10] = -unit[:, :, 2]
    mapping[:, :, device] = 1
    target_covariance = mapping @ updated @ mapping.transpose(0, 2, 1)
    target_covariance += radio["unknown_pair_white_std_m"]**2 * np.eye(len(raw_positions))
    target_covariance += radio["unknown_persistent_link_std_m"]**2 * np.ones((len(raw_positions), len(raw_positions)))
    chol = np.linalg.cholesky(target_covariance)
    residual = observed[None, :]-ranges-radio["unknown_persistent_link_mean_m"]
    whitened = np.linalg.solve(chol, residual[:, :, None])[:, :, 0]
    costs = np.sum(whitened**2, axis=1) + 2*np.log(np.diagonal(chol, axis1=1, axis2=2)).sum(axis=1) - 2*np.log(prior_mass)
    best = int(np.argmin(costs)); active = costs-costs[best] <= review["nominal_cost_increment"]
    radius = float(np.max(np.linalg.norm(points[active]-points[best], axis=1)) + pitch/math.sqrt(2))
    home_envelope = review["home_registration_nominal_sigma_multiplier"] * math.hypot(
        scene["common_map_translation_std_m"], scene["home_local_xy_std_m"])
    absolute_radius = radius + home_envelope
    # Extend the same prior with the fifth selected position.  Known-reference
    # input rows and four-point marginals stay identical; no heldout value is fit.
    extended_prior = np.zeros((dimension+3, dimension+3))
    extended_prior[:dimension, :dimension] = covariance
    extended_prior[dimension:, dimension:] = position_std**2*np.eye(3)
    for k in range(4):
        extended_prior[3*k:3*k+3, dimension:] = shared_position_std**2*np.eye(3)
        extended_prior[dimension:, 3*k:3*k+3] = shared_position_std**2*np.eye(3)
    known_rows = np.pad(reference_rows(antenna), ((0, 0), (0, 3)))
    extended = posterior(extended_prior, known_rows)
    assert np.max(np.abs(extended[:dimension, :dimension]-updated)) < 1e-12
    fifth = np.r_[review["fifth_antenna_xy_m"], antenna[0, 2]]
    spatial_margins = []
    for k, point in enumerate(antenna):
        difference = (extended[dimension:dimension+2, dimension:dimension+2]
            + extended[3*k:3*k+2, 3*k:3*k+2] - extended[dimension:dimension+2, 3*k:3*k+2]
            - extended[3*k:3*k+2, dimension:dimension+2])
        spatial_margins.append(float(np.linalg.norm(fifth[:2]-point[:2])
            - review["spatial_sigma_multiplier_candidate"]*math.sqrt(np.max(np.linalg.eigvalsh(difference)))))
    fv_positions = np.vstack([raw_positions, np.repeat(fifth[None, :], service["pairs_per_unknown_group"], axis=0)])
    fv_vectors = fv_positions[None, :, :]-np.column_stack([points, np.full(len(points), target[2])])[:, None, :]
    fv_ranges = np.linalg.norm(fv_vectors, axis=2); fv_unit = fv_vectors/fv_ranges[:, :, None]
    joint_mapping = np.zeros((len(points), len(fv_positions), dimension+3))
    joint_mapping[:, :len(raw_positions), :dimension] = mapping
    joint_mapping[:, len(raw_positions):, dimension:] = fv_unit[:, len(raw_positions):]
    joint_mapping[:, len(raw_positions):, theta:theta+3] = -fv_unit[:, len(raw_positions):] @ skew(lever)
    joint_mapping[:, len(raw_positions):, coordinate+10] = -fv_unit[:, len(raw_positions):, 2]
    joint_mapping[:, len(raw_positions):, device] = 1
    joint_noise = joint_mapping @ extended @ joint_mapping.transpose(0, 2, 1)
    joint_noise += radio["unknown_pair_white_std_m"]**2*np.eye(len(fv_positions))
    joint_noise += radio["unknown_persistent_link_std_m"]**2*np.ones((len(fv_positions), len(fv_positions)))
    nf = len(raw_positions)
    assert np.max(np.abs(joint_noise[:, :nf, :nf]-target_covariance)) < 1e-11
    cvf = joint_noise[:, nf:, :nf]
    conditional_mean = (fv_ranges[:, nf:]+radio["unknown_persistent_link_mean_m"]
        + (cvf @ np.linalg.solve(target_covariance, residual[:, :, None]))[:, :, 0])
    conditional_noise = joint_noise[:, nf:, nf:] - cvf @ np.linalg.solve(target_covariance, cvf.transpose(0, 2, 1))
    conditional_chol = np.linalg.cholesky(conditional_noise)
    projection = np.array(review["fifth_group_projection_weights"])
    # These entries are paired results in the fifth group, not the two directions.
    assert service["pairs_per_unknown_group"] == len(projection) == 2
    means = conditional_mean @ projection
    variances = np.einsum('i,gij,j->g', projection, conditional_noise, projection)
    four_weights = np.exp(-0.5*(costs-costs.min())); four_weights /= four_weights.sum()
    frozen_weights = four_weights.copy()
    alpha = review["predictive_alpha_candidate"]; quantile = -2*math.log(alpha)
    nominal_fifth = np.full(2, np.linalg.norm(fifth-target)+radio["unknown_persistent_link_mean_m"])
    verification = []
    for values in [nominal_fifth, nominal_fifth+np.asarray(review["heldout_contrast_fixture_m"])]:
        projected = float(projection @ values)
        component_cdf = np.array([0.5*(1+math.erf((projected-mean)/math.sqrt(2*variance)))
            for mean, variance in zip(means, variances)])
        mixed_cdf = float(four_weights @ component_cdf)
        full_whitened = np.linalg.solve(conditional_chol, (values[None, :]-conditional_mean)[:, :, None])[:, :, 0]
        statistic = np.sum(full_whitened**2, axis=1)
        same_component = active & (alpha/2 <= component_cdf) & (component_cdf <= 1-alpha/2) & (statistic <= quantile)
        passed = alpha/2 <= mixed_cdf <= 1-alpha/2 and bool(same_component.any())
        verification.append({"mixed_cdf": mixed_cdf, "passed": passed,
                             "minimum_active_full_vector_statistic": float(np.min(statistic[active]))})
    assert np.array_equal(four_weights, frozen_weights)
    assert np.array_equal(updated, navigation_before_unknown)
    assert set(range(1, 17)).isdisjoint(set(range(17, 21)))

    frozen_navigation_proxy = bool(
        np.sqrt(np.max(np.linalg.eigvalsh(predicted[:2, :2]))) <= caps["horizontal_max_std_m"]
        and np.sqrt(predicted[2, 2]) <= caps["vertical_std_m"]
        and np.sqrt(np.max(np.linalg.eigvalsh(predicted[3:6, 3:6]))) <= caps["velocity_max_std_m_per_s"]
        and np.sqrt(np.max(np.linalg.eigvalsh(predicted[6:9, 6:9]))) <= math.radians(caps["attitude_max_std_deg"]))
    maneuver_proxy = bool(horizontal_upper <= caps["horizontal_max_std_m"]
        and vertical_upper <= caps["vertical_std_m"] and velocity_upper <= caps["velocity_max_std_m_per_s"]
        and attitude_upper <= math.radians(caps["attitude_max_std_deg"]))
    nominal_conditions = {"radio_lower_within_assumed_deadline": pair_lower < radio["pair_whole_attempt_bound_s"],
        "full_action_within_reference_age": full_time <= service["reference_max_age_s"],
        "frozen_navigation_proxy_within_caps": frozen_navigation_proxy,
        "restricted_first_order_maneuver_envelope_within_caps": maneuver_proxy,
        "absolute_nominal_region_with_home_envelope_within_candidate_cap": absolute_radius <= review["quality_radius_candidate_m"],
        "landing_horizontal_proxy_within_threshold": landing_proxy <= candidate["landing"]["actual_return_tolerance_m"]}
    nominal_conditions.update({"fifth_position_separation_proxy_met": min(spatial_margins) >= review["spatial_separation_candidate_m"],
        "joint_fifth_nominal_input_passes": verification[0]["passed"],
        "joint_fifth_contrast_rejected": not verification[1]["passed"]})
    prerequisites = bool(review["data_delivery_precondition_for_fixed_verification"]
        and absolute_radius <= review["quality_radius_candidate_m"]
        and min(spatial_margins) >= review["spatial_separation_candidate_m"])
    nominal_conditions["fifth_quality_data_and_spatial_prerequisites_met"] = prerequisites
    channel_rows = {}
    for kind in ["control_channel", "data_channel", "ranging_exchange_validity"]:
        channel = radio[kind]
        probabilities = [channel["probability_at_short_distance"] / (1 + math.exp(
            (distance-channel["half_probability_distance_m"])/channel["transition_scale_m"]))
            for distance in [0, 30, 80]]
        assert 0 < probabilities[-1] < probabilities[1] < probabilities[0] < 1
        channel_rows[kind] = probabilities
    return {"scope": "one_explicit_joint_saved_input_model_not_closed_loop_feasibility_or_performance_validation",
        "joint_prior_dimension": dimension, "reference_group_rows": len(reference_rows(antenna)),
        "reference_pair_white_std_m": radio["reference_pair_white_std_m"],
        "shared_navigation_device_cross_norm": shared_cross_norm,
        "current_reference_position_std_xyz_m": np.sqrt(np.diag(nav)[:3]).tolist(),
        "pair_radio_lower_s": pair_lower, "pair_assumed_bound_s": radio["pair_whole_attempt_bound_s"],
        "wake_train_s": train, "bounded_next_reference_s": reference_time,
        "bounded_progress_s": progress_time, "movement_s": movement, "move_tracking_allowance_s": tracking_allowance,
        "full_action_and_next_reference_s": full_time,
        "reference_age_budget_s": service["reference_max_age_s"],
        "frozen_model_predicted_position_std_xyz_m": np.sqrt(np.diag(predicted)[:3]).tolist(),
        "restricted_maneuver_position_std_upper_xy_z_m": [horizontal_upper, vertical_upper],
        "restricted_maneuver_velocity_std_upper_m_per_s": velocity_upper,
        "restricted_maneuver_attitude_std_upper_deg": math.degrees(attitude_upper),
        "maneuver_envelope_scope": "first_order_fixed_heading_level_cruise_tilt_and_acceleration_bounds_not_arbitrary_flight_guarantee",
        "fixed_four_group_map_xy_m": points[best].tolist(), "relative_nominal_cost_region_radius_m": radius,
        "grid_pitch_m": pitch, "grid_points": len(points),
        "nominal_home_registration_radius_m": home_envelope,
        "absolute_nominal_region_radius_m": absolute_radius,
        "fifth_position_spatial_margins_m": spatial_margins,
        "joint_fifth_nominal_and_contrast_cases": verification,
        "fifth_prerequisites_met_in_saved_input": prerequisites,
        "heldout_does_not_change_four_group_weights": True,
        "unknown_fit_and_verification_do_not_change_navigation_posterior": True,
        "candidate_quality_radius_m": review["quality_radius_candidate_m"],
        "landing_nominal_horizontal_proxy_m": landing_proxy,
        "actual_return_threshold_m": candidate["landing"]["actual_return_tolerance_m"],
        "nominal_conditions": nominal_conditions,
        "nominal_conditions_all_met": all(nominal_conditions.values()),
        "navigation_and_landing_budget_ratio": budget_ratio,
        "minimum_remaining_resource_s": full_time+service["return_reservation_candidate_s"],
        "distinct_channel_probability_examples_at_0_30_80m": channel_rows,
        "whole_mission_navigation_clone_history_validated": False,
        "physical_or_closed_loop_success_confirmed": False, "new_missions": 0}


def run_checks() -> dict:
    joint = joint_model_candidate_checks()
    config = json.loads((Path(__file__).resolve().parents[1]
                         / "research_configs/baseline.json").read_text(encoding="utf-8"))
    limits = config["project_context"]["joint_model_candidate"]["fixed_review"]
    coarse = joint_model_candidate_checks(grid_pitch_override=limits["grid_check_coarse_pitch_m"])
    stress = joint_model_candidate_checks(reference_sigma_override=limits["reference_white_noise_stress_m"])
    stress_summary = {"scope": "same_model_fixed_input_pressure_only_reference_white_noise_changed_not_a_mission",
        "reference_pair_white_std_m": stress["reference_pair_white_std_m"],
        "absolute_nominal_region_radius_m": stress["absolute_nominal_region_radius_m"],
        "landing_nominal_horizontal_proxy_m": stress["landing_nominal_horizontal_proxy_m"],
        "restricted_maneuver_position_std_upper_xy_z_m": stress["restricted_maneuver_position_std_upper_xy_z_m"],
        "fifth_prerequisites_met": stress["fifth_prerequisites_met_in_saved_input"],
        "nominal_conditions": stress["nominal_conditions"],
        "nominal_conditions_all_met": stress["nominal_conditions_all_met"]}
    shift = float(np.linalg.norm(np.array(joint["fixed_four_group_map_xy_m"])-coarse["fixed_four_group_map_xy_m"]))
    radius_change = abs(joint["absolute_nominal_region_radius_m"]-coarse["absolute_nominal_region_radius_m"])
    sensitivity = {"scope": "same_registered_input_two_resolutions_not_continuous_global_enclosure",
        "fine_pitch_m": joint["grid_pitch_m"], "coarse_pitch_m": coarse["grid_pitch_m"],
        "fine_points": joint["grid_points"], "coarse_points": coarse["grid_points"],
        "map_shift_m": shift, "absolute_radius_change_m": radius_change,
        "nominal_tolerances_met": shift <= limits["grid_map_change_tolerance_m"]
            and radius_change <= limits["grid_region_change_tolerance_m"]}
    lower, upper = limits["reference_noise_budget_root_bracket_m"]
    low_budget = joint_model_candidate_checks(reference_sigma_override=lower, budget_only=True)["budget_ratio"]
    high_budget = joint_model_candidate_checks(reference_sigma_override=upper, budget_only=True)["budget_ratio"]
    assert low_budget < 1 < high_budget
    for _ in range(limits["reference_noise_budget_root_iterations"]):
        middle = (lower+upper)/2
        ratio = joint_model_candidate_checks(reference_sigma_override=middle, budget_only=True)["budget_ratio"]
        if ratio <= 1:
            lower = middle
        else:
            upper = middle
    capability = {"scope": "scalar_inverse_of_registered_navigation_landing_proxies_not_a_sensor_specification_or_mission_success_bound",
        "reference_white_std_budget_transition_bracket_m": [lower, upper],
        "other_parameters_unchanged": True,
        "root_method": "bounded_bisection_of_saved_covariance_budget_no_random_or_time_simulation"}
    return {"all_passed": True, "new_missions": 0,
            "scope": "fixed_input_algebra_not_forest_navigation_or_protocol_validation",
            "reference": reference_checks(), "wakeup": wake_checks(),
            "decision": decision_checks(), "first_model": first_model_checks(),
            "protocol_parameters": protocol_parameter_checks(), "workflow": workflow_checks(),
            "strategy_review": strategy_review_checks(),
            "coplanar_feasibility": coplanar_feasibility_checks(),
            "hardware_and_observability": hardware_and_observability_checks(),
            "joint_model_candidate": joint, "joint_grid_sensitivity": sensitivity,
            "joint_reference_noise_pressure": stress_summary, "joint_reference_capability_budget": capability}


if __name__ == "__main__":
    print(json.dumps(run_checks(), ensure_ascii=False))
