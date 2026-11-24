"""Fixed-input checks of the forest design equations; never a mission simulator.

All numeric inputs are illustrative algebra fixtures, not a radio/aircraft
profile. See 论文总纲 sections 3.2-3.7, 5.2 and 6.1-6.5.
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


def run_checks() -> dict:
    return {"all_passed": True, "new_missions": 0,
            "scope": "fixed_input_algebra_not_forest_navigation_or_protocol_validation",
            "reference": reference_checks(), "wakeup": wake_checks(),
            "decision": decision_checks(), "first_model": first_model_checks(),
            "protocol_parameters": protocol_parameter_checks()}


if __name__ == "__main__":
    print(json.dumps(run_checks(), ensure_ascii=False))
