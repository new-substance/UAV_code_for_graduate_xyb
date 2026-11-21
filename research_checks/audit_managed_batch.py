"""Audit compact managed results independently of the MATLAB controller."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def check_region_prediction(detail: dict, cfg: dict) -> None:
    """Reconstruct witness statistics directly, without MATLAB helper code."""
    import numpy as np

    fit, check, bundle = detail["fit"], detail["check"], detail["input"]
    if not check.get("witness_xy_m"):
        assert not check["passed"]
        return
    point = np.asarray(check["witness_xy_m"], dtype=float).reshape(2)
    target = np.r_[point, cfg["scene"]["terminal_height_m"]]
    anchors = np.asarray(bundle["anchors"], dtype=float)
    delta = anchors - target[:, None]
    distances = np.linalg.norm(delta, axis=0)
    G = np.zeros((5, 15))
    for k in range(5):
        G[k, 3*k:3*k+3] = delta[:, k] / distances[k]
    C = np.asarray(bundle["Cradio"]) + G @ np.asarray(bundle["Pnav"]) @ G.T
    K = np.linalg.solve(C[:4, :4], C[:4, 4]).T
    variance = float(C[4, 4] - K @ C[:4, 4])
    residuals = np.asarray(bundle["ranges"]).reshape(5) - distances
    residual = float(residuals[4] - K @ residuals[:4])
    bounds = np.asarray(bundle["model_bias_bounds_m"]).reshape(5)
    bound = float(bounds[4] + np.abs(K) @ bounds[:4])
    statistic = max(0, abs(residual)-bound)**2 / variance
    for actual, expected in ((check["variance"], variance), (check["residual"], residual),
                             (check["model_residual_bound_m"], bound), (check["statistic"], statistic)):
        assert math.isclose(actual, expected, rel_tol=1e-7, abs_tol=1e-8)
    L = np.linalg.cholesky(C[:4, :4])
    whitened = np.linalg.solve(L, residuals[:4])
    signs = np.array([[1 if (j >> k) & 1 else -1 for k in range(4)] for j in range(16)])
    a = max(np.linalg.norm(np.linalg.solve(L, signs[j]*bounds[:4])) for j in range(16))
    lower = max(0, np.linalg.norm(whitened)-a)**2 + 2*np.log(np.diag(L)).sum()
    assert lower <= fit["region_reference_upper"] + cfg["stage"]["region_gamma"] + 1e-6
    assert np.linalg.norm(point-np.asarray(fit["x"]).reshape(2)) + fit["datum_allowance_m"] <= fit["radius"] + 1e-6
    assert check["passed"] == (fit["quality"] and statistic <= cfg["stage"]["verification_gate"])


def read_lines(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def check_information_prediction(choice: dict, cfg: dict) -> None:
    """Direct covariance inverses independently check the rank-one proxy."""
    import numpy as np

    x = np.asarray(choice["estimate_xy_m"], dtype=float).reshape(2)
    observed = np.asarray(choice["observed_xy_m"], dtype=float).reshape(-1, 2)
    height = cfg["scene"]["flight_height_m"] - cfg["scene"]["terminal_height_m"]
    s2 = cfg["ranging"]["range_white_std_m"]**2 / cfg["priority"]["cold_services_per_position"]
    b2 = cfg["ranging"]["device_persistent_bias_std_m"]**2
    def rows(points):
        delta = x - points
        return delta / np.sqrt(np.sum(delta**2, axis=1) + height**2)[:, None]
    def minimum(H):
        if not len(H):
            return 0.0
        C = s2*np.eye(len(H)) + b2*np.ones((len(H), len(H)))
        return max(0.0, float(np.linalg.eigvalsh(H.T @ np.linalg.solve(C, H)).min()))
    H = rows(observed)
    before = minimum(H)
    for row in choice["candidates"]:
        if row[4] and "allowed_reference_ids" in choice:
            allowed = choice["allowed_reference_ids"]
            if isinstance(allowed, (int, float)):
                allowed = [allowed]
            assert row[1] in allowed
        q = np.asarray(row[2:4]).reshape(1, 2)
        expected_gain = max(0.0, minimum(np.vstack([H, rows(q)]))-before) / (1+row[7])
        assert math.isclose(row[9], expected_gain, rel_tol=1e-7, abs_tol=1e-10)
        slant = math.sqrt(float(np.sum((x-q.reshape(2))**2)) + height**2)
        assert math.isclose(row[6], slant, rel_tol=1e-10, abs_tol=1e-10)
        risk = max(0.0, slant+choice["radius_proxy_m"]-cfg["scene"]["design_service_slant_m"])
        assert math.isclose(row[8], risk, abs_tol=1e-9)


def audit(directory: Path) -> dict:
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    seeds = manifest["seeds"]
    if isinstance(seeds, (int, float)):
        seeds = [seeds]
    outcome_lines = (directory / "outcomes.jsonl").read_bytes().splitlines(keepends=True)
    rows = [json.loads(line) for line in outcome_lines]
    assert len(rows) == len(seeds) and [r["seed"] for r in rows] == seeds
    limits = manifest["limits"]
    cfg = manifest["manifest"]["configuration_expanded"]
    files = [p for p in directory.rglob("*") if p.is_file()]
    total = sum(p.stat().st_size for p in files)
    assert total <= limits["batch_bytes"]
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    assert total == summary["bytes_before_summary"] + (directory / "summary.json").stat().st_size
    assert total + summary["active_bytes_before_batch"] <= limits["active_bytes"]
    reports = []
    for row_index, row in enumerate(rows):
        seed = int(row["seed"])
        event_path = directory / f"events_{seed}.jsonl"
        events = read_lines(event_path)
        event_bytes = event_path.stat().st_size if event_path.exists() else 0
        outcome_bytes = len(outcome_lines[row_index])
        assert event_bytes == row["event_bytes"] and event_bytes + outcome_bytes <= limits["case_bytes"]
        assert outcome_bytes <= 16384
        complete_trace = not row["event_budget_exhausted"]
        published = set()
        qualified = set()
        for event in events:
            kind = event["kind"]
            data = event["data"]
            if kind == "SPATIAL_SERVICE":
                assert data["address"] not in published, "New RTT service after publication"
                children = data["constituent_ids"]
                if isinstance(children, (int, float)):
                    children = [children]
                assert len(children) == len(set(children))
                assert data["qualified_spatial_record"] == (len(children) == cfg["priority"]["cold_services_per_position"])
                if "reserved_block_cap" in data:
                    minimum = math.ceil(cfg["priority"]["cold_services_per_position"] / cfg["priority"]["services_per_atomic_block"])
                    assert minimum <= data["reserved_block_cap"] <= cfg["priority"]["burst_recovery_blocks"]
                    cost = data["reserved_block_cap"]*(data["atomic_reserve_s"]+data["image_reserve_s"])
                    cost += data["compute_reserve_s"]+data["stop_reserve_s"]
                    assert math.isclose(cost, data["reserved_transaction_s"], abs_tol=1e-8)
                    assert cost + data["return_reserve_s"] <= data["start_remaining_s"]+1e-8
                    assert data["end_s"]-data["start_s"] <= cost-data["compute_reserve_s"]-data["stop_reserve_s"]+1e-6
                if data["constituent_ids"]:
                    qualified.add(data["address"])
            elif kind == "POSITION_FEEDBACK" and "alternatives" in data["choice"]:
                choice = data["choice"]
                if choice.get("information_proxy") == "white_plus_common_bias":
                    check_information_prediction(choice, cfg)
                if "coverage_priority" in choice:
                    expected_priority = (len(qualified) < cfg["scene"]["terminal_counts"] and
                                         choice["coverage_vertices_remaining"] > 0)
                    assert choice["coverage_priority"] == expected_priority
                    if expected_priority:
                        allowed = choice["allowed_reference_ids"]
                        if isinstance(allowed, (int, float)):
                            allowed = [allowed]
                        assert set(allowed) == {choice["current_reference_id"], choice["next_coverage_reference"]}
                rows = choice["alternatives"]
                if rows and isinstance(rows[0], (int, float)):
                    rows = [rows]
                def key(row):
                    owner, phase, candidate, _, cost, risk, gain = row
                    if choice["rule"] == "cost":
                        return phase, 0, 0, cost, owner, candidate
                    return (phase, risk, 0, cost, owner, candidate) if phase == 0 else (phase, -gain, risk, cost, owner, candidate)
                selected = min(rows, key=key)
                assert selected[0] == data["address"] and selected[2] == choice["candidate_id"]
            elif kind == "LOCALIZATION" and data["detail"]["check"]:
                detail = data["detail"]
                check = detail["check"]
                fit = detail["fit"]
                bundle = detail["input"]
                assert fit["record_ids"] == bundle["record_ids"][:4]
                assert fit["nav_version"] == bundle["version"] == check["nav_version"]
                assert check["heldout_id"] not in fit["record_ids"]
                groups = bundle["constituent_ids"]
                members = [v for group in groups for v in (group if isinstance(group, list) else [group])]
                assert len(members) == len(set(members)), "Raw services reused in fit/heldout"
                if check.get("kind") == "grid_conditional":
                    check_region_prediction(detail, cfg)
            elif kind == "PUBLISHED":
                assert data["address"] not in published
                proof = data["proof"]
                assert proof["passed"] and len(set(proof["fit_ids"])) == 4
                assert proof["heldout_id"] not in proof["fit_ids"]
                published.add(data["address"])
        outcome = row["outcome"]
        if "nodes" in outcome:
            nodes = outcome["nodes"]
            if isinstance(nodes, dict):
                nodes = [nodes]
            target = cfg["localization"]["offline_position_error_limit_m"]
            for node in nodes:
                if complete_trace:
                    assert node["discovered"] == (node["address"] in qualified)
                    assert node["online_completed"] == (node["address"] in published)
                else:
                    assert node["address"] not in qualified or node["discovered"]
                    assert node["address"] not in published or node["online_completed"]
                if node["online_completed"]:
                    assert node["phase"] == "DONE"
                    error = node["true_error_m"]
                    assert node["false_accept"] == (error is None or not math.isfinite(error) or error > target)
                if node.get("true_position_xy_m") and node["estimate_xy_m"]:
                    if all(v is not None and math.isfinite(v) for v in node["estimate_xy_m"]):
                        expected_error = math.dist(node["estimate_xy_m"], node["true_position_xy_m"])
                        assert math.isclose(node["true_error_m"], expected_error, rel_tol=1e-9, abs_tol=1e-9)
                    else:
                        assert node["true_error_m"] is None
            if "physical_home_xyz_m" in outcome:
                return_error = math.dist(outcome["physical_home_xyz_m"], outcome["physical_final_xyz_m"])
                assert math.isclose(outcome["return_error_m"], return_error, rel_tol=1e-9, abs_tol=1e-9)
                expected_return = (outcome["physically_landed"] and outcome["return_command_completed"] and
                                   outcome["return_arrived"] and return_error <= cfg["motion_and_budget"]["return_position_tolerance_m"])
                assert outcome["returned"] == expected_return
            complete = all(n["online_completed"] and not n["false_accept"] and
                           n["stop_ack"] and n["deep_sleep"] for n in nodes) and outcome["returned"]
            assert complete == outcome["end_to_end_success"]
        reports.append({"seed": seed, "reason": outcome["reason"],
                        "end_to_end_success": outcome["end_to_end_success"],
                        "returned": outcome.get("returned"),
                        "feedback_delivered": outcome.get("position_feedback_delivered", 0),
                        "event_bytes": event_bytes,"complete_event_trace": complete_trace})
    return {"all_passed": True, "cases": len(rows), "bytes": total,
            "purpose": "development_audit_not_independent_success_rate", "outcomes": reports}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.directory), ensure_ascii=False))


if __name__ == "__main__":
    main()
