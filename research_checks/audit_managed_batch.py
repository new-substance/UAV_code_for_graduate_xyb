"""Audit compact managed results independently of the MATLAB controller."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def read_lines(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def audit(directory: Path) -> dict:
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    seeds = manifest["seeds"]
    if isinstance(seeds, (int, float)):
        seeds = [seeds]
    outcome_lines = (directory / "outcomes.jsonl").read_bytes().splitlines(keepends=True)
    rows = [json.loads(line) for line in outcome_lines]
    assert len(rows) == len(seeds) and [r["seed"] for r in rows] == seeds
    limits = manifest["limits"]
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
                if data["constituent_ids"]:
                    qualified.add(data["address"])
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
            cfg = manifest["manifest"]["configuration_expanded"]
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
