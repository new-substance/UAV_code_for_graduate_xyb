"""Audit completed managed batches and write one compact current overview.

This never runs a simulation or pools versions into a common success rate.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from audit_managed_batch import audit


def summarize(root: Path) -> dict:
    cases = []
    unfinished = []
    for directory in sorted(root.iterdir()):
        if not directory.is_dir() or directory.name.startswith("."):
            continue
        if not (directory / "manifest.json").exists():
            continue
        if not (directory / "summary.json").exists():
            unfinished.append(directory.name)
            continue
        report = audit(directory)
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))["manifest"]
        cfg = manifest["configuration_expanded"]
        for line in (directory / "outcomes.jsonl").read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            out = row["outcome"]
            nodes = out.get("nodes", [])
            if isinstance(nodes, dict):
                nodes = [nodes]
            accepted_errors = [n["true_error_m"] for n in nodes
                               if n["online_completed"] and n.get("true_error_m") is not None]
            cases.append({"label": directory.name, "seed": int(row["seed"]),
                "count": manifest["options"]["count"], "rule": manifest["options"]["rule"],
                "purpose": manifest.get("review_purpose", manifest["purpose"]),
                "source_sha256": manifest["identity"]["source"],
                "configuration_sha256": manifest["identity"]["configuration"],
                "model": {
                    "flight_height_m": cfg["scene"]["flight_height_m"],
                    "online_range_white_std_m": cfg["ranging"]["range_white_std_m"],
                    "physical_range_white_std_m": cfg["sim"]["radio_truth_range_white_std_m"],
                    "time_allowance_s": cfg["stage"]["platform_resource_s"],
                },
                "reason": out["reason"], "complete": out["end_to_end_success"],
                "discovered": sum(n["discovered"] for n in nodes),
                "online_completed": sum(n["online_completed"] for n in nodes),
                "false_accept_count": out.get("false_accept_count"),
                "best_accepted_true_error_m": min(accepted_errors, default=None),
                "worst_accepted_true_error_m": max(accepted_errors, default=None),
                "unfinished_addresses": [n["address"] for n in nodes if not n["online_completed"]],
                "ack_missing_addresses": [n["address"] for n in nodes if n["online_completed"] and not n["stop_ack"]],
                "awake_addresses": [n["address"] for n in nodes if not n["deep_sleep"]],
                "returned": out.get("returned"), "return_error_m": out.get("return_error_m"),
                "total_time_s": out.get("total_time_s"),
                "feedback_decisions": out.get("position_feedback_decisions"),
                "feedback_delivered": out.get("position_feedback_delivered"),
                "batch_bytes": report["bytes"], "audit_passed": report["all_passed"]})
    batch_bytes = {c["label"]: c["batch_bytes"] for c in cases}
    return {"purpose": "current_file_inventory_and_per_case_audit_not_pooled_performance",
        "cases": cases, "unfinished_batches": unfinished,
        "writer_lock_exists": (root / ".writer_lock").exists(),
        "completed_batch_bytes": sum(batch_bytes.values())}


def main() -> None:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=repo / "results/current")
    parser.add_argument("--output", type=Path, default=repo / "results/current_overview.json")
    args = parser.parse_args()
    result = summarize(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes((json.dumps(result, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    print(json.dumps({"cases": len(result["cases"]), "completed_bytes": result["completed_batch_bytes"],
        "unfinished_batches": result["unfinished_batches"], "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
