"""Read saved V8 summaries; never launch a simulation or reclassify outcomes."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def inspect(source: Path) -> dict:
    rows = []
    hashes = []
    timeline = []
    for seed in range(93001, 93021):
        path = source / f"seed_{seed}" / "result.json"
        raw = path.read_bytes()
        data = json.loads(raw.decode("utf-8-sig"))
        hashes.append({"seed": seed, "sha256": hashlib.sha256(raw).hexdigest()})
        if data["online_completed"]:
            continue
        fit = data["fits"][-1] if data["fits"] else {}
        fields = ("reason", "relative_radius_m", "region_radius", "local_radius",
                  "datum_allowance_m", "model_allowance_m", "radius", "sensitivity_ok")
        counts = {}
        for event in data["events"]:
            kind = event["kind"]
            counts[kind] = counts.get(kind, 0) + 1
            if seed != 93005:
                continue
            payload = event["data"]
            if kind == "FOUR_FIT" and payload["fit"]["quality"]:
                timeline.append({"time_s": event["time_s"], "kind": kind,
                                 "fit_reason": payload["fit"]["reason"]})
            if kind == "EXTRA_SERVICE_PLANNED" and payload["purpose"] == "heldout":
                timeline.append({"time_s": event["time_s"], "kind": kind,
                                 "target_xy_m": payload["target_xy_m"]})
            if event["time_s"] > 980 and kind in ("SERVICE", "POINT_SKIPPED"):
                timeline.append({"time_s": event["time_s"], "kind": kind, "data": payload})
        rows.append({"seed": seed, "exit": data["reason"],
                     "true_error_m_offline_only": data["true_error_m"],
                     "online_completed_unchanged": False,
                     "last_fit": {k: fit.get(k) for k in fields},
                     "verification_rounds": data["verification_rounds"],
                     "counts": counts})
    return {"kind": "saved_input_review_not_new_evaluation", "new_missions": 0,
            "source_hashes": hashes, "unfinished": rows, "seed_93005_timeline": timeline,
            "interpretation": [
                "Small offline point errors do not establish online quality or completion.",
                "93005 planned three heldouts: first incomplete radio service, then two admission skips; no heldout obtained.",
                "Candidate history must distinguish a visited/attempted point from a qualified spatial measurement.",
                "93017 is marginal at 20.08796 m; 93007 has a much wider grid region. One threshold tweak cannot explain both.",
                "93020 reaches a constrained boundary; inherited interior sensitivity is explicitly invalid there."
            ]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = inspect(args.source)
    encoded = (json.dumps(report, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    if len(encoded) > 100_000:
        raise ValueError("Saved-input review exceeds its 100 KB output cap")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream:
        stream.write(encoded)
    print(json.dumps({"unfinished": len(report["unfinished"]), "new_missions": 0,
                      "bytes": len(encoded), "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
