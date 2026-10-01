#!/usr/bin/env python3
"""Run the worthwhile inventory suite and require intended mutation failures."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--pwsh", default="pwsh")
    parser.add_argument("--makensis", default="makensis")
    parser.add_argument("--jobs", type=int, choices=(1, 2, 3, 4), default=2)
    args = parser.parse_args()
    root, out = args.source_root.resolve(), args.output_dir.resolve()
    virtual = root / "VirtualQuest"
    spec = importlib.util.spec_from_file_location("assurance", root / "tools/verify-assurance.py")
    assurance = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assurance)
    identity = assurance.source_identity(root, virtual)
    if (out / "result.json").exists():
        raise ValueError("Use a fresh output directory; stale evidence cannot survive a failed run")
    out.mkdir(parents=True, exist_ok=True)
    record = {"schemaVersion": 2, "sources": identity, "sourcesUnchanged": False,
              "success": False, "fullSuite": False, "dirty": True, "suites": {}}
    try:
        subprocess.run([sys.executable, str(virtual / "formal/verify-extension.py"),
                        "--source-root", str(root), "--output-dir", str(out),
                        "--compiler", args.compiler, "--pwsh", args.pwsh,
                        "--makensis", args.makensis, "--jobs", str(args.jobs)], check=True)
        record = json.loads((out / "result.json").read_text())
        record["success"] = False
        inventory = record["executions"]["inventory"]
        if not inventory.get("success") or not inventory.get("sourcesUnchanged"):
            raise ValueError("Inventory execution failed or changed sources")
        assurance.validate_inventory_mutants(inventory.get("mutants"))
        capture = record["executions"]["capture"]
        capture_controls = [dict(row, log=(out / "capture" / (row["name"] + ".log")).read_text())
                            for row in capture["mutants"]]
        assurance.validate_capture_mutants(capture_controls)
        capture["assertionControls"] = capture_controls
        suite = record["suites"]["inventory-extension"]
        suite["negativeControls"] = {
            "mutants": inventory["mutants"],
            "acceptanceFixtures": assurance.inventory_negative_control_fixtures(),
            "captureMutants": capture_controls,
            "captureAcceptanceFixtures": assurance.capture_negative_control_fixtures(capture_controls),
        }
        record["sourcesUnchanged"] = identity == assurance.source_identity(root, virtual)
        record["dirty"] = any(assurance.git(repo, "status", "--porcelain") for repo in (root, virtual))
        record["success"] = record["sourcesUnchanged"]
        registry = json.loads((virtual / "formal/inventory-scope.json").read_text())
        now = {"A01", "A02", "A03", "P01", "P02", "P03", "P04", "P05", "S04", "S07", "N05", "N06"}
        expected = {"inventory-extension": sorted(set(registry["items"]) - now - {"V09"})}
        if len(registry["items"]) != 44 or len(expected["inventory-extension"]) != 31:
            raise ValueError("Inventory scope changed without updating acceptance")
        assurance.validate(record, identity, expected, required={"inventory-extension"},
                           release=False, require_negative_controls=True)
        print("All 26 C++ and four Python mutants reached their intended assertions; 222 acceptance fixtures passed.")
    except BaseException:
        record["success"] = False
        raise
    finally:
        (out / "result.json").write_text(json.dumps(record, indent=2) + "\n")


if __name__ == "__main__":
    main()
