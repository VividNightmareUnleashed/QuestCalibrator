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
    parser.add_argument("--jobs", type=int, choices=(1, 2, 3, 4), default=2)
    parser.add_argument("--public-only", action="store_true",
                        help="Exclude VirtualQuest-specific checks from public CI")
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
                        "--jobs", str(args.jobs),
                        *(["--public-only"] if args.public_only else [])], check=True)
        record = json.loads((out / "result.json").read_text())
        record["success"] = False
        inventory = record["executions"]["inventory"]
        if not inventory.get("success") or not inventory.get("sourcesUnchanged"):
            raise ValueError("Inventory execution failed or changed sources")
        suite_name = "public-inventory-extension" if args.public_only else "inventory-extension"
        suite = record["suites"][suite_name]
        if args.public_only:
            if set(record["executions"]) != {"inventory", "installer"}:
                raise ValueError("Private execution leaked into public inventory checks")
            assurance.validate_cpp_mutants(inventory.get("mutants"), assurance.PUBLIC_INVENTORY_MUTANTS)
            suite["negativeControls"] = {
                "mutants": inventory["mutants"],
                "acceptanceFixtures": assurance.cpp_negative_control_fixtures(assurance.PUBLIC_INVENTORY_MUTANTS),
                "privateSelectionsRefused": inventory.get("privateSelectionsRefused"),
            }
        else:
            assurance.validate_inventory_mutants(inventory.get("mutants"))
            capture = record["executions"]["capture"]
            capture_controls = [dict(row, log=(out / "capture" / (row["name"] + ".log")).read_text())
                                for row in capture["mutants"]]
            assurance.validate_capture_mutants(capture_controls)
            capture["assertionControls"] = capture_controls
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
        selected = set(registry["items"]) - now - {"V09"}
        if args.public_only:
            selected = {name for name in selected if not name.startswith('V')}
        expected = {suite_name: sorted(selected)}
        if not now | {"V09"} <= set(registry["items"]) or not selected:
            raise ValueError("The inventory scope no longer holds the Now contracts, V09 and an extension")
        assurance.validate(record, identity, expected, required={suite_name},
                           release=False, require_negative_controls=True)
        controls = suite["negativeControls"]
        fixtures = controls["acceptanceFixtures"] + controls.get("captureAcceptanceFixtures", 0)
        mutants = len(controls["mutants"])
        print(f"All {mutants} public mutants reached their intended assertions; {fixtures} acceptance fixtures passed."
              if args.public_only else
              f"All {mutants} C++ and {len(controls['captureMutants'])} Python mutants reached their intended assertions; "
              f"{fixtures} acceptance fixtures passed.")
    except BaseException:
        record["success"] = False
        raise
    finally:
        (out / "result.json").write_text(json.dumps(record, indent=2) + "\n")


if __name__ == "__main__":
    main()
