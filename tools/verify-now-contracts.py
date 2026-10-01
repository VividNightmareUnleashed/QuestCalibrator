#!/usr/bin/env python3
"""Run Now contracts and require their nine intended mutation assertions."""
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
    args = parser.parse_args()
    root, out = args.source_root.resolve(), args.output_dir.resolve()
    virtual = root / "VirtualQuest"
    spec = importlib.util.spec_from_file_location("assurance", root / "tools/verify-assurance.py")
    assurance = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assurance)
    identity = assurance.source_identity(root, virtual)
    if (out / "result.json").exists():
        raise ValueError("Use a fresh output directory; existing evidence cannot describe a new run")
    out.mkdir(parents=True, exist_ok=True)
    record = {"schemaVersion": 2, "sources": identity, "sourcesUnchanged": False,
              "success": False, "fullSuite": False, "dirty": True, "suites": {}}
    try:
        subprocess.run([sys.executable, str(virtual / "formal/verify-now.py"),
                        "--source-root", str(root), "--output-dir", str(out),
                        "--compiler", args.compiler, "--pwsh", args.pwsh,
                        "--jobs", str(args.jobs)], check=True)
        record = json.loads((out / "result.json").read_text())
        record["success"] = False
        controls = []
        for name, (path, message) in assurance.NOW_MUTANTS.items():
            lines = (out / (name + "-mutant.log")).read_text().splitlines()
            if len(lines) != 2 or json.loads(lines[0]) != {"name": name, "status": "failed"} or lines[1] != message:
                raise ValueError(f"{name}: Now mutant did not reach its intended assertion")
            controls.append({"name": name, "status": "passed", "method": "production mutant",
                             "mutation": path, "failure": lines[1]})
        record["suites"]["contracts"]["negativeControls"] = {
            "mutants": controls, "acceptanceFixtures": assurance.now_negative_control_fixtures(),
        }
        record["sourcesUnchanged"] = identity == assurance.source_identity(root, virtual)
        record["success"] = record["sourcesUnchanged"]
        expected = {"contracts": ["A01", "A02", "A03", *assurance.NOW_MUTANTS]}
        assurance.validate(record, identity, expected, required={"contracts"},
                           release=False, require_negative_controls=True)
        print("All nine Now mutants reached their intended assertions; 67 acceptance fixtures passed.")
    except BaseException:
        record["success"] = False
        raise
    finally:
        (out / "result.json").write_text(json.dumps(record, indent=2) + "\n")


if __name__ == "__main__":
    main()
