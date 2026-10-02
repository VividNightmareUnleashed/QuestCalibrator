#!/usr/bin/env python3
"""Run one private VirtualQuest proof command with its output kept off the log.

This repository is public, so anyone can read its Actions logs and artifacts,
while VirtualQuest is private. The command's output goes to a file in the
runner's temp folder that is never printed or uploaded; investigate a failure
locally. With --raw, the record the command writes is reduced to what the
release evidence needs and only that copy is kept (--result): for a check.ps1
record, check names, tools, statuses and source hashes; for an inventory
extension record, what tools/assemble-assurance.py checks of it, accepted here
on the whole record first. The log shows how many checks passed and the names
of any that did not.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def reduce_proof(raw):
    return {
        "schemaVersion": raw.get("schemaVersion"),
        "complete": raw.get("complete"),
        "success": raw.get("success"),
        "sourcesUnchanged": raw.get("sourcesUnchanged"),
        "sourceHashes": raw.get("sourceHashes"),
        "tools": raw.get("tools"),
        "checks": [{key: check[key] for key in ("name", "tool", "status") if key in check}
                   for check in raw.get("checks") or []],
    }


def summarize(label, checks):
    counts = {}
    for check in checks:
        counts[check.get("status")] = counts.get(check.get("status"), 0) + 1
    print(f"{label}: {counts.get('passed', 0)} passed, {counts.get('failed', 0)} failed, "
          f"{counts.get('skipped', 0)} skipped")
    for check in checks:
        if check.get("status") != "passed":
            print(f"  {check.get('status')}: {check.get('name')}")


def reduce_extension(raw):
    assurance = module("assurance", ROOT / "tools/verify-assurance.py")
    assemble = module("assemble_assurance", ROOT / "tools/assemble-assurance.py")
    local = module("local_assurance", ROOT / "tools/local-assurance.py")
    virtual = ROOT / "VirtualQuest"
    assemble.validate_extension(raw, assurance, assurance.source_identity(ROOT, virtual),
                                assurance.expected_checks(virtual, ROOT))
    return local.extension_metadata(raw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True)
    parser.add_argument("--raw", type=Path, help="the record the command writes")
    parser.add_argument("--record", choices=("proof", "extension"), default="proof",
                        help="a check.ps1 record, or a complete inventory extension record")
    parser.add_argument("--result", type=Path, help="where to keep the reduced record")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("no command to run")
    temp = Path(os.environ.get("RUNNER_TEMP") or tempfile.gettempdir())
    log = temp / ("private-" + "".join(c if c.isalnum() else "-" for c in args.label) + ".log")
    with log.open("wb") as output:
        code = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT).returncode
    if args.raw is None:
        print(f"{args.label}: " + ("passed" if code == 0 else f"failed (exit {code})"))
        return code

    try:
        raw = json.loads(args.raw.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        print(f"{args.label}: wrote no record (exit {code})")
        return code or 1
    if args.record == "extension":
        summarize(args.label, ((raw.get("suites") or {}).get("inventory-extension") or {}).get("checks") or [])
        if code != 0:
            return code
        try:
            record = reduce_extension(raw)
        except (KeyError, TypeError, ValueError) as error:
            # Messages name checks and files only, never their contents.
            print(f"{args.label}: record refused: {error}")
            return 1
        succeeded = True
    else:
        record = reduce_proof(raw)
        checks = record["checks"]
        summarize(args.label, checks)
        names = {check.get("name") for check in checks}
        succeeded = (code == 0 and bool(checks) and raw.get("complete") is True and raw.get("success") is True
                     and raw.get("sourcesUnchanged") is True and set(raw.get("expected") or []) <= names
                     and all(check.get("status") == "passed" for check in checks))
    if args.result:
        args.result.parent.mkdir(parents=True, exist_ok=True)
        args.result.write_text(json.dumps(record, indent=2) + "\n")
    return 0 if succeeded else (code or 1)


if __name__ == "__main__":
    sys.exit(main())
