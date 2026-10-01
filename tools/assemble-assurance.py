#!/usr/bin/env python3
"""Assemble complete named proof suites; fail closed on stale or partial input."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path


def load(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quest", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--virtual", type=Path)
    parser.add_argument("--contracts", type=Path, required=True)
    parser.add_argument("--extension", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--core", type=Path, nargs="+", required=True)
    parser.add_argument("--numeric", type=Path, nargs="+", required=True)
    parser.add_argument("--traces", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--linux-only", action="store_true")
    args = parser.parse_args()
    virtual = args.virtual or args.quest / "VirtualQuest"
    spec = importlib.util.spec_from_file_location("assurance", args.quest / "tools/verify-assurance.py")
    assurance = importlib.util.module_from_spec(spec);spec.loader.exec_module(assurance)
    identity = assurance.source_identity(args.quest, virtual)
    record = load(args.contracts)
    if record.get("sources") != identity or not record.get("success") or not record.get("sourcesUnchanged"):
        raise ValueError("Contracts do not describe this immutable source pair")
    expected = assurance.expected_checks(virtual, args.quest)
    assurance.validate(record, identity, expected, required={"contracts"}, release=False,
                       require_negative_controls=True)
    extension = load(args.extension)
    assurance.validate(extension, identity, expected, required={"inventory-extension"}, release=False,
                       require_negative_controls=True)
    executions = extension.get("executions", {})
    if set(executions) != {"inventory", "portable", "smoother", "capture", "installer"} or any(
            not execution.get("success") or not execution.get("sourcesUnchanged") for execution in executions.values()):
        raise ValueError("Missing, failed or changed extension execution")
    assurance.validate_inventory_mutants(extension.get("executions", {}).get("inventory", {}).get("mutants"))
    if extension["executions"]["inventory"]["mutants"] != extension["suites"]["inventory-extension"]["negativeControls"]["mutants"]:
        raise ValueError("Inventory execution and accepted negative controls disagree")
    capture_controls = executions["capture"].get("assertionControls")
    assurance.validate_capture_mutants(capture_controls)
    if (capture_controls != extension["suites"]["inventory-extension"]["negativeControls"]["captureMutants"]
            or [{k:row[k] for k in ("name", "status", "method")} for row in capture_controls] != executions["capture"].get("mutants")):
        raise ValueError("Capture execution and accepted negative controls disagree")
    record["suites"]["inventory-extension"] = extension["suites"]["inventory-extension"]
    binary = load(args.binary)
    assurance.validate(binary, identity, expected, required={"binary-correspondence"}, release=False,
                       require_negative_controls=True)
    if binary.get("negativeControls") != binary["suites"]["binary-correspondence"]["negativeControls"]:
        raise ValueError("Binary execution and accepted negative controls disagree")
    record["suites"]["binary-correspondence"] = binary["suites"]["binary-correspondence"]
    for suite, paths in (("private-core", args.core), ("input-validation", args.numeric), ("hub-traces", [args.traces] if args.traces else [])):
        if not paths and not args.linux_only:
            raise ValueError("Hub trace evidence is required")
        if not paths:
            continue
        checks = []
        tools = {}
        for path in paths:
            result = load(path)
            if result.get("schemaVersion") != 2 or not result.get("sourcesUnchanged") or not result.get("complete") or not result.get("success"):
                raise ValueError(f"Incomplete or failed proof result: {path}")
            if not result.get("sourceHashes") or not result.get("tools"):
                raise ValueError(f"Missing tool/source provenance: {path}")
            for source, digest in result["sourceHashes"].items():
                repository, relative = source.split(":", 1)
                root = args.quest if repository == "QuestCalibrator" else virtual
                file = root / relative
                if not file.is_file() or hashlib.sha256(file.read_bytes().replace(b"\r\n", b"\n")).hexdigest() != digest:
                    raise ValueError(f"Stale checker input: {source}")
            for name, version in result["tools"].items():
                if name in tools and tools[name] != version:
                    raise ValueError(f"Inconsistent tool identity: {name}")
                tools[name] = version
            checks.extend(result["checks"])
        record["suites"][suite] = {"complete": True, "tools": tools, "checks": checks}
    record["fullSuite"] = not args.linux_only
    record["dirty"] = any(assurance.git(root,"status","--porcelain","--untracked-files=no") for root in (args.quest,virtual))
    required = assurance.REQUIRED_SUITES - ({"hub-traces"} if args.linux_only else set())
    assurance.validate(record, identity, expected, required=required, release=not args.linux_only)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record,indent=2)+"\n")
    print("Complete selected assurance suites match the source pair.")


if __name__ == "__main__":
    main()
