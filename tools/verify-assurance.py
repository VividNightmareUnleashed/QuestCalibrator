#!/usr/bin/env python3
"""Validate complete, exact-pair formal evidence before a release.

Evidence is a local assurance record, not an authenticated third-party attestation.
The release workflow still separately gates the Windows build and integration tests.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import re
import subprocess


REQUIRED_SUITES = {"private-core", "input-validation", "contracts", "hub-traces", "inventory-extension", "binary-correspondence"}
BINARY_CONTROLS = {
    "wrong-model-reset": "Quest reset mismatch",
    "wrong-model-position": "Quest position mismatch",
    "wrong-model-rotation": "Quest rotation mismatch",
    "model-used-as-binary-oracle": "No original Quest routine executed",
    "changed-binary-libtrackingengines.so": "This verifier only supports the recovered engine binary",
    "changed-binary-VirtualDesktop.LibOVRRT64_1.dll": "Unsupported DLL: expected the documented SHA-256",
    "changed-binary-driver_oculus.dll": "Unsupported DLL: expected the documented SHA-256",
    "wrong-steamvr-clock-address": "Wrong poseTimeOffset",
    "wrong-steamvr-validity-mask": "Wrong tracking result",
    "wrong-steamvr-yaw-mask": "Wrong validity/connection/yaw flags",
}


def validate_binary_controls(controls):
    if not isinstance(controls, list) or len(controls) != len(BINARY_CONTROLS):
        raise ValueError("Missing binary correspondence negative controls")
    names = [row.get("name") for row in controls if isinstance(row, dict)]
    if len(names) != len(set(names)) or set(names) != set(BINARY_CONTROLS):
        raise ValueError("Missing, duplicate or unexpected binary negative control")
    if any(row.get("status") != "passed" or row.get("intendedFailure") != BINARY_CONTROLS[row["name"]]
           for row in controls):
        raise ValueError("Unintended binary failure accepted as mutation evidence")


def binary_negative_control_fixtures():
    valid = [{"name": name, "status": "passed", "intendedFailure": message}
             for name, message in BINARY_CONTROLS.items()]
    validate_binary_controls(valid)
    fixtures = [[], None, valid + [valid[0]], valid + [{"name": "unknown"}]]
    for index in range(len(valid)):
        fixtures.append(valid[:index] + valid[index+1:])
        for key, value in (("name", "unknown"), ("status", "skipped"), ("intendedFailure", "unrelated error")):
            altered = copy.deepcopy(valid)
            altered[index][key] = value
            fixtures.append(altered)
    for invalid in fixtures:
        try:
            validate_binary_controls(invalid)
        except ValueError:
            pass
        else:
            raise ValueError("Invalid binary negative-control fixture accepted")
    return len(fixtures) + 1
NOW_MUTANTS = {
    "P01": ("Driver/RuntimePose.h", "P01: normalization/base/field composition order"),
    "P02": ("Driver/RuntimePose.h", "P02: independent valid/connection/result/bounds table"),
    "P03": ("Overlay/TrackerFrameCorrections.h", "P03: duplicate timestamp refused atomically"),
    "P04": ("Overlay/FrameRecovery.h", "P04: duplicate identity keys have no recovery interpretation"),
    "P05": ("Driver/RuntimeSnapshot.h", "P05: retry fallback is one previously coherent payload"),
    "S04": ("Overlay/CalibrationRun.h", "S04: CarryCalibration composed in correct space"),
    "S07": ("Overlay/LighthouseFrameWatch.h", "S07: late corroboration cannot revive expired inference"),
    "N05": ("Driver/ProtocolValidation.h", "N05: frame/identity/all anchors copied"),
    "N06": ("Overlay/SettingsRecordJson.h", "N06: Settings boundaries/atomicity/round-trip"),
}

# Each compiling mutation must reach its intended contract assertion. An exit
# code and arbitrary nonempty exception text do not establish that obligation.
INVENTORY_MUTANTS = {
    "P06": ("Driver/HookLifecyclePolicy.h", "Quiescence decision"),
    "P07": ("common/PoseRingCounters.h", "Modular sequence ordering"),
    "P08": ("Driver/AlignmentField.cpp", "Actual angular cap, including large arcs"),
    "S01": ("Overlay/DriverSyncTracker.h", "Only a submitted current-state verdict is accepted"),
    "S02": ("common/IPCFramePolicy.h", "Incomplete/error pipe request is never dispatchable"),
    "S03": ("Overlay/RingSampleGate.h", "Invalid clock period cannot fabricate an accepted time"),
    "S05": ("Overlay/ChaperoneRestorePolicy.h", "Protected geometry requires matching current owner"),
    "S06": ("Overlay/LocalPoseContinuity.h", "Duplicate time never supplies adjacent-frame evidence"),
    "S08": ("Overlay/ContinuousAlignment.cpp", "Insufficient fresh window interrupts freeze evidence"),
    "S09": ("Overlay/PersistenceState.h", "Persisted revision never absent"),
    "N01": ("Overlay/CalibrationEngine.cpp", "Rich noiseless hand-eye trajectory is accepted"),
    "N02": ("Overlay/SolverResourcePolicy.h", "Thin indices in range and strictly ordered"),
    "N03": ("Overlay/CalibrationEngine.cpp", "Latency is negated once then asymmetrically clamped"),
    "N04": ("Overlay/RobustPolicy.h", "Huber IRLS definition and clipping identity"),
    "N07": ("Overlay/UpdatePolicy.h", "Version order agrees with independent tuple specification"),
    "N08": ("Overlay/ChaperoneMath.h", "Standing center is left-composed once"),
    "V01": ("VirtualQuest/Tests/VirtualQuestValidation.h", "Invalid controller indexing refused"),
    "V02": ("VirtualQuest/Tests/VirtualQuest.cpp", "Angular derivative carries the roll axis through pitch"),
    "V03": ("VirtualQuest/Tests/VirtualQuest.cpp", "Hold preserves state exactly"),
    "V04": ("VirtualQuest/Tests/HealthPolicy.h", "Fresh hold-off episode after reset"),
    "V05": ("VirtualQuest/Tests/VirtualQuest.cpp", "Other controller health is isolated"),
    "V06": ("VirtualQuest/Tests/CacheStamp.h", "First delivery after a gap cannot reuse an old session cache"),
    "O01": ("Overlay/UpdaterRevisionPolicy.h", "Cancel or stale revision cannot launch a helper"),
    "O03": ("Overlay/CalibrationActionPolicy.h", "Stale start cannot replace an owned measurement"),
    "O04": ("Overlay/TexturePolicy.h", "Texture dimensions agree with atlas/icon policy"),
    "O05": ("Overlay/TrackingStreamDigest.h", "Duplicate capture is explicitly excluded from prediction statistics"),
}
CAPTURE_MUTANTS = {
    "V07-clock": {"test_clock_interval_includes_endpoint_uncertainty": "Tuples differ: (7, 13) != (7, 25)"},
    "V07-provenance": {"test_quantitative_context_requires_complete_observed_bracketing": "True != False"},
    "V08-frame-order": {"test_reference_jump_is_removed_while_physical_motion_remains": None,
                        "test_seeded_frame_removal_and_half_turns": None},
    "V08-bracketing": {"test_quantitative_context_requires_complete_observed_bracketing": "True != False"},
}
PUBLIC_INVENTORY_MUTANTS = {name: contract for name, contract in INVENTORY_MUTANTS.items()
                            if not name.startswith('V')}


def validate_cpp_mutants(mutants, registry):
    if not isinstance(mutants, list) or any(not isinstance(row, dict) for row in mutants):
        raise ValueError("C++ negative controls are absent or malformed")
    names = [row.get("name") for row in mutants]
    if any(not isinstance(name, str) for name in names) or len(names) != len(set(names)) or set(names) != set(registry):
        raise ValueError("C++ negative controls are missing, duplicate or unexpected")
    for row in mutants:
        path, message = registry[row["name"]]
        if (row.get("status") != "passed" or row.get("method") != "production mutant"
                or row.get("mutation") != path or row.get("failure") != message):
            raise ValueError(f'{row["name"]}: mutant did not reach its intended assertion')
    return True


def validate_inventory_mutants(mutants):
    return validate_cpp_mutants(mutants, INVENTORY_MUTANTS)


def validate_now_mutants(mutants):
    return validate_cpp_mutants(mutants, NOW_MUTANTS)


def cpp_negative_control_fixtures(registry):
    valid = [{"name": name, "status": "passed", "method": "production mutant",
              "mutation": path, "failure": message}
             for name, (path, message) in registry.items()]
    validate_cpp_mutants(valid, registry)
    fixtures = [[], valid[:-1], valid + [valid[0]]]
    for index in range(len(valid)):
        for key, value in (("failure", ""), ("failure", "std::bad_alloc"),
                           ("failure", "unrelated contract assertion"), ("name", "unknown"),
                           ("status", "skipped"), ("method", "compiler error"),
                           ("mutation", "unrelated.cpp")):
            invalid = copy.deepcopy(valid)
            invalid[index][key] = value
            fixtures.append(invalid)
    for invalid in fixtures:
        try:
            validate_cpp_mutants(invalid, registry)
        except ValueError:
            pass
        else:
            raise ValueError("Unrelated mutant failure accepted as intended evidence")
    return len(fixtures) + 1


def inventory_negative_control_fixtures():
    return cpp_negative_control_fixtures(INVENTORY_MUTANTS)


def now_negative_control_fixtures():
    return cpp_negative_control_fixtures(NOW_MUTANTS)


def validate_capture_mutants(mutants):
    if not isinstance(mutants, list) or any(not isinstance(row, dict) for row in mutants):
        raise ValueError("Capture negative controls are absent or malformed")
    names = [row.get("name") for row in mutants]
    if any(not isinstance(name, str) for name in names) or len(names) != len(set(names)) or set(names) != set(CAPTURE_MUTANTS):
        raise ValueError("Capture negative controls are missing, duplicate or unexpected")
    for row in mutants:
        log = row.get("log")
        if row.get("status") != "passed" or row.get("method") != "compiling Python mutant" or not isinstance(log, str):
            raise ValueError("Capture mutant did not compile and reach an assertion")
        failures = list(re.finditer(r"^FAIL: (test_\w+) \(test_quest_pose_debug\.PoseAnalysisTests\.\1\)$", log, re.M))
        expected = CAPTURE_MUTANTS[row["name"]]
        if (re.search(r"^ERROR:", log, re.M) or len(failures) != len(expected)
                or set(m.group(1) for m in failures) != set(expected)
                or len(re.findall(r"^FAIL:", log, re.M)) != len(expected)
                or re.findall(r"^Ran (\d+) tests in .+$", log, re.M) != ["19"]
                or re.findall(r"^FAILED \(failures=(\d+)\)$", log, re.M) != [str(len(expected))]):
            raise ValueError(f'{row["name"]}: capture mutant did not reach its intended tests')
        for index, failure in enumerate(failures):
            end = failures[index + 1].start() if index + 1 < len(failures) else len(log)
            assertions = re.findall(r"^AssertionError: (.+)$", log[failure.end():end], re.M)
            message = expected[failure.group(1)]
            if len(assertions) != 1 or (message is not None and assertions[0] != message):
                raise ValueError(f'{row["name"]}: unexpected capture assertion')
            if message is None:
                # This independent geometry oracle rejects a distance outside
                # the declared tolerance; values need not be bit-identical
                # between supported Python/libm implementations.
                match = re.fullmatch(r"([0-9.eE+-]+) not less than 1e-06", assertions[0])
                if not match or not 1e-6 <= float(match[1]) < float("inf"):
                    raise ValueError(f'{row["name"]}: unexpected geometry assertion')
    return True


def capture_negative_control_fixtures(valid):
    validate_capture_mutants(valid)
    fixtures = [[], valid[:-1], valid + [valid[0]]]
    for index in range(len(valid)):
        log = valid[index]["log"]
        for key, value in (("status", "skipped"), ("method", "compiler error"),
                           ("log", "std::bad_alloc"), ("log", log + "\nERROR: unrelated exception\n"),
                           ("log", log.replace("AssertionError:", "RuntimeError:")),
                           ("log", log.replace("test_", "unrelated_")),
                           ("log", log.replace("Ran 19 tests", "Ran 0 tests")),
                           ("log", log.replace("FAILED (failures=", "INCOMPLETE (failures="))):
            invalid = copy.deepcopy(valid)
            invalid[index][key] = value
            fixtures.append(invalid)
    for invalid in fixtures:
        try:
            validate_capture_mutants(invalid)
        except ValueError:
            pass
        else:
            raise ValueError("Unintended capture failure accepted as mutation evidence")
    return len(fixtures) + 1


def git(repo, *arguments):
    return subprocess.check_output(["git", "-C", str(repo), *arguments], text=True).strip()


def source_identity(quest, virtual):
    identities = {}
    for name, root in (("QuestCalibrator", quest), ("VirtualQuest", virtual)):
        paths = subprocess.check_output(
            ["git", "-C", str(root), "ls-files", "-z", "--cached", "--others", "--exclude-standard"]
        ).decode().split("\0")
        # Every file Git stores as text is hashed with LF line endings, as a
        # Linux checkout has it: a Windows checkout of the same commit can hold
        # CRLF copies of any of them, whatever their extension.
        text = set()
        for entry in subprocess.check_output(["git", "-C", str(root), "ls-files", "-z", "--eol"]).decode().split("\0"):
            info, _, path = entry.partition("\t")
            if info.split()[:1] and info.split()[0] in ("i/lf", "i/crlf", "i/mixed"):
                text.add(path)
        hashes = {}
        for path in sorted(set(paths)):
            file = root / path
            if not file.is_file():
                continue
            data = file.read_bytes()
            if path in text or file.suffix.lower() in {".h", ".cpp", ".c", ".lean", ".tla", ".cfg", ".ps1", ".py", ".yml", ".json", ".inc", ".g", ".vcxproj", ".targets", ".md", ".txt"} or file.name == "Dockerfile":
                data = data.replace(b"\r\n", b"\n")
            hashes[path] = hashlib.sha256(data).hexdigest()
        identities[name] = {"commit": git(root, "rev-parse", "HEAD"), "files": hashes}
    return identities


def validate(record, identity, expected, required=REQUIRED_SUITES, release=True, require_negative_controls=False):
    if record.get("schemaVersion") != 2:
        raise ValueError("Unsupported assurance record")
    if record.get("sources") != identity or not record.get("sourcesUnchanged"):
        raise ValueError("Assurance sources differ from this exact source pair")
    if not record.get("success"):
        raise ValueError("Assurance run did not succeed")
    if release and (not record.get("fullSuite") or record.get("dirty")):
        raise ValueError("A partial or uncommitted-source run cannot satisfy a release")
    suites = record.get("suites", {})
    for suite in required:
        entry = suites.get(suite, {})
        checks = entry.get("checks", [])
        names = [check.get("name") for check in checks]
        if not entry.get("complete") or not entry.get("tools"):
            raise ValueError(f"{suite}: missing completion or tool versions")
        if len(names) != len(set(names)) or set(names) != set(expected[suite]):
            raise ValueError(f"{suite}: missing, duplicate or unexpected checks")
        if any(check.get("status") != "passed" for check in checks):
            raise ValueError(f"{suite}: failed or skipped checks")
        if suite == "contracts" and "P01" in expected[suite]:
            controls = entry.get("negativeControls")
            if release or require_negative_controls or controls is not None:
                if not isinstance(controls, dict) or controls.get("acceptanceFixtures") != 67:
                    raise ValueError("Missing Now negative-control acceptance evidence")
                validate_now_mutants(controls.get("mutants"))
                for check in checks:
                    if check["name"] in NOW_MUTANTS and (check.get("negative") is not True or type(check.get("cases")) is not int or check["cases"] <= 0):
                        raise ValueError("Now contract lacks a nonempty positive or intended negative witness")
        if suite == "inventory-extension" and "P06" in expected[suite]:
            # Low-level conformance runners may validate partial records before
            # the public wrapper binds their controls. Assembly and release
            # always require the complete intended-failure evidence.
            controls = entry.get("negativeControls")
            if release or require_negative_controls or controls is not None:
                if not isinstance(controls, dict) or controls.get("acceptanceFixtures") != 186:
                    raise ValueError("Missing inventory negative-control acceptance evidence")
                validate_inventory_mutants(controls.get("mutants"))
                if controls.get("captureAcceptanceFixtures") != 36:
                    raise ValueError("Missing capture negative-control acceptance evidence")
                validate_capture_mutants(controls.get("captureMutants"))
                if any(type(check.get("cases")) is not int or check["cases"] <= 0 for check in checks):
                    raise ValueError("Inventory extension has an empty positive witness")
        if suite == "public-inventory-extension":
            if (len(expected[suite]) != 23 or any(name.startswith('V') for name in expected[suite])):
                raise ValueError("Public inventory scope contains private or missing obligations")
            controls = entry.get("negativeControls")
            if require_negative_controls or controls is not None:
                if not isinstance(controls, dict) or controls.get("acceptanceFixtures") != 144:
                    raise ValueError("Missing public inventory negative-control acceptance evidence")
                validate_cpp_mutants(controls.get("mutants"), PUBLIC_INVENTORY_MUTANTS)
                if controls.get("privateSelectionsRefused") != 6:
                    raise ValueError("Private inventory selections were not refused")
                if any(type(check.get("cases")) is not int or check["cases"] <= 0 for check in checks):
                    raise ValueError("Public inventory extension has an empty positive witness")
        if suite == "binary-correspondence" and "V09" in expected[suite]:
            validate_binary_controls(entry.get("negativeControls"))
            if any(type(check.get("cases")) is not int or check["cases"] != 4577 for check in checks):
                raise ValueError("Binary correspondence has an incomplete positive witness")
            if release or require_negative_controls:
                if entry.get("acceptanceFixtures") != 45:
                    raise ValueError("Missing binary negative-control acceptance evidence")
    return True


def expected_checks(virtual, source, pwsh="pwsh"):
    expected = {"contracts": ["A01", "A02", "A03", "P01", "P02", "P03", "P04", "P05", "S04", "S07", "N05", "N06"]}
    registry = json.loads((virtual / "formal/inventory-scope.json").read_text())
    extension = set(registry["items"]) - set(expected["contracts"]) - {"V09"}
    if len(registry["items"]) != 44 or len(extension) != 31:
        raise ValueError("Inventory scope changed without updating the release policy")
    expected["inventory-extension"] = sorted(extension)
    expected["binary-correspondence"] = ["V09"]
    for suite, selection in (("private-core", ["-Core"]), ("input-validation", ["-Only", "input-validation"])):
        result = subprocess.check_output([pwsh, "-NoProfile", "-File", str(virtual / "formal/check.ps1"), "-ListChecks", "-SourceRoot", str(source), *selection], text=True)
        checks = json.loads(result)
        if isinstance(checks, dict):
            checks = [checks]
        expected[suite] = [check["Name"] for check in checks]
    # These names come from the committed implementation trace generator.
    # A trace record stores its exact emitted names; enforce positives plus all
    # ten registered negative traces, rather than accepting an empty artifact.
    expected["hub-traces"] = trace_names(virtual)
    return expected


def trace_names(virtual):
    import re
    text = (virtual / "Tests/FormalConformanceTests.cpp").read_text(encoding="utf-8-sig")
    match = re.search(r"const int perConsumer = (\d+);", text)
    if not match or '"hub-%s-%02d.tla"' not in text:
        raise ValueError("Trace generator changed; update its correspondence contract")
    positives = [f"Trace-{consumer}-{index:02d}" for consumer in ("collector", "monitor") for index in range(int(match[1]))]
    mutants = ["Trace-" + name for name in ("HoleNotClosed", "OverflowUncounted", "BoundaryNotInHole", "GapAfterPrefix", "BacklogKeptAtBoundary", "MonitorBeforeFix", "BoundaryReadBeforeDrain", "PerDrainLeash", "CollectorPerPass", "NoRunBoundaryCheck")]
    return positives + mutants


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quest", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--virtual", type=Path)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--pwsh", default="pwsh")
    args = parser.parse_args()
    virtual = args.virtual or args.quest / "VirtualQuest"
    expected = expected_checks(virtual, args.quest, args.pwsh)
    record = json.loads(args.evidence.read_text(encoding="utf-8-sig"))
    validate(record, source_identity(args.quest, virtual), expected)
    if git(args.quest, "rev-parse", "HEAD:VirtualQuest") != git(virtual, "rev-parse", "HEAD"):
        raise ValueError("QuestCalibrator submodule pin does not match the verified VirtualQuest commit")
    print("Complete formal evidence matches both exact release commits.")


if __name__ == "__main__":
    main()
