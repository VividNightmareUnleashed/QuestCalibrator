# The twelve Now assurance obligations

The inventory identifies A01–A03, P01–P05, S04, S07, N05 and N06 as
the immediate work. `VirtualQuest/formal/obligations.json` records their methods,
checker names, implementation dependencies and external assumptions. The portable
contract runner reports a focused milestone; it never issues a full release
certificate by itself.

| Item | Implemented evidence and behavior | Scope |
| --- | --- | --- |
| A01 | Fail closed on empty selections, empty shards, missing strict dependencies, interrupted tools and unexpected mutant failures. Lean declarations are audited for proof holes and added axioms. Real fault fixtures exercise the acceptance policy. | Pinned tools, explicit permitted Lean axioms; expected theorem/assertion failures must be recognized. |
| A02 | Source hashes, exact commits, tool identities, named outcomes and source stability accompany results. Assembly requires every registered core, numeric, contract and emitted trace check. Both release paths require complete exact-pair evidence. | Local evidence is an assurance record, not an authenticated external attestation. Windows integration remains a separate release job. |
| A03 | An explicit dependency map, production headers, caller-order guards and intended production mutants connect checks to code. Models state their manual correspondence assumptions. | Hash equality detects changes; it does not establish a mathematical refinement proof. |
| P01 | Lean group-law proofs and independently composed Eigen reference poses check normalization, scale, base correction, field lookup, derivatives and time. The driver calls the tested `RuntimePose.h` kernel. | Exact invertible similarities in Lean; finite noncommuting C++ cases exercise floating-point implementation behavior. |
| P02 | Every finite eligibility row checks the hidden tracker exception and cache effects. Raw samples are published before the tested correction kernel; hiding follows it. | Trusted ring eligibility stays stricter than forwarding eligibility. Hardware pose classification is external. |
| P03 | Bounded frame/run transitions and actual ordered frame-correction chains exercise per-device normalization, rebinding, reset and recalibration. Duplicate move timestamps and nonfinite sample times are refused. | Five model steps, two devices, bounded translation epochs; C++ checks exercise three slots and rotation/translation composition. |
| P04 | The actual recovery and publication kernels are checked over every assignment of three slots and three nonzero identities, with session/profile/version refusal and parked-device isolation. Duplicate nonzero live or saved keys are refused before output changes. | The complete FNV64 identity/profile keys are assumed unique within a session. Formatted calibration time is not injective; matching keys do not independently prove physical or profile identity. |
| P05 | An RC11 model includes controls, base, per-device frame, field, fallback and shared-cache locking. Relaxed loads and an omitted slot lock produce intended witnesses. C++ contracts exercise the actual `RuntimeSnapshot.h` read and fallback. | One writer, serialized callbacks per slot, quiescent hooks before teardown, and no sequence wrap during a read. Logical payload tags abstract the scalar arrays. |
| S04 | Group algebra, bounded manual-run transitions and actual `ToStart`, `Carry`, `Express` and `End` kernels exercise frame preservation and exactly-once neutralization release. | The model abstracts solver arithmetic and OpenVR callbacks. |
| S07 | Bounded history freshness/reachability, actual short frame chains, inference expiry, tracking returns, setup signals and queue overflow are exercised. Late inference is refused; overflow aborts the current run and disables calibration. | Endpoint tolerance is not a transitive equivalence. Hardware logs, clocks and firmware classification remain external evidence. |
| N05 | Direct ESBMC/Gappa obligations cover numerical validation and normalization. A separate direct proof establishes the production copy helper preserves every scalar of every finite sanitized field, including unused anchors. C++ publication checks inspect every device slot, identity keys and refusal atomicity. | Each numeric harness records its unwind options. Composition uses the proven sanitizer result and the reviewed post-validation helper call. |
| N06 | Production settings/chaperone codecs are shared with contracts and fuzz targets. Profile and settings refusal preserve the destination. Direct ESBMC checks cover byte/depth/geometry/version guards. | Record size is capped at 16 MiB, nesting at 16, geometry at 16,384 quads. Registry/JSON allocation and OS failure behavior are outside the guard proof. |

N05 replaces the older manual whole-state field-copy bridge described in the
VirtualQuest numeric overview. `FieldCopy.cpp` proves the actual
`CopyAlignmentField` helper; the full-state validator calls it only after its
transform, field and frame checks succeed.

## Reproduce focused checks

Initialize the exact `VirtualQuest` submodule, prepare the tools described in its
formal README, and run from the QuestCalibrator root on Linux:

```bash
python tools/verify-now-contracts.py --source-root "$PWD" --output-dir /tmp/now
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Only lean -Strict -SourceRoot "$PWD" -ResultPath /tmp/lean.json
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Only calibration-run -Strict -SourceRoot "$PWD" -ResultPath /tmp/run.json
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Only frames -Strict -SourceRoot "$PWD" -ResultPath /tmp/frames.json
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Only seqlock -Strict -SourceRoot "$PWD" -ResultPath /tmp/snapshot.json
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Only RecordBounds -Strict -SourceRoot "$PWD" -ResultPath /tmp/records.json
```

The focused runner compiles the actual production kernels, runs all nine behavior
contracts, compiles nine separate production mutations, and requires each mutant
to fail its intended obligation. A compilation failure cannot kill a mutant. It
also runs Lean/runner fault injection and eleven stale/partial manifest fixtures.
The result lists the twelve items and the remaining assumptions. Optional
`--proof-results` attaches selected proof records without upgrading them into a
full-suite certificate.

## Complete exact-pair evidence

Commit both repositories and update the QuestCalibrator gitlink before collecting
release evidence. Leave sources unchanged throughout all checks. Run the full
private core, full numeric suite, focused contracts and real Windows hub trace
replay, each with `-Strict` and `-ResultPath` where supported. Build and run the
Windows solver suite separately. Numerical checks need substantial memory; run
one memory-heavy ESBMC process per machine, or use separate shard runners.

```bash
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Core -Strict -SourceRoot "$PWD" -ResultPath /tmp/core.json
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Only input-validation -Strict -SourceRoot "$PWD" -ResultPath /tmp/numeric.json -Parallel 1
python tools/verify-now-contracts.py --source-root "$PWD" --output-dir /tmp/now
```

On Windows, the trace command is:

```powershell
pwsh -NoProfile -File VirtualQuest/formal/check.ps1 -Only Traces -Strict -ResultPath traces.json
```

Transfer that trace record to the same exact source checkout and assemble:

```bash
python tools/verify-inventory-extension.py --source-root . --output-dir /tmp/extension
python -m pip install -r VirtualQuest/formal/binary-requirements.txt
python tools/verify-binary-correspondence.py --source-root . --output-dir /tmp/binary
python tools/assemble-assurance.py --contracts /tmp/now/result.json --extension /tmp/extension/result.json --binary /tmp/binary/result.json --core /tmp/core.json --numeric /tmp/numeric.json --traces traces.json --output /tmp/complete.json
python tools/verify-assurance.py --evidence /tmp/complete.json
```

`--core` and `--numeric` accept multiple shard records. Missing, duplicate, skipped,
stale or failed checks refuse assembly. Canonical source hashing permits CRLF/LF
checkout differences. The extension gate requires each compiling mutation's exact
intended assertion; arbitrary runtime exceptions do not count as a rejected mutant.
Capture mutations must also fail their intended named tests and assertion types.
The public Now runner requires all nine registered assertion messages and tests
67 acceptance fixtures. The extension gate tests another 222 acceptance fixtures.
V09 has a dedicated required suite: SHA-pinned instruction correspondence, ten
intended failure controls and 45 acceptance fixtures. The supplied SteamVR build
has a separate audited address profile; unknown builds remain refused.
The tool and configuration records preserve the actual
run details. `--linux-only` explicitly produces partial evidence that cannot pass
the release verifier. Generated evidence belongs in CI artifacts or a private
assurance record, not in the source tree.

For ordinary public CI, run the extension with `--public-only`. It reports a
distinct 23-obligation partial suite with 20 intended mutants and 144 acceptance
fixtures. V01–V08, private portable simulations, smoothing and capture checks
run in the release's formal assurance and V09 locally. That partial public suite
cannot replace the full extension record in a release certificate.

V09 runs on the supplied third-party binaries, so it stays off hosted runners.
Publish its record with
`python tools/local-assurance.py --binary-only --evidence /tmp/binary/result.json --publish`
before tagging a hosted release. The private Git note stores verification
metadata; it leaves both source commits unchanged.

The release workflow checks the exact tag and gitlink, runs the private core, the
complete numeric suite, the contracts and the complete extension with their output
kept off its public log, reads the V09 record, collects real Windows traces,
verifies the complete assembled record, and only then creates a draft. The local release entry point requires
`-FormalEvidence /path/to/complete.json` for the same gate. Publishing and hardware
acceptance remain the normal release process described in `releasing.md`.
