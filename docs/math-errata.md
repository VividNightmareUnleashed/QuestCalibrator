# math.pdf errata and addenda

`math.pdf` is upstream's derivation of the two-stage solve, kept verbatim for
reference. Its *algorithmic recipe* (axis-pair Kabsch for the rotation, the
linear system (8) for the translation) is what this repo implements and is
correct — but two steps of the derivation as written are wrong, and a few
properties the implementation depends on are never stated. Anyone extending
the solver from the pdf should read this first.

## Errata

### Eqs. (1)/(2): the tracker offset P composes on the wrong side

The doc writes

> ∀i : p⃗ᴀᵢ = P F p⃗ʙᵢ  (1)
> ∀i : Qᴀᵢ = Pᵣ Fᵣ Qʙᵢ  (2)

with P "the constant transform from B's position and orientation to A's",
composed on the **left** (world side). A rigid tracker-to-tracker offset is
constant only in the tracker **body** frame, so it must compose on the
**right**:

    poseᴀ(t) = F ∘ poseʙ(t) ∘ P
    Qᴀᵢ = Fᵣ Qʙᵢ Pᵣ
    p⃗ᴀᵢ = Fᵣ p⃗ʙᵢ + F⃗ₚ + (Fᵣ Qʙᵢ) p⃗_P

Under the doc's own left-composed model no constant P exists once the rig
rotates, and eqs. (3)/(4) — which the rest of the derivation builds on — are
false as derived. Under the corrected right-composed model they hold, and the
constant in (3) is provably the body-frame mount offset (see "P is
recoverable" below).

### Eq. (6): the relation is a conjugation, not a one-sided product

The doc writes

> ∀i,j : Qᴀᵢ Qᴀⱼᵀ = Fᵣ Qʙᵢ Qʙⱼᵀ  (6)

As a matrix equation this is wrong (and does not follow from (2) in either
composition). The correct relation, from Qᴀᵢ = Fᵣ Qʙᵢ Pᵣ (the constant Pᵣ
cancels), is the **similarity**

    Qᴀᵢ Qᴀⱼᵀ = Fᵣ (Qʙᵢ Qʙⱼᵀ) Fᵣᵀ

The axis-Kabsch recipe works precisely because conjugation maps rotation
**axes** by Fᵣ while preserving rotation **angles**:

- axis(Δᴀ) = Fᵣ · axis(Δʙ) — the property the Kabsch pairing solves for;
- angle(Δᴀ) = angle(Δʙ) — free consistency information; the implementation
  uses it to reject jittered pairs (the angle-mismatch gate in
  `Overlay/CalibrationEngine.cpp`).

Implementing (6) literally — e.g. as a linear least-squares over full rotation
matrices, a natural "upgrade" from axis-only — solves the wrong equation and
biases Fᵣ whenever it has significant yaw.

Two practical corollaries the doc omits: the axis of a rotation has a sign
ambiguity tied to the angle's sign (the implementation canonicalizes to the
w ≥ 0 hemisphere on both sides, and rejects the ambiguous near-180° band where
noise decorrelates that choice), and equality of angles is what makes the
axis pairing well-posed.

## Addenda (what the doc's model leaves out)

- **Observability.** Fᵣ needs relative rotations about ≥ 2 independent axes
  (with one axis, Fᵣ is determined only up to rotation about it). The
  translation system (8) constrains F⃗ₚ only perpendicular to each pair's
  relative-rotation axis, so yaw-dominant motion leaves the vertical component
  resting on noise even when the rotation solve is acceptable. The
  implementation gates both (`minAxisSpread`, `minTransEigRatio`).
- **Latency.** The two streams are not simultaneous: p⃗ʙ is observed at
  t − τ. Unmodeled, τ biases both solves during motion. The implementation
  estimates τ by cross-correlating angular-speed profiles before pairing.
- **Scale.** The doc assumes metric-identical spaces; real runtimes disagree
  by a small scale s (p⃗ʙ → s·p⃗ʙ throughout). The implementation searches s
  on the translation residual.
- **P is recoverable, and useful.** The doc abandons P as unsolvable
  ambiguity, but the constant of eq. (3) under the corrected model **is**
  Pᵣᵀ-conjugated mount data: d = Qᴀᵀ(p⃗ᴀ − Fᵣ p⃗ʙ − F⃗ₚ) is the body-frame
  mount offset. Its per-pair scatter is a rigidity statistic, and the mount
  extrinsic that continuous calibration depends on
  (`ContinuousAlignment::DeriveMountExtrinsic`) is exactly this quantity.
  The joint refinement step (`EngineConfig::refineIterations`) also estimates
  it as a nuisance parameter.
