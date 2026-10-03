#pragma once

#include "ContinuousAlignment.h"

#include <optional>

namespace questcal
{

// Retains the newest continuous correction until confirmation. A trigger that
// was already held when the correction arrived is not confirmation: the user
// must release it and squeeze again.
class ContinuousCorrectionGate
{
public:
	using Correction = ContinuousAlignment::Correction;

	void Offer(const Correction &correction, bool triggerPressed)
	{
		if (!pending)
			releasedSinceOffer = !triggerPressed;
		else if (!triggerPressed)
			releasedSinceOffer = true;
		pending = correction;
	}

	bool Take(bool confirmationRequired, bool triggerPressed, Correction &out)
	{
		if (!pending)
			return false;
		if (confirmationRequired)
		{
			if (!triggerPressed)
			{
				releasedSinceOffer = true;
				return false;
			}
			if (!releasedSinceOffer)
				return false;
		}

		out = *pending;
		Clear();
		return true;
	}

	bool HasPending() const noexcept { return pending.has_value(); }

	void Clear() noexcept
	{
		pending.reset();
		releasedSinceOffer = false;
	}

private:
	std::optional<Correction> pending;
	bool releasedSinceOffer = false;
};

// What one tick did with the engine's output (see HandleContinuousOutput).
struct ContinuousOutputHandling
{
	enum class Outcome { None, Applied, Refused };
	Outcome reanchor = Outcome::None;
	Outcome correction = Outcome::None;
	// A correction now waits for a trigger pull, and was not waiting before.
	bool correctionReady = false;
};

// Takes the engine's output after ContinuousAlignment::Update, for the overlay
// and the simulations alike. A re-anchor (or its undoing) replaces a
// calibration that is already off by more than a freeze, so it is applied at
// once and snapped, past the gate. A correction goes through the gate: only
// the newest one counts, none while the engine says a correction may not
// apply, and with confirmation required only after a fresh trigger pull.
// `apply(delta, snap)` applies a delta and returns false when it was refused;
// `triggerPressed()` is asked only when a correction waits on confirmation.
template <typename Apply, typename TriggerPressed>
ContinuousOutputHandling HandleContinuousOutput(ContinuousAlignment &engine,
	ContinuousCorrectionGate &gate, bool confirmationRequired,
	Apply &&apply, TriggerPressed &&triggerPressed)
{
	using Outcome = ContinuousOutputHandling::Outcome;
	ContinuousOutputHandling handled;

	ContinuousAlignment::Correction reanchor;
	if (engine.PollReanchor(reanchor))
	{
		gate.Clear();
		handled.reanchor = apply(reanchor, true) ? Outcome::Applied : Outcome::Refused;
	}

	ContinuousAlignment::Correction correction;
	bool hadPending = gate.HasPending();
	bool received = false;
	while (engine.PollCorrection(correction))
		received = true;
	if (!engine.CorrectionEligible())
	{
		gate.Clear();
		hadPending = false;
		received = false;
	}

	const bool pressed = confirmationRequired && (hadPending || received) && triggerPressed();
	if (received)
		gate.Offer(correction, pressed);
	if (gate.HasPending())
	{
		if (gate.Take(confirmationRequired, pressed, correction))
			handled.correction = apply(correction, false) ? Outcome::Applied : Outcome::Refused;
		else
			handled.correctionReady = !hadPending;
	}
	return handled;
}

} // namespace questcal
