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

} // namespace questcal
