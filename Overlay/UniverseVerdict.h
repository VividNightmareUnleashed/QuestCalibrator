#pragma once

#include "ChaperoneMath.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>

// The profile-universe verdict, kept free of OpenVR and CalibrationContext so
// the harness can drive it: whether the headset's worldFromDriver (WFD) still
// matches the one the profile was calibrated in, and if not, whether the
// change was followed (adopt the new WFD) or the profile has to be latched
// unsafe. CalibrationSpace feeds it the HMD's WFD transitions and the jump
// detector's compensations, and applies what it decides.
//
// A transition out of the profile's WFD is followed when
//   - the composed pose stayed on its trajectory across it: the driver
//     re-expressed the local pose and the world did not move, or
//   - a compensation was applied for a step at that very sample. The exact
//     path also moves the profile's WFD itself, but only from the WFD its
//     rebase started at (ExactDeltaRebasesProfile); when the profile is still
//     behind, the chain follows the rebase once the earlier change is.
// Transitions are followed in order, so each adoption covers exactly the
// changes that were explained. While the jump detector still holds a headset
// candidate raised at the next unexplained transition (a map switch waiting
// up to 30 s for a controller), the verdict waits for it. Otherwise the
// profile is latched after GraceSeconds, counted from the moment the
// profile's current WFD first mismatched: a compensation that moves the
// profile's WFD starts the count again.
namespace questcal
{

struct WorldFromDriver
{
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translation{ 0, 0, 0 };
};

inline bool SameWorldFromDriver(const WorldFromDriver &a, const WorldFromDriver &b)
{
	return !WorldFromDriverChanged(a.rotation, a.translation, b.rotation, b.translation);
}

class UniverseVerdict
{
public:
	static constexpr double GraceSeconds = 0.5;

	struct Decision
	{
		bool adopt = false;
		WorldFromDriver adopted;
		bool latch = false;
	};

	// A universe delta was applied, exact or heuristic; `time` is the HMD's
	// jump sample.
	void NoteCompensation(double time)
	{
		compensated.push_back(time);
		if (compensated.size() > MaxRemembered)
			compensated.pop_front();
	}

	// Space saw the HMD's WFD change between two adjacent samples, the second
	// at `time`.
	void NoteTransition(double time, const WorldFromDriver &from,
		const WorldFromDriver &to, bool composedContinuous)
	{
		transitions.push_back({ time, from, to, composedContinuous });
		if (transitions.size() > MaxRemembered)
			transitions.pop_front();
	}

	// Whether an exact delta from `previous` may move the profile's WFD to its
	// endpoint. From any other WFD the endpoint would also cover an earlier
	// change that nothing compensated.
	static bool ExactDeltaRebasesProfile(const WorldFromDriver &profile,
		const WorldFromDriver &previous)
	{
		return SameWorldFromDriver(profile, previous);
	}

	// A mismatch is being timed, or waits for a held headset candidate.
	bool Pending() const noexcept { return mismatchSince >= 0.0 || waiting; }
	void Clear()
	{
		mismatchSince = NoTime;
		waiting = false;
		transitions.clear();
		compensated.clear();
	}

	// One ProfileUniverseTick past its gates: `observed` is the HMD's newest
	// WFD; `candidateLiveAt(t)` says whether the jump detector still holds a
	// headset candidate raised at sample time t.
	Decision Evaluate(double now, const WorldFromDriver &profile,
		const WorldFromDriver &observed,
		const std::function<bool(double)> &candidateLiveAt)
	{
		Decision decision;
		if (SameWorldFromDriver(profile, observed))
		{
			Clear();
			return decision;
		}

		WorldFromDriver reference = profile;
		waiting = false;
		for (auto it = transitions.begin(); it != transitions.end();)
		{
			if (!SameWorldFromDriver(it->from, reference))
			{
				++it;
				continue;
			}
			if (it->composedContinuous || Compensated(it->time))
			{
				reference = it->to;
				decision.adopt = true;
				decision.adopted = reference;
				it = transitions.erase(it);
				continue;
			}
			waiting = candidateLiveAt(it->time);
			break;
		}

		if (SameWorldFromDriver(reference, observed))
		{
			Clear();
			return decision;
		}
		if (waiting)
		{
			mismatchSince = NoTime;
			return decision;
		}
		if (mismatchSince < 0.0 || !SameWorldFromDriver(timedFrom, reference))
		{
			mismatchSince = now;
			timedFrom = reference;
		}
		if (now - mismatchSince < GraceSeconds)
			return decision;
		Clear();
		decision.latch = true;
		return decision;
	}

private:
	struct Transition
	{
		double time;
		WorldFromDriver from;
		WorldFromDriver to;
		bool composedContinuous;
	};

	static constexpr double NoTime = -1e9;
	static constexpr std::size_t MaxRemembered = 32;

	bool Compensated(double time) const
	{
		for (double t : compensated)
			if (std::abs(t - time) <= 1e-9)
				return true;
		return false;
	}

	double mismatchSince = NoTime;
	WorldFromDriver timedFrom;
	bool waiting = false;
	std::deque<Transition> transitions;
	std::deque<double> compensated;
};

} // namespace questcal
