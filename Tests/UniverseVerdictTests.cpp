#include "../common/Protocol.h"
#include "../Overlay/UniverseVerdict.h"

#include <cstdio>
#include <functional>

// The profile-universe verdict on its own: the HMD's worldFromDriver (WFD)
// transitions and the jump detector's compensations go in, adoptions and the
// latch come out, on the 50 Hz tick CalibrationTick runs it at.
namespace
{
using Check = void (*)(const char *, bool, const char *);
using questcal::UniverseVerdict;
using questcal::WorldFromDriver;

WorldFromDriver Wfd(double x)
{
	WorldFromDriver w;
	w.translation.x() = x;
	return w;
}

struct Rig
{
	UniverseVerdict verdict;
	WorldFromDriver profile = Wfd(0);
	WorldFromDriver observed = Wfd(0);
	double latchedAt = -1.0;
	int adoptions = 0;
	std::function<bool(double)> live = [](double) { return false; };

	void Transition(double time, double x, bool composedContinuous)
	{
		verdict.NoteTransition(time, observed, Wfd(x), composedContinuous);
		observed = Wfd(x);
	}

	void Tick(double now)
	{
		if (latchedAt >= 0.0)
			return;
		auto decision = verdict.Evaluate(now, profile, observed, live);
		if (decision.adopt)
		{
			profile = decision.adopted;
			++adoptions;
		}
		if (decision.latch)
			latchedAt = now;
	}

	void Run(double from, double to)
	{
		for (double now = from; now <= to + 1e-9; now += 0.02)
			Tick(now);
	}
};

void Detail(char (&buf)[160], const Rig &rig)
{
	std::snprintf(buf, sizeof buf, "latched at %.2f s, profile WFD x %.2f, %d adoption(s)",
		rig.latchedAt, rig.profile.translation.x(), rig.adoptions);
}
} // namespace

void RunUniverseVerdictScenarios(Check check)
{
	char buf[160];
	{
		Rig rig;
		rig.Transition(1.0, 1.0, false);
		rig.Run(1.0, 2.0);
		Detail(buf, rig);
		check("universe verdict: an unexplained rebase latches after the grace period",
			rig.latchedAt >= 1.5 && rig.latchedAt < 1.54 && rig.adoptions == 0, buf);
	}
	{
		// The first rebase is seen at 1.0 s; the UI then stalls 0.6 s, in which
		// the detector compensates it (the profile takes its endpoint) and a
		// second rebase arrives. The grace period belongs to the second.
		Rig rig;
		rig.Transition(1.0, 1.0, false);
		rig.Tick(1.0);
		rig.profile = Wfd(1.0);
		rig.Transition(1.6, 2.0, false);
		rig.Tick(1.6);
		const bool heldAtStall = rig.latchedAt < 0.0;
		rig.Run(1.62, 2.4);
		Detail(buf, rig);
		check("universe verdict: a compensated rebase restarts the grace period",
			heldAtStall && rig.latchedAt >= 2.1 && rig.latchedAt < 2.14, buf);
	}
	{
		// The driver re-expressed the local pose: the world did not move.
		Rig rig;
		rig.Transition(1.0, 1.0, true);
		rig.Run(1.0, 3.0);
		Detail(buf, rig);
		check("universe verdict: a bookkeeping worldFromDriver change is adopted",
			rig.latchedAt < 0.0 && rig.adoptions == 1 && rig.profile.translation.x() == 1.0, buf);
	}
	{
		// A WFD change the heuristic path sees, held 12 s for a controller.
		Rig rig;
		bool confirmed = false;
		rig.live = [&](double t) { return !confirmed && t == 1.0; };
		rig.Transition(1.0, 1.0, false);
		rig.Run(1.0, 13.0);
		confirmed = true;
		rig.verdict.NoteCompensation(1.0);
		rig.Run(13.02, 14.0);
		Detail(buf, rig);
		check("universe verdict: a held headset step is not latched before its confirmation",
			rig.latchedAt < 0.0 && rig.adoptions == 1 && rig.profile.translation.x() == 1.0, buf);
	}
	{
		Rig rig;
		bool confirmed = false;
		rig.live = [&](double t) { return !confirmed && t == 1.0; };
		rig.Transition(1.0, 1.0, false);
		rig.Run(1.0, 3.0);
		confirmed = true;   // the hold expired unconfirmed
		rig.Run(3.02, 4.0);
		Detail(buf, rig);
		check("universe verdict: an expired hold latches after the grace period",
			rig.latchedAt >= 3.5 && rig.latchedAt < 3.54, buf);
	}
	{
		// A compensated heuristic step changed WFD at 1.0 s; an unrelated,
		// uncompensated change follows at 1.3 s, before the UI's next tick.
		Rig rig;
		rig.Transition(1.0, 1.0, false);
		rig.verdict.NoteCompensation(1.0);
		rig.Transition(1.3, 2.0, false);
		rig.Run(1.4, 2.2);
		Detail(buf, rig);
		check("universe verdict: adoption takes only the compensated transition",
			rig.profile.translation.x() == 1.0 && rig.latchedAt >= 1.9 && rig.latchedAt < 1.94, buf);
	}
	{
		// A heuristic step at 1.0 s and an exact rebase at 1.3 s both land in
		// one stall: the profile is still behind the first when the second is
		// compensated, so the exact path leaves it, and the chain follows both.
		Rig rig;
		rig.Transition(1.0, 1.0, false);
		rig.verdict.NoteCompensation(1.0);
		rig.Transition(1.3, 2.0, false);
		if (UniverseVerdict::ExactDeltaRebasesProfile(rig.profile, Wfd(1.0)))
			rig.profile = Wfd(2.0);
		rig.verdict.NoteCompensation(1.3);
		rig.Run(1.4, 2.4);
		Detail(buf, rig);
		check("universe verdict: an exact rebase behind a pending adoption is followed",
			rig.latchedAt < 0.0 && rig.profile.translation.x() == 2.0, buf);
	}
	check("universe verdict: an exact rebase moves only a profile it started from",
		UniverseVerdict::ExactDeltaRebasesProfile(Wfd(1.0), Wfd(1.0)) &&
		!UniverseVerdict::ExactDeltaRebasesProfile(Wfd(0.0), Wfd(1.0)),
		"profile WFD must equal the rebase's previous WFD");
}
