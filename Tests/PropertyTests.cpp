
#include "Fuzz/FuzzTargets.h"
#include "../Driver/AlignmentField.h"
#include "../Driver/PoseTransform.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

// Randomized properties, seeded by --property-seed and sized by
// --property-trials like the solver's.
//
// Fuzz replay: every target in Fuzz/FuzzTargets.h over its seeds and seeded
// mutations of them, so each property holds in every run and not only when
// tools/fuzz.ps1 runs the coverage-guided fuzzer.
//
// Slew: SlewTowardAt in floating point. formal/lean/Slew.lean proves the step
// fraction keeps the device inside its translation and rotation budgets and
// never overshoots, in exact rational arithmetic and with the angle it is
// given. Here the same three claims are checked of the doubles the driver
// runs, the rotation measured on the quaternion nlerp actually produces.
namespace
{
using Check = void (*)(const char *, bool, const char *);

// ---------------------------------------------------------------------------
// Mutations

const char *const InterestingText[] = { "0", "-0", "1", "-1", "2", "3", "0.25", "4", "4.000000000000001",
	"1e308", "-1e308", "1e999", "1e-320", "4294967295", "4294967296", "10000", "10000.000000000002", "100",
	"100.00000000000001", "0.05", "0.049999999999999996", "1.0000000000000002", "1e-7", "nan", "\"\"",
	"\"x\"", "true", "false", "null", "[]", "{}", "[[[[[[[[[[[[[[[[[[", "\\u0000", "\"S-", "LHR-", ":", " ", "(",
	")", "S-99999999999", "sha256:", "questcalibrator-v", ".", "Feb 30", "99:99:99.9", "23:59:60.999" };

double InterestingDouble(std::mt19937 &rng)
{
	const double values[] = { 0.0, -0.0, 1.0, -1.0, 0.5, 2.0, std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), DBL_MAX, -DBL_MAX,
		DBL_MIN, 4.9e-324, 1e-6, 9.99999e-7, 0.25, 0.2499999999999999, 4.0, 4.000000000000001, 1e4,
		10000.000000000002, 100.0, 100.00000000000001, 1.0, 0.05, 0.04999999, 1e300, 1e-300, 1e154 };
	return values[rng() % (sizeof values / sizeof values[0])];
}

uint32_t InterestingWord(std::mt19937 &rng)
{
	const uint32_t values[] = { 0u, 1u, 2u, 3u, 7u, 8u, 9u, 63u, 64u, 0xFFFFFFFFu, 0x80000000u,
		protocol::Version, protocol::Version + 1u, vr::k_unMaxTrackedDeviceCount };
	return values[rng() % (sizeof values / sizeof values[0])];
}

std::string Mutate(std::string s, const std::vector<std::string> &seeds, bool binary, std::mt19937 &rng)
{
	const int edits = 1 + static_cast<int>(rng() % 6);
	for (int e = 0; e < edits; ++e)
	{
		const size_t pos = s.empty() ? 0 : rng() % (s.size() + 1);
		switch (rng() % (binary ? 9 : 7))
		{
		case 0:
			if (!s.empty())
				s[pos % s.size()] ^= static_cast<char>(1u << (rng() % 8));
			break;
		case 1:
			if (!s.empty())
				s[pos % s.size()] = "{}[]\",:\\-e.0 9(xS"[rng() % 17];
			break;
		case 2:
			s.insert(pos, 1, static_cast<char>(rng() % 256));
			break;
		case 3:
			if (!s.empty())
				s.erase(pos % s.size(), 1 + rng() % 8);
			break;
		case 4:
			if (!s.empty())
			{
				const size_t from = rng() % s.size();
				s.insert(pos, s.substr(from, 1 + rng() % 16));
			}
			break;
		case 5:
		{
			const std::string &other = seeds[rng() % seeds.size()];
			const size_t from = other.empty() ? 0 : rng() % other.size();
			s = s.substr(0, pos) + other.substr(from);
			break;
		}
		case 6:
		{
			// Replace a token (a run of number or name characters) with an edge.
			size_t start = pos < s.size() ? pos : 0;
			size_t end = start;
			while (end < s.size() && (std::isalnum(static_cast<unsigned char>(s[end])) ||
				s[end] == '.' || s[end] == '-' || s[end] == '+'))
				++end;
			s.replace(start, end - start, InterestingText[rng() % (sizeof InterestingText / sizeof InterestingText[0])]);
			break;
		}
		case 7:
			// Binary: an edge double on an 8-byte boundary (after the flag byte).
			if (s.size() > 9)
			{
				const size_t at = 1 + 8 * (rng() % ((s.size() - 1) / 8));
				const double d = InterestingDouble(rng);
				if (at + 8 <= s.size())
					std::memcpy(&s[at], &d, 8);
			}
			break;
		default:
			// Binary: an edge word on a 4-byte boundary.
			if (s.size() > 5)
			{
				const size_t at = 1 + 4 * (rng() % ((s.size() - 1) / 4));
				const uint32_t w = InterestingWord(rng);
				if (at + 4 <= s.size())
					std::memcpy(&s[at], &w, 4);
			}
			break;
		}
	}
	return s;
}

std::string Printable(const std::string &s)
{
	std::string out;
	for (char c : s.substr(0, 160))
		out += (c >= 32 && c < 127) ? c : '.';
	return out;
}

void FuzzReplay(Check check, int trials, uint32_t propertySeed)
{
	for (const auto &target : questcalfuzz::Targets())
	{
		const std::vector<std::string> seeds = target.seeds();
		const bool binary = std::string(target.name) == "request";
		std::mt19937 rng(propertySeed ^ static_cast<uint32_t>(std::hash<std::string>()(target.name)));
		std::string failure, input;
		int seedFailures = 0;
		auto run = [&](const std::string &in) -> bool
		{
			std::string why;
			try
			{
				why = target.check(reinterpret_cast<const uint8_t *>(in.data()), in.size());
			}
			catch (...)
			{
				why = "threw something other than the caller catches";
			}
			if (why.empty())
				return true;
			if (failure.empty())
			{
				failure = why;
				input = in;
			}
			return false;
		};
		for (const std::string &s : seeds)
			if (!run(s))
				++seedFailures;
		const int iterations = trials * 64;
		int failures = 0;
		for (int i = 0; i < iterations; ++i)
			if (!run(Mutate(seeds[rng() % seeds.size()], seeds, binary, rng)))
				++failures;

		char name[96], detail[512];
		snprintf(name, sizeof name, "fuzz replay: %s", target.name);
		snprintf(detail, sizeof detail, "%zu seeds (%d fail), %d mutations seed %u (%d fail)%s%s%s%s", seeds.size(),
			seedFailures, iterations, propertySeed, failures, failure.empty() ? "" : ": ", failure.c_str(),
			failure.empty() ? "" : " <- ", binary ? "" : Printable(input).c_str());
		check(name, seedFailures == 0 && failures == 0, detail);
	}
}

// ---------------------------------------------------------------------------
// SlewTowardAt

vr::HmdQuaternion_t RandomUnit(std::mt19937 &rng)
{
	std::normal_distribution<double> n(0.0, 1.0);
	double q[4] = { n(rng), n(rng), n(rng), n(rng) };
	const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	return { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };
}

// Rotates `q` by `angle` about a random axis.
vr::HmdQuaternion_t Near(const vr::HmdQuaternion_t &q, double angle, std::mt19937 &rng)
{
	vr::HmdQuaternion_t axis = RandomUnit(rng);
	const double n = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
	const vr::HmdQuaternion_t d{ std::cos(angle / 2), std::sin(angle / 2) * axis.x / n,
		std::sin(angle / 2) * axis.y / n, std::sin(angle / 2) * axis.z / n };
	return questcal::driverpose::Multiply(d, q);
}

// The rotation angle of a^-1 b, by atan2: acos loses the small angles a slew
// step is made of to rounding.
double AngleBetween(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b)
{
	const vr::HmdQuaternion_t d = questcal::driverpose::Multiply({ a.w, -a.x, -a.y, -a.z }, b);
	return 2.0 * std::atan2(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), std::abs(d.w));
}

void PointOf(const alignfield::EvalState &s, const double (&p)[3], double (&out)[3])
{
	auto r = questcal::driverpose::RotateVector(s.rot, p);
	for (int k = 0; k < 3; ++k)
		out[k] = r.v[k] + s.trans[k];
}

double Distance(const double (&a)[3], const double (&b)[3])
{
	return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

void SlewProperties(Check check, int trials, uint32_t propertySeed)
{
	std::mt19937 rng(propertySeed ^ 0x51E3u);
	std::uniform_real_distribution<double> unit(0.0, 1.0);
	const double spreads[] = { 0.001, 0.01, 0.05, 0.2, 1.0, 3.0 };   // rad between current and target
	int budgetFails = 0, overshootFails = 0, convergeFails = 0, snapFails = 0, finiteFails = 0;
	double worstTrans = 0.0, worstRotSmall = 0.0, worstRotAny = 0.0;
	const int n = trials * 32;
	for (int t = 0; t < n; ++t)
	{
		const alignfield::SlewLimits limits = t % 2 ? alignfield::FieldSlewLimits : alignfield::BaseSlewLimits;
		const double spread = spreads[t % 6];
		const vr::HmdQuaternion_t targetRot = RandomUnit(rng);
		double targetTrans[3], position[3];
		for (int k = 0; k < 3; ++k)
		{
			targetTrans[k] = (unit(rng) - 0.5) * 4.0;
			position[k] = (unit(rng) - 0.5) * 10.0;
		}
		alignfield::EvalState s;
		s.hasCurrent = true;
		s.generation = 5;
		s.lastTime = 100.0;
		s.rot = Near(targetRot, spread * unit(rng), rng);
		for (int k = 0; k < 3; ++k)
			s.trans[k] = targetTrans[k] + (unit(rng) - 0.5) * spread;

		alignfield::EvalState goal;
		goal.rot = targetRot;
		std::copy(targetTrans, targetTrans + 3, goal.trans);
		double goalPoint[3];
		PointOf(goal, position, goalPoint);

		// One step: inside both budgets, and no further from the target.
		const double dt = limits.maxGapSeconds * (0.01 + 0.99 * unit(rng));
		alignfield::EvalState before = s;
		alignfield::SlewTowardAt(targetRot, targetTrans, position, s.lastTime + dt, limits, 5, s);
		double p0[3], p1[3];
		PointOf(before, position, p0);
		PointOf(s, position, p1);
		const double moved = Distance(p0, p1);
		const double turned = AngleBetween(before.rot, s.rot);
		const double transRatio = moved / (limits.maxTranslationPerSec * dt);
		const double rotRatio = turned / (limits.maxRotationPerSec * dt);
		worstTrans = (std::max)(worstTrans, transRatio);
		worstRotAny = (std::max)(worstRotAny, rotRatio);
		if (spread <= 0.05)
			worstRotSmall = (std::max)(worstRotSmall, rotRatio);
		if (!std::isfinite(s.rot.w) || !questcalfuzz::UnitQuat(s.rot) || !questcalfuzz::Finite3(s.trans))
			++finiteFails;
		// Translation to rounding; rotation to what nlerp adds over slerp at
		// the corrections the driver slews (a few degrees), a small fraction.
		if (transRatio > 1.0 + 1e-9 || (spread <= 0.05 && rotRatio > 1.0 + 1e-4))
			++budgetFails;
		if (Distance(p1, goalPoint) > Distance(p0, goalPoint) + 1e-12 ||
			AngleBetween(s.rot, targetRot) > AngleBetween(before.rot, targetRot) + 1e-9)
			++overshootFails;

		// Held steady, the target is reached in the steps the budgets allow.
		const double need = (std::max)(Distance(p0, goalPoint) / (limits.maxTranslationPerSec * dt),
			AngleBetween(before.rot, targetRot) / (limits.maxRotationPerSec * dt));
		const int allowed = static_cast<int>(std::ceil(need * 1.5)) + 3;
		int steps = 1;
		while (steps < allowed && (Distance(p1, goalPoint) > 1e-9 || AngleBetween(s.rot, targetRot) > 1e-7))
		{
			alignfield::SlewTowardAt(targetRot, targetTrans, position, s.lastTime + dt, limits, 5, s);
			PointOf(s, position, p1);
			++steps;
		}
		if (Distance(p1, goalPoint) > 1e-9 || AngleBetween(s.rot, targetRot) > 1e-7)
			++convergeFails;

		// A new generation, a gap past the limit, or time standing still snaps.
		alignfield::EvalState a = before, b = before, c = before;
		alignfield::SlewTowardAt(targetRot, targetTrans, position, a.lastTime + dt, limits, 6, a);
		alignfield::SlewTowardAt(targetRot, targetTrans, position, b.lastTime + limits.maxGapSeconds * 1.01, limits, 5, b);
		alignfield::SlewTowardAt(targetRot, targetTrans, position, c.lastTime, limits, 5, c);
		for (const alignfield::EvalState *snapped : { &a, &b, &c })
			if (snapped->rot.w != targetRot.w || snapped->rot.x != targetRot.x || snapped->trans[0] != targetTrans[0] ||
				snapped->trans[1] != targetTrans[1] || snapped->trans[2] != targetTrans[2])
				++snapFails;
	}
	char detail[320];
	snprintf(detail, sizeof detail,
		"%d steps seed %u: budget %d overshoot %d converge %d snap %d finite %d; worst step/budget "
		"translation %.12f, rotation %.9f within 3 deg, %.9f at any angle",
		n, propertySeed, budgetFails, overshootFails, convergeFails, snapFails, finiteFails, worstTrans,
		worstRotSmall, worstRotAny);
	check("slew: floating point keeps the proved budgets", budgetFails + overshootFails + convergeFails +
		snapFails + finiteFails == 0, detail);

	// Where nlerp runs ahead of slerp: a step of three quarters of what
	// remains, at the largest step each profile allows (a quarter-second gap).
	// For a remaining half-angle t, nlerp at fraction f turns 2 atan2(f sin t,
	// 1 - f + f cos t) where slerp turns 2 f t: about t^2/48 over the budget at
	// f = 3/4, its worst. The step must be exactly that.
	double worstField = 0.0, worstBase = 0.0;
	bool bounded = true;
	for (const alignfield::SlewLimits *limits : { &alignfield::FieldSlewLimits, &alignfield::BaseSlewLimits })
	{
		const double maxAngle = limits->maxRotationPerSec * limits->maxGapSeconds;
		const double angle = maxAngle / 0.75;
		const double half = angle / 2.0;
		const vr::HmdQuaternion_t target{ std::cos(half), 0.0, std::sin(half), 0.0 };
		const double origin[3] = {}, none[3] = {};
		alignfield::EvalState s;
		s.hasCurrent = true;
		s.lastTime = 1.0;
		alignfield::SlewTowardAt(target, none, origin, 1.0 + limits->maxGapSeconds, *limits, 0, s);
		const double ratio = AngleBetween({ 1.0, 0.0, 0.0, 0.0 }, s.rot) / maxAngle;
		(limits == &alignfield::FieldSlewLimits ? worstField : worstBase) = ratio;
		const double nlerp = 2.0 * std::atan2(0.75 * std::sin(half), 0.25 + 0.75 * std::cos(half)) / maxAngle;
		bounded = bounded && std::abs(ratio - nlerp) <= 1e-9 && ratio <= 1.0 + half * half / 47.0;
	}
	snprintf(detail, sizeof detail, "worst step/budget: field %.9f, base calibration %.12f", worstField, worstBase);
	check("slew: nlerp overshoots the rotation budget by about t^2/48 at most", bounded, detail);
}

// ---------------------------------------------------------------------------
// Version order and quaternion normalization

void VersionProperties(Check check, int trials, uint32_t propertySeed)
{
	using namespace questcal::update;
	std::mt19937 rng(propertySeed ^ 0x7E55u);
	const char *const labels[] = { "", "alpha", "beta", "rc" };
	auto random = [&]()
	{
		Version v;
		v.major = rng() % 3;
		v.minor = rng() % 3;
		v.patch = rng() % 3;
		v.prereleaseLabel = labels[rng() % 4];
		v.prereleaseOrdinal = v.prereleaseLabel.empty() ? 0 : rng() % 3;
		return v;
	};
	int bad = 0;
	std::string first;
	const int n = trials * 64;
	for (int i = 0; i < n; ++i)
	{
		const Version a = random(), b = random(), c = random();
		const int ab = CompareVersions(a, b), ba = CompareVersions(b, a);
		const int bc = CompareVersions(b, c), ac = CompareVersions(a, c);
		const bool same = VersionString(a) == VersionString(b);
		std::string why;
		if (ab != -ba)
			why = "not antisymmetric";
		else if ((ab == 0) != same)
			why = "equal order but different versions";
		else if (ab <= 0 && bc <= 0 && ac > 0)
			why = "not transitive";
		else if (!IsPrerelease(a) && CompareVersions(a, a) != 0)
			why = "not reflexive";
		Version parsed;
		if (why.empty() && !IsPrerelease(a) &&
			(!ParseReleaseTag("questcalibrator-v" + VersionString(a), parsed) || CompareVersions(parsed, a) != 0))
			why = "a final version's tag does not parse back";
		if (why.empty() && IsPrerelease(a) && ParseReleaseTag("questcalibrator-v" + VersionString(a), parsed))
			why = "a prerelease tag parses as a release";
		if (!why.empty() && ++bad == 1)
			first = why + " at " + VersionString(a) + " / " + VersionString(b) + " / " + VersionString(c);
	}
	char detail[256];
	snprintf(detail, sizeof detail, "%d triples seed %u, %d bad%s%s", n, propertySeed, bad, bad ? ": " : "", first.c_str());
	check("updates: version order is a total order and tags round-trip", bad == 0, detail);
}

void NormalizeProperties(Check check, int trials, uint32_t propertySeed)
{
	std::mt19937 rng(propertySeed ^ 0x9A7u);
	std::uniform_int_distribution<int> exponent(-12, 300);
	int bad = 0, accepted = 0;
	const int n = trials * 64;
	for (int i = 0; i < n; ++i)
	{
		vr::HmdQuaternion_t unit = RandomUnit(rng);
		const double scale = std::pow(10.0, exponent(rng));
		vr::HmdQuaternion_t q{ unit.w * scale, unit.x * scale, unit.y * scale, unit.z * scale };
		if (i % 5 == 0)
			(&q.w)[rng() % 4] = InterestingDouble(rng);
		const vr::HmdQuaternion_t in = q;
		if (!questcal::numeric::NormalizeQuaternion(q))
			continue;
		++accepted;
		// Accepted: finite input, unit output, the same direction.
		const double big = (std::max)((std::max)(std::abs(in.w), std::abs(in.x)), (std::max)(std::abs(in.y), std::abs(in.z)));
		const double w = in.w / big, x = in.x / big, y = in.y / big, z = in.z / big;
		const double norm = std::sqrt(w * w + x * x + y * y + z * z);
		const double dot = (q.w * w + q.x * x + q.y * y + q.z * z) / norm;
		if (!questcalfuzz::UnitQuat(q) || !std::isfinite(in.w) || !std::isfinite(in.x) ||
			!std::isfinite(in.y) || !std::isfinite(in.z) || std::abs(dot - 1.0) > 1e-12)
			++bad;
	}
	char detail[160];
	snprintf(detail, sizeof detail, "%d quaternions seed %u, %d accepted, %d bad", n, propertySeed, accepted, bad);
	check("driver input: normalization is unit and keeps direction", bad == 0 && accepted > 0, detail);
}

// What the replay found, pinned whatever the seed: picojson throws, with an
// empty message, for a number past a double's range. The feed parse let it
// escape its invalid-list verdict, and the record loads showed a blank reason.
void OutOfRangeNumbers(Check check)
{
	using namespace questcal::update;
	ReleaseCandidate candidate;
	bool available = true;
	std::string error;
	bool selected = true, threw = false;
	try
	{
		selected = SelectReleaseCandidate("[{\"size\":1e999}]", Version(), candidate, available, error);
	}
	catch (const std::exception &)
	{
		threw = true;
	}
	check("updates: a number past double range is an invalid list",
		!threw && !selected && !available && error == "GitHub returned an invalid release list.", error.c_str());

	questcal::ProfileRecord record;
	questcal::ProfileParseResult result;
	std::string reason;
	const bool read = questcalfuzz::ReadProfile(
		"[{\"reference_tracking_system\":\"lighthouse\",\"target_tracking_system\":\"oculus\","
		"\"rotation_quat\":[1,0,0,0],\"translation_meters\":[1e999,0,0]}]", record, result, reason);
	check("persistence: a number past double range is refused with a reason", !read && !reason.empty(),
		reason.c_str());

	// strtod reads "nan", and NaN fails every range comparison, so a clock of
	// 22:09:nan passed as a known time.
	const std::string line = "Fri Sep 11 2026 22:09:nan [Info] - lighthouse: LHR-A3C36EA5 C: SOB: add S-5";
	const std::string why = questcalfuzz::CheckLighthouseLine(
		reinterpret_cast<const uint8_t *>(line.data()), line.size());
	check("lighthouse log: a clock that is not a number is not a time", why.empty(), why.c_str());
}

} // namespace

void RunPropertyScenarios(Check check, int trials, uint32_t propertySeed)
{
	OutOfRangeNumbers(check);
	FuzzReplay(check, trials, propertySeed);
	SlewProperties(check, trials, propertySeed);
	VersionProperties(check, trials, propertySeed);
	NormalizeProperties(check, trials, propertySeed);
}
