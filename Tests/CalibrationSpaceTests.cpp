#include "../Overlay/Calibration.h"
#include "../Overlay/CalibrationDriver.h"
#include "../Overlay/CalibrationSpace.h"
#include "../Overlay/PoseStreamHub.h"
#include "../common/TransformLimits.h"

#include <cstdio>
#include <cstring>
#include <limits>

// ApplyCalibrationDelta is where every continuous correction, re-anchor and
// headset re-center lands on the profile, and driver sync decides what the
// driver is sent. They run here as the overlay links them, against
// OverlayStubs.cpp instead of SteamVR.
namespace
{
using Check = void (*)(const char *, bool, const char *);
using Reason = CalibrationContext::DisableReason;

// A profile as a calibration leaves it.
CalibrationContext SolvedProfile()
{
	CalibrationContext ctx;
	ctx.validProfile = true;
	ctx.referenceTrackingSystem = "oculus";
	ctx.targetTrackingSystem = "lighthouse";
	ctx.SetCalibration(Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY())),
		Eigen::Vector3d(1.0, 0.2, -0.5), 1.0);
	return ctx;
}

void ProtectChaperone(CalibrationContext &ctx, const Eigen::Vector3d &standingCenter)
{
	ctx.chaperone.valid = true;
	ctx.chaperone.standingCenter = {};
	for (int axis = 0; axis < 3; ++axis)
	{
		ctx.chaperone.standingCenter.m[axis][axis] = 1.0f;
		ctx.chaperone.standingCenter.m[axis][3] = static_cast<float>(standingCenter(axis));
	}
	ctx.chaperone.playSpaceSize = { { 2.0f, 2.0f } };
	vr::HmdQuad_t wall{};
	wall.vCorners[1].v[0] = 2.0f;
	wall.vCorners[2].v[0] = 2.0f;
	wall.vCorners[2].v[1] = 2.0f;
	wall.vCorners[3].v[1] = 2.0f;
	ctx.chaperone.geometry = { wall };
}

Eigen::Vector3d StandingCenter(const CalibrationContext &ctx)
{
	const auto &m = ctx.chaperone.standingCenter.m;
	return Eigen::Vector3d(m[0][3], m[1][3], m[2][3]);
}

// Where a calibration puts a target-space point.
Eigen::Vector3d Mapped(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation,
	const Eigen::Vector3d &point)
{
	return rotation * point + translation;
}

bool Unchanged(const CalibrationContext &before, const CalibrationContext &after)
{
	if (before.fieldAnchors.size() != after.fieldAnchors.size())
		return false;
	for (size_t i = 0; i < before.fieldAnchors.size(); ++i)
	{
		const auto &a = before.fieldAnchors[i];
		const auto &b = after.fieldAnchors[i];
		if (a.position != b.position || a.rotation.coeffs() != b.rotation.coeffs() ||
			a.translationMeters != b.translationMeters)
			return false;
	}
	return before.transform.rotation.coeffs() == after.transform.rotation.coeffs() &&
		before.transform.translationMeters == after.transform.translationMeters &&
		before.baseGeneration == after.baseGeneration &&
		before.fieldGeneration == after.fieldGeneration &&
		before.persistence.revision == after.persistence.revision &&
		before.persistence.profileDirty == after.persistence.profileDirty &&
		before.persistence.settingsDirty == after.persistence.settingsDirty &&
		std::memcmp(&before.chaperone.standingCenter, &after.chaperone.standingCenter,
			sizeof before.chaperone.standingCenter) == 0;
}

void RunDeltaCompositionScenario(Check check)
{
	// Checked by where the calibration puts target-space points, not by
	// restating the formula: after the delta, every point lands where the old
	// calibration put it, moved by the delta.
	const Eigen::Quaterniond delta(Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.2, 1.0, 0.1).normalized()));
	const Eigen::Vector3d shift(0.03, -0.01, 0.02);
	const Eigen::Vector3d points[] = { { 0, 0, 0 }, { 1.5, 0.3, -2.0 }, { -0.4, 1.7, 0.9 } };
	char detail[192];
	for (bool snap : { true, false })
	{
		CalibrationContext ctx = SolvedProfile();
		const CalibrationContext before = ctx;
		const bool applied = questcal::ApplyCalibrationDelta(ctx, delta, shift, snap, 5.0, false);
		double worst = 0.0;
		for (const auto &point : points)
		{
			const Eigen::Vector3d expected = delta *
				Mapped(before.transform.rotation, before.transform.translationMeters, point) + shift;
			worst = (std::max)(worst, (Mapped(ctx.transform.rotation,
				ctx.transform.translationMeters, point) - expected).norm());
		}
		// A snap bumps the base generation (the driver jumps) and advances
		// the revision with Settings; a slewed correction touches neither.
		const bool generation = ctx.baseGeneration == before.baseGeneration + (snap ? 1u : 0u);
		const bool persisted = ctx.persistence.profileDirty &&
			ctx.persistence.settingsDirty == snap &&
			ctx.persistence.revision == before.persistence.revision + (snap ? 1u : 0u);
		snprintf(detail, sizeof detail, "applied %d, worst point %.2e m, generation %d, persisted %d",
			applied, worst, generation, persisted);
		check(snap ? "calibration delta: a snapped delta composes onto the profile"
			: "calibration delta: a slewed correction composes onto the profile",
			applied && worst < 1e-12 && generation && persisted, detail);
	}
}

void RunDeltaAnchorScenario(Check check)
{
	// Field anchors are absolute solves at their positions; a delta moves them
	// with the base, so each still maps its own neighborhood as before, moved.
	CalibrationContext ctx = SolvedProfile();
	CalibrationContext::FieldAnchor anchor;
	anchor.position = Eigen::Vector3d(0.5, 1.0, 0.5);
	anchor.rotation = (Eigen::Quaterniond(Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY())) *
		ctx.transform.rotation).normalized();
	anchor.translationMeters = ctx.transform.translationMeters + Eigen::Vector3d(0.004, 0.0, -0.002);
	ctx.fieldAnchors.push_back(anchor);
	const CalibrationContext before = ctx;
	const Eigen::Quaterniond delta(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()));
	const Eigen::Vector3d shift(0.1, 0.0, 0.05);
	const bool applied = questcal::ApplyCalibrationDelta(ctx, delta, shift, true, 5.0, false);
	const auto &moved = ctx.fieldAnchors.front();
	const Eigen::Vector3d point(-0.3, 1.2, 0.8);
	const double mapping = (Mapped(moved.rotation, moved.translationMeters, point) -
		(delta * Mapped(anchor.rotation, anchor.translationMeters, point) + shift)).norm();
	const double position = (moved.position - (delta * anchor.position + shift)).norm();
	// The driver's field blend must snap with the base.
	const bool generation = ctx.fieldGeneration == before.fieldGeneration + 1;
	char detail[160];
	snprintf(detail, sizeof detail, "applied %d, mapping %.2e m, position %.2e m, generation %d",
		applied, mapping, position, generation);
	check("calibration delta: field anchors move with the profile",
		applied && mapping < 1e-12 && position < 1e-12 && generation, detail);
}

void RunDeltaChaperoneScenario(Check check)
{
	// The protected chaperone lives in the reference space: a headset
	// re-center moves it with the delta; a correction or re-anchor, which
	// moves only the target side, leaves it.
	const Eigen::Quaterniond delta(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
	const Eigen::Vector3d shift(0.2, 0.0, -0.1);
	const Eigen::Vector3d center(1.0, 0.0, 2.0);
	CalibrationContext recentered = SolvedProfile();
	ProtectChaperone(recentered, center);
	const bool recenterApplied = questcal::ApplyCalibrationDelta(recentered, delta, shift, true, 5.0, true);
	const double recenterError = (StandingCenter(recentered) - (delta * center + shift)).norm();
	CalibrationContext corrected = SolvedProfile();
	ProtectChaperone(corrected, center);
	const bool correctionApplied = questcal::ApplyCalibrationDelta(corrected, delta, shift, true, 5.0, false);
	const double correctionError = (StandingCenter(corrected) - center).norm();
	char detail[160];
	snprintf(detail, sizeof detail, "re-center %d moved off by %.2e m; correction %d left off by %.2e m",
		recenterApplied, recenterError, correctionApplied, correctionError);
	check("calibration delta: the chaperone moves only with the reference space",
		recenterApplied && recenterError < 1e-5 && correctionApplied && correctionError == 0.0,
		detail);
}

void RunDeltaRefusalScenario(Check check)
{
	// Every refusal comes before the first write: nothing moves, nothing is
	// marked for saving.
	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double edge = protocol::limits::MaxAbsTranslationMeters - 0.5;
	struct Case
	{
		const char *name;
		Eigen::Quaterniond rotation;
		Eigen::Vector3d translation;
		bool moveChaperone;
		void (*setup)(CalibrationContext &);
	};
	const Case cases[] = {
		{ "non-finite rotation", Eigen::Quaterniond(nan, 0, 0, 0), Eigen::Vector3d::Zero(), false,
			[](CalibrationContext &) {} },
		{ "translation out of range",
			Eigen::Quaterniond::Identity(), Eigen::Vector3d(2.0 * edge, 0, 0), false,
			[](CalibrationContext &) {} },
		{ "result out of range", Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0, 0), false,
			[](CalibrationContext &ctx)
			{
				ctx.SetCalibration(ctx.transform.rotation,
					Eigen::Vector3d(protocol::limits::MaxAbsTranslationMeters - 0.5, 0, 0), 1.0);
			} },
		{ "anchor out of range", Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0, 0), false,
			[](CalibrationContext &ctx)
			{
				CalibrationContext::FieldAnchor anchor;
				anchor.position = Eigen::Vector3d(protocol::limits::MaxAbsAnchorPositionMeters - 0.5, 0, 0);
				anchor.rotation = ctx.transform.rotation;
				anchor.translationMeters = ctx.transform.translationMeters;
				ctx.fieldAnchors.push_back(anchor);
			} },
		{ "chaperone out of range", Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0, 0), true,
			[](CalibrationContext &ctx)
			{
				ProtectChaperone(ctx, Eigen::Vector3d(
					protocol::limits::MaxAbsChaperoneCoordinateMeters - 0.5, 0, 0));
			} },
	};
	for (const auto &c : cases)
	{
		CalibrationContext ctx = SolvedProfile();
		c.setup(ctx);
		const CalibrationContext before = ctx;
		const bool applied = questcal::ApplyCalibrationDelta(ctx, c.rotation, c.translation,
			true, 5.0, c.moveChaperone);
		char name[96];
		snprintf(name, sizeof name, "calibration delta: refuses a %s and changes nothing", c.name);
		check(name, !applied && Unchanged(before, ctx), applied ? "applied" : "state changed");
	}
}

void RunDriverDisableReasonScenario(Check check)
{
	// The status line names the cause the profile was switched off for; each
	// wants a different action from the player.
	PoseStreamHub hub;   // never started: no ring, so no fresh headset pose
	questcal::StartCalibrationSpace(hub, 1e-7);
	CalibrationContext ctx = SolvedProfile();
	ctx.profileUniverseUnsafe = true;
	ctx.frameMovesLost = true;
	questcal::SynchronizeCalibrationDriver(ctx);
	const bool lost = !ctx.enabled && ctx.disableReason == Reason::FrameMovesLost;
	ctx.frameMovesLost = false;
	questcal::SynchronizeCalibrationDriver(ctx);
	const bool recentered = !ctx.enabled && ctx.disableReason == Reason::UniverseUnsafe;
	// A new calibration takes the universe over and forgets the old cause.
	ctx.frameMovesLost = true;
	questcal::RebindCalibrationUniverse(ctx, "headset-serial", true);
	const bool rebound = !ctx.profileUniverseUnsafe && !ctx.frameMovesLost;
	// Safe again, but with no SteamVR there is no headset to check against.
	questcal::SynchronizeCalibrationDriver(ctx);
	const bool noHeadset = !ctx.enabled && ctx.disableReason == Reason::HmdMismatch;
	questcal::StopCalibrationSpace();
	char detail[128];
	snprintf(detail, sizeof detail, "lost %d, re-centered %d, rebound %d, no headset %d",
		lost, recentered, rebound, noHeadset);
	check("driver sync: each disable cause keeps its own reason",
		lost && recentered && rebound && noHeadset, detail);
}
}

void RunCalibrationSpaceScenarios(Check check)
{
	RunDeltaCompositionScenario(check);
	RunDeltaAnchorScenario(check);
	RunDeltaChaperoneScenario(check);
	RunDeltaRefusalScenario(check);
	RunDriverDisableReasonScenario(check);
}
