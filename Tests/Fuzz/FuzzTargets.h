#pragma once

// The properties every untrusted input must keep, one function per input the
// program parses or validates. Each takes arbitrary bytes and returns "" when
// every property holds, or the property that failed. Two drivers run them:
// Tests/Fuzz/FuzzMain.cpp under libFuzzer and AddressSanitizer
// (tools/fuzz.ps1), and Tests/PropertyTests.cpp, which replays the seeds
// below and seeded mutations of them in every harness run. Refusing an input
// is always allowed; what an input may not do is crash, throw anything but
// the exception its caller catches, or be accepted in a shape the rest of the
// program cannot live with.
//
//   CheckProfileRecord  the Config record (ProfileRecordJson.h): refused with
//                       a std::exception or accepted, and what is accepted
//                       writes and reads back as the same record
//   CheckReleaseFeed    the update feed (UpdatePolicy.h): a selected package
//                       is newer, canonical, digested and sized; a release
//                       tag parses only in its one canonical spelling
//   CheckLighthouseLine a vrserver.txt line (LighthouseLog.cpp): events are
//                       well formed and every time known is finite
//   CheckDriverRequest  a pipe request into vrserver (IPCProtocolGate.h,
//                       ProtocolValidation.h): only a handshaken, versioned
//                       mutation is dispatched, and anything accepted is
//                       bounded and keeps every pose the driver computes from
//                       it finite

#include "../../Driver/AlignmentField.h"
#include "../../Driver/IPCProtocolGate.h"
#include "../../Driver/PoseTransform.h"
#include "../../Driver/ProtocolValidation.h"
#include "../../Overlay/LighthouseLog.h"
#include "../../Overlay/ProfileRecordJson.h"
#include "../../Overlay/UpdatePolicy.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace questcalfuzz
{

inline std::string Text(const uint8_t *data, size_t size)
{
	return std::string(reinterpret_cast<const char *>(data), size);
}

// ---------------------------------------------------------------------------
// Config record

// The one parse Configuration.cpp's LoadProfile makes of the Config value.
inline bool ReadProfile(const std::string &text, questcal::ProfileRecord &record,
	questcal::ProfileParseResult &result, std::string &error)
{
	try
	{
		questcal::RejectExcessiveJsonNesting(text);
		std::istringstream in(text);
		picojson::value v = questcal::ParseProfileEnvelope(in);
		questcal::LegacyProfileSettings legacy;
		result = questcal::ParseProfileObject(record, legacy, v.get<picojson::object>(),
			protocol::SetAlignmentField::MaxAnchors);
		return true;
	}
	catch (const std::exception &e)
	{
		error = e.what();
		return false;
	}
}

inline bool QuatNear(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b)
{
	// Normalizing a unit quaternion again may move it an ulp or two.
	const double tol = 1e-15;
	return std::abs(a.w() - b.w()) <= tol && std::abs(a.x() - b.x()) <= tol &&
		std::abs(a.y() - b.y()) <= tol && std::abs(a.z() - b.z()) <= tol;
}

inline std::string ProfileDifference(const questcal::ProfileRecord &a, const questcal::ProfileRecord &b)
{
	std::string d;
	auto note = [&d](const char *field) { d += d.empty() ? field : (std::string("/") + field); };
	if (a.referenceTrackingSystem != b.referenceTrackingSystem) note("reference");
	if (a.targetTrackingSystem != b.targetTrackingSystem) note("target");
	if (!QuatNear(a.rotation, b.rotation)) note("rotation");
	if (a.translationMeters != b.translationMeters) note("translation");
	if (a.scale != b.scale) note("scale");
	if (a.timeOffset != b.timeOffset) note("timeOffset");
	if (a.calibrationUnixTime != b.calibrationUnixTime) note("calibrationTime");
	if (a.universeUnsafe != b.universeUnsafe) note("universeUnsafe");
	if (a.universeValid != b.universeValid) note("universeValid");
	if (a.universeHmdSerial != b.universeHmdSerial) note("universeSerial");
	if (a.universeValid && b.universeValid &&
		(!QuatNear(a.universeRotation, b.universeRotation) || a.universeTranslation != b.universeTranslation))
		note("universeBaseline");
	if (a.fieldEnabled != b.fieldEnabled) note("fieldEnabled");
	if (a.continuousEnabled != b.continuousEnabled) note("continuousEnabled");
	if (a.continuousTrackerSerial != b.continuousTrackerSerial) note("continuousSerial");
	if (a.continuousLatencyReestimation != b.continuousLatencyReestimation) note("continuousLatency");
	if (a.continuousRequireTrigger != b.continuousRequireTrigger) note("continuousTrigger");
	if (a.continuousNoPause != b.continuousNoPause) note("continuousNoPause");
	if (a.hideMountedTracker != b.hideMountedTracker) note("hideMountedTracker");
	if (a.mountExtrinsic.valid != b.mountExtrinsic.valid) note("mount.valid");
	else if (a.mountExtrinsic.valid &&
		(!QuatNear(a.mountExtrinsic.rotation, b.mountExtrinsic.rotation) ||
		 a.mountExtrinsic.translationMeters != b.mountExtrinsic.translationMeters ||
		 a.mountExtrinsic.rotationRmsDeg != b.mountExtrinsic.rotationRmsDeg ||
		 a.mountExtrinsic.translationRmsM != b.mountExtrinsic.translationRmsM))
		note("mount");
	if (a.fieldAnchors.size() != b.fieldAnchors.size()) note("anchorCount");
	else
		for (size_t i = 0; i < a.fieldAnchors.size(); ++i)
			if (a.fieldAnchors[i].position != b.fieldAnchors[i].position ||
				!QuatNear(a.fieldAnchors[i].rotation, b.fieldAnchors[i].rotation) ||
				a.fieldAnchors[i].translationMeters != b.fieldAnchors[i].translationMeters)
				note("anchor");
	return d;
}

inline std::string CheckProfileRecord(const uint8_t *data, size_t size)
{
	questcal::ProfileRecord first;
	questcal::ProfileParseResult parsed;
	std::string error;
	if (!ReadProfile(Text(data, size), first, parsed, error))
	{
		// Refused: the profile is kept, marked unreadable, and the reason is
		// the one thing the error banner tells the user.
		return error.empty() ? "a profile is refused without a reason" : "";
	}

	// Accepted: the record the next save writes must load as this one, or a
	// profile that loaded once is gone (or different) at the next launch.
	const uint32_t revision = parsed.revision.present ? parsed.revision.value : 1;
	std::ostringstream out;
	questcal::WriteProfile(first, revision, out);
	questcal::ProfileRecord second;
	questcal::ProfileParseResult reparsed;
	if (!ReadProfile(out.str(), second, reparsed, error))
		return "an accepted profile does not read back once written: " + error;
	if (!reparsed.revision.present || reparsed.revision.value != revision)
		return "the revision does not survive a write";
	const std::string diff = ProfileDifference(first, second);
	if (!diff.empty())
		return "an accepted profile reads back different once written: " + diff;
	return "";
}

// ---------------------------------------------------------------------------
// Update feed

inline bool SameVersion(const questcal::update::Version &a, const questcal::update::Version &b)
{
	return questcal::update::CompareVersions(a, b) == 0 && a.prereleaseLabel == b.prereleaseLabel &&
		a.prereleaseOrdinal == b.prereleaseOrdinal;
}

// The first three bytes are the running version; the rest is the feed. The
// whole input is also tried as a release tag and as a digest.
inline std::string CheckReleaseFeed(const uint8_t *data, size_t size)
{
	using namespace questcal::update;
	const std::string text = Text(data, size);

	Version tagged;
	if (ParseReleaseTag(text, tagged) && text != "questcalibrator-v" + VersionString(tagged))
		return "a release tag parses in a spelling other than its canonical one";
	std::array<unsigned char, 32> digest;
	if (ParseSha256Digest(text, digest))
	{
		static const char hex[] = "0123456789abcdef";
		std::string rendered = "sha256:";
		for (unsigned char b : digest)
		{
			rendered += hex[b >> 4];
			rendered += hex[b & 15];
		}
		std::string lowered = text;
		for (char &c : lowered)
			if (c >= 'A' && c <= 'F')
				c = static_cast<char>(c - 'A' + 'a');
		if (lowered != rendered)
			return "a digest parses to bytes other than the ones it spells";
	}

	if (size < 3)
		return "";
	Version current;
	current.major = data[0] % 4;
	current.minor = data[1] % 4;
	current.patch = data[2] % 4;
	ReleaseCandidate candidate;
	bool available = false;
	std::string error;
	const bool selected = SelectReleaseCandidate(text.substr(3), current, candidate, available, error);
	if (selected != error.empty())
		return "the verdict and the error disagree";
	if (!selected && !(available || error == "GitHub returned an invalid release list."))
		return "a refusal that is neither a bad list nor a bad newest release";
	if (!selected || !available)
		return "";

	Version fromTag;
	const std::string base = "https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/";
	if (CompareVersions(candidate.version, current) <= 0)
		return "the offered release is not newer than the running build";
	if (IsPrerelease(candidate.version))
		return "a prerelease is offered";
	if (!ParseReleaseTag(candidate.tag, fromTag) || !SameVersion(fromTag, candidate.version))
		return "the offered version is not its tag's";
	if (candidate.packageName != CanonicalPackageName(candidate.version))
		return "the offered package is not the canonical one";
	if (candidate.releaseUrl != base + "tag/" + candidate.tag ||
		candidate.downloadUrl != base + "download/" + candidate.tag + "/" + candidate.packageName)
		return "the offered release points outside the repository";
	if (!ParseSha256Digest(candidate.digest, digest))
		return "the offered package has no digest";
	if (candidate.size < 128u * 1024u || candidate.size > 64u * 1024u * 1024u)
		return "the offered package size is out of range";
	return "";
}

// ---------------------------------------------------------------------------
// vrserver.txt

inline std::string CheckLighthouseLine(const uint8_t *data, size_t size)
{
	using lighthouselog::Event;
	const std::string line = Text(data, size);
	double stamp = 0.0;
	if (lighthouselog::ParseTimestamp(line, stamp) && !std::isfinite(stamp))
		return "a timestamp parses to a time that is not finite";

	Event e;
	e.channel = 77;
	e.visibleChannels = { 1, 2, 3 };
	e.historical = true;
	if (!lighthouselog::ParseLine(line, e))
		return "";
	if (e.timeKnown && !std::isfinite(e.unixTime))
		return "an event's known time is not finite";
	if (e.serial.compare(0, 4, "LHR-") != 0)
		return "an event's serial is not an LHR serial";
	if (e.historical)
		return "the parse left an earlier event's field behind";
	if (e.visibleChannels.size() != e.visibleIds.size())
		return "visible channels and ids differ in length";
	for (int channel : e.visibleChannels)
		if (channel < 0)
			return "a visible channel is negative";
	switch (e.kind)
	{
	case Event::Kind::StationAdded:
		if (!e.visibleKnown || e.channel < 0 || e.visibleChannels.empty() ||
			e.visibleChannels.back() != e.channel || e.visibleIds.back() != e.stationId)
			return "an add is not in the visible set it reports";
		break;
	case Event::Kind::StationDropped:
		if (!e.visibleKnown || e.channel < 0)
			return "a drop names no station";
		break;
	case Event::Kind::NoneSeen:
		if (!e.visibleKnown || !e.visibleChannels.empty())
			return "no station seen, yet some visible";
		break;
	default:
		if (e.visibleKnown)
			return "a bootstrap line claims a visible set";
		break;
	}
	return "";
}

// ---------------------------------------------------------------------------
// Pipe requests into vrserver

inline bool Finite3(const double (&v)[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline bool UnitQuat(const vr::HmdQuaternion_t &q)
{
	const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	return std::isfinite(n) && std::abs(n - 1.0) <= 1e-12;
}

// Poses at the edges of what the overlay's ring gate lets through: the driver
// transforms whatever vrserver hands it, and must never hand back a NaN.
inline std::vector<vr::DriverPose_t> EdgePoses()
{
	std::vector<vr::DriverPose_t> poses;
	const double edges[] = { 0.0, 1.0, -protocol::limits::MaxAbsPosePositionMeters,
		protocol::limits::MaxAbsPosePositionMeters };
	for (double e : edges)
	{
		vr::DriverPose_t pose{};
		pose.qWorldFromDriverRotation = { 0.5, 0.5, -0.5, 0.5 };
		pose.qDriverFromHeadRotation = { 1.0, 0.0, 0.0, 0.0 };
		pose.qRotation = { 1.0, 0.0, 0.0, 0.0 };
		for (int k = 0; k < 3; ++k)
		{
			pose.vecWorldFromDriverTranslation[k] = e;
			pose.vecPosition[k] = -e;
			pose.vecVelocity[k] = e == 0.0 ? 0.0 : protocol::limits::MaxAbsLinearVelocityMetersPerSecond;
			pose.vecAcceleration[k] = e;
		}
		poses.push_back(pose);
	}
	return poses;
}

inline bool PoseFinite(const vr::DriverPose_t &p)
{
	return std::isfinite(p.qWorldFromDriverRotation.w) && std::isfinite(p.qWorldFromDriverRotation.x) &&
		std::isfinite(p.qWorldFromDriverRotation.y) && std::isfinite(p.qWorldFromDriverRotation.z) &&
		Finite3(p.vecWorldFromDriverTranslation) && Finite3(p.vecPosition) && Finite3(p.vecVelocity) &&
		Finite3(p.vecAcceleration) && std::isfinite(p.poseTimeOffset);
}

inline std::string CheckTransform(const protocol::SetDeviceTransform &t)
{
	using namespace protocol::limits;
	if (t.openVRID >= vr::k_unMaxTrackedDeviceCount || t.enabled > 1 || t.hidden > 1)
		return "an accepted transform has an out-of-range id or flag";
	if (!questcal::numeric::IsBoundedVector3(t.translation.v, MaxAbsTranslationMeters) ||
		!(t.scale >= MinScale && t.scale <= MaxScale) ||
		!questcal::numeric::IsFiniteBounded(t.timeOffset, MaxAbsTimeOffsetSeconds))
		return "an accepted transform is out of bounds";
	if (!UnitQuat(t.rotation))
		return "an accepted transform's rotation is not unit length";
	for (vr::DriverPose_t pose : EdgePoses())
	{
		questcal::driverpose::Apply(pose, t.rotation, t.translation.v, t.scale, t.timeOffset);
		if (!PoseFinite(pose))
			return "an accepted transform makes a pose that is not finite";
	}
	return "";
}

inline std::string CheckField(const protocol::SetAlignmentField &f)
{
	using namespace protocol::limits;
	if (f.enabled > 1 || f.anchorCount > protocol::SetAlignmentField::MaxAnchors ||
		!(f.sigmaMeters >= MinFieldSigmaMeters && f.sigmaMeters <= MaxFieldSigmaMeters))
		return "an accepted field is out of range";
	for (uint32_t i = 0; i < protocol::SetAlignmentField::MaxAnchors; ++i)
	{
		const protocol::FieldAnchor &a = f.anchors[i];
		if (i >= f.anchorCount)
		{
			if (!(a == protocol::FieldAnchor{}))
				return "an accepted field keeps bytes past its anchor count";
			continue;
		}
		if (!questcal::numeric::IsBoundedVector3(a.position, MaxAbsAnchorPositionMeters) ||
			!questcal::numeric::IsBoundedVector3(a.translationDelta, MaxAbsAnchorDeltaMeters) ||
			!UnitQuat(a.rotationDelta))
			return "an accepted anchor is out of bounds";
	}
	// The blend and the slew the pose threads run on it, anywhere a device can be.
	const double places[][3] = { { 0.0, 0.0, 0.0 }, { 1.5, 1.0, -2.0 },
		{ MaxAbsPosePositionMeters, 0.0, -MaxAbsPosePositionMeters } };
	for (const auto &place : places)
	{
		vr::HmdQuaternion_t rot;
		double trans[3];
		alignfield::BlendAt(f, place, rot, trans);
		if (!UnitQuat(rot) || !Finite3(trans))
			return "an accepted field blends to a transform that is not finite";
		alignfield::EvalState state;
		alignfield::Evaluate(f, place, 1.0, state);
		alignfield::EvalState moved = state;
		moved.rot = { 0.9, 0.1, -0.3, 0.3 };
		const double norm = std::sqrt(0.81 + 0.01 + 0.09 + 0.09);
		moved.rot = { moved.rot.w / norm, moved.rot.x / norm, moved.rot.y / norm, moved.rot.z / norm };
		alignfield::Evaluate(f, place, 1.01, moved);
		if (!UnitQuat(state.rot) || !Finite3(state.trans) || !UnitQuat(moved.rot) || !Finite3(moved.trans))
			return "an accepted field slews to a transform that is not finite";
	}
	return "";
}

// The first byte says whether this connection already completed a handshake;
// the rest overlays a Request.
inline std::string CheckDriverRequest(const uint8_t *data, size_t size)
{
	if (size < 1)
		return "";
	questcal::ipc::ConnectionState connection;
	connection.handshakeComplete = (data[0] & 1) != 0;
	const bool handshaken = connection.handshakeComplete;
	protocol::Request request;
	std::memcpy(&request, data + 1, (std::min)(size - 1, sizeof request));

	protocol::Response response;
	const bool dispatch = questcal::ipc::PrepareRequest(request, connection, response);
	if (request.type == protocol::RequestHandshake)
	{
		if (dispatch || response.type != protocol::ResponseHandshake ||
			connection.handshakeComplete != (request.protocol.version == protocol::Version))
			return "a handshake is answered wrongly";
		return "";
	}
	if (!dispatch)
	{
		if (response.type != protocol::ResponseInvalid)
			return "a refused request is not answered as invalid";
		return "";
	}
	if (!handshaken || request.protocol.version != protocol::Version ||
		(request.type != protocol::RequestSetDeviceTransform && request.type != protocol::RequestSetRuntimeState))
		return "a request is dispatched without a handshake, a version or a mutation";

	if (request.type == protocol::RequestSetDeviceTransform)
	{
		protocol::SetDeviceTransform sanitized;
		if (!questcal::driverinput::ValidateAndSanitize(request.setDeviceTransform, sanitized))
			return "";
		return CheckTransform(sanitized);
	}
	protocol::SetRuntimeState sanitized;
	std::memset(&sanitized, 0xA5, sizeof sanitized);
	const protocol::SetRuntimeState before = sanitized;
	if (!questcal::driverinput::ValidateAndSanitize(request.setRuntimeState, sanitized))
	{
		if (std::memcmp(&before, &sanitized, sizeof sanitized) != 0)
			return "a refused runtime state wrote part of its output";
		return "";
	}
	if ((sanitized.hiddenMask & ~sanitized.enabledMask) != 0)
		return "an accepted runtime state hides a device it does not enable";
	std::string why = CheckTransform(sanitized.transform);
	if (why.empty())
		why = CheckField(sanitized.field);
	return why;
}

// ---------------------------------------------------------------------------
// Seeds: inputs each target accepts, for the fuzzer to start from.

inline std::vector<std::string> ProfileSeeds()
{
	std::vector<std::string> seeds;
	questcal::ProfileRecord r;
	r.valid = true;
	r.referenceTrackingSystem = "lighthouse";
	r.targetTrackingSystem = "oculus";
	r.rotation = Eigen::Quaterniond(0.9, 0.1, -0.3, 0.2).normalized();
	r.translationMeters = Eigen::Vector3d(0.5, -0.25, 1.75);
	r.scale = 1.01;
	r.timeOffset = 0.012;
	r.calibrationUnixTime = 1757620000.0;
	std::ostringstream minimal;
	questcal::WriteProfile(r, 3, minimal);
	seeds.push_back(minimal.str());

	r.universeValid = true;
	r.universeHmdSerial = "1WMHH000X00000";
	r.universeRotation = Eigen::Quaterniond(0.7, 0.0, 0.7, 0.0).normalized();
	r.universeTranslation = Eigen::Vector3d(0.1, 1.6, -0.2);
	r.continuousEnabled = true;
	r.continuousTrackerSerial = "LHR-A3C36EA5";
	r.mountExtrinsic.valid = true;
	r.mountExtrinsic.rotation = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);
	r.mountExtrinsic.translationMeters = Eigen::Vector3d(0.0, 0.08, 0.1);
	r.mountExtrinsic.rotationRmsDeg = 0.2;
	r.mountExtrinsic.translationRmsM = 0.003;
	r.fieldEnabled = true;
	questcal::PersistedFieldAnchor a;
	a.position = Eigen::Vector3d(1.5, 0.0, -2.0);
	a.rotation = (r.rotation * Eigen::Quaterniond(Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY()))).normalized();
	a.translationMeters = r.translationMeters + Eigen::Vector3d(0.01, 0.0, -0.02);
	r.fieldAnchors.push_back(a);
	std::ostringstream full;
	questcal::WriteProfile(r, 7, full);
	seeds.push_back(full.str());

	// A Config-only release's record: no revision, settings_version 1 and the
	// global preferences embedded.
	seeds.push_back("[{\"reference_tracking_system\":\"lighthouse\",\"target_tracking_system\":\"oculus\","
		"\"rotation_quat\":[1,0,0,0],\"translation_meters\":[0.1,0,-0.2],\"scale\":1.05,"
		"\"apply_time_offset\":true,\"solve_scale\":true,\"ui_advanced\":false,\"calibration_speed\":1}]");
	return seeds;
}

inline std::vector<std::string> FeedSeeds()
{
	const std::string digest = "sha256:" + std::string(64, 'a');
	const std::string release =
		"{\"draft\":false,\"prerelease\":false,\"tag_name\":\"questcalibrator-v2.1.0\","
		"\"html_url\":\"https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/tag/questcalibrator-v2.1.0\","
		"\"assets\":[{\"name\":\"QuestCalibrator-2.1.0.zip\",\"size\":1048576,\"digest\":\"" + digest + "\","
		"\"browser_download_url\":\"https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/download/"
		"questcalibrator-v2.1.0/QuestCalibrator-2.1.0.zip\"}]}";
	const std::string others =
		",{\"draft\":true,\"prerelease\":false,\"tag_name\":\"questcalibrator-v9.0.0\"}"
		",{\"draft\":false,\"prerelease\":true,\"tag_name\":\"questcalibrator-v3.0.0\"}"
		",{\"draft\":false,\"prerelease\":false,\"tag_name\":\"v4.0.0\"}";
	return { std::string("\x01\x00\x00", 3) + "[" + release + others + "]",
		std::string("\x02\x01\x00", 3) + "[" + release + "]",
		"questcalibrator-v1.20.3", digest };
}

inline std::vector<std::string> LighthouseSeeds()
{
	const std::string prefix = "Fri Sep 11 2026 22:09:41.018 [Info] - lighthouse: ";
	return {
		prefix + "LHR-A3C36EA5 C: SOB: add S-16 also seeing S-5 (D3D4E73B) S-8 (170EE067)",
		prefix + "LHR-A3C36EA5 C: SOB: add S-9 (generation changed) also seeing S-5 (D3D4E73B)",
		prefix + "LHR-A3C36EA5 C: SOB: drop S-5 (D3D4E73B) seeing S-8 (170EE067) S-16 ( 4D47FB4)",
		prefix + "LHR-3E61E6B7 C: No base stations seen...",
		prefix + "LHR-3E61E6B7 C: ----- BOOTSTRAPPED base F210FBA6 (best) distance 2.14m -----",
		prefix + "LHR-3E61E6B7 C: Trying to start tracking from base D3D4E73B: Not enough samples",
	};
}

inline std::string RequestBytes(bool handshaken, const protocol::Request &request)
{
	std::string bytes(1, handshaken ? '\x01' : '\x00');
	bytes.append(reinterpret_cast<const char *>(&request), sizeof request);
	return bytes;
}

inline std::vector<std::string> RequestSeeds()
{
	protocol::Request handshake(protocol::RequestHandshake);
	protocol::Request transform(protocol::RequestSetDeviceTransform);
	transform.setDeviceTransform = protocol::SetDeviceTransform(3, true, { { 0.5, -1.0, 2.0 } },
		{ 0.9, 0.1, -0.3, 0.3 }, 1.02, 0.01);
	protocol::Request state(protocol::RequestSetRuntimeState);
	state.setRuntimeState.enabledMask = 0x6;
	state.setRuntimeState.hiddenMask = 0x4;
	state.setRuntimeState.transform.rotation = { 0.9, 0.1, -0.3, 0.3 };
	state.setRuntimeState.field.enabled = 1;
	state.setRuntimeState.field.anchorCount = 2;
	state.setRuntimeState.field.anchors[0].position[0] = 1.5;
	state.setRuntimeState.field.anchors[0].translationDelta[2] = 0.02;
	state.setRuntimeState.field.anchors[1].rotationDelta = { 0.999, 0.0, 0.02, 0.0 };
	return { RequestBytes(false, handshake), RequestBytes(true, transform), RequestBytes(true, state) };
}

struct Target
{
	const char *name;
	std::string (*check)(const uint8_t *, size_t);
	std::vector<std::string> (*seeds)();
};

inline const std::vector<Target> &Targets()
{
	static const std::vector<Target> targets = {
		{ "profile", CheckProfileRecord, ProfileSeeds },
		{ "feed", CheckReleaseFeed, FeedSeeds },
		{ "lighthouse", CheckLighthouseLine, LighthouseSeeds },
		{ "request", CheckDriverRequest, RequestSeeds },
	};
	return targets;
}

} // namespace questcalfuzz
