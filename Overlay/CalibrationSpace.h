#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

class PoseStreamHub;
struct CalibrationContext;
namespace protocol { struct DevicePoseSample; }

namespace questcal
{

void StartCalibrationSpace(PoseStreamHub &poseHub, double qpcToSeconds);
void StopCalibrationSpace();

// Maintains the persisted profile universe and protected chaperone from the
// dedicated HMD raw-pose stream.
void CalibrationSpaceTick(CalibrationContext &ctx, double now);
void CheckProtectedChaperone(CalibrationContext &ctx);

// Universe-jump observation is fed from the shared runtime-monitor stream so
// its accepted deltas land before drift and continuous-calibration decisions.
void ObserveUniversePose(const protocol::DevicePoseSample &sample);
void ResetUniverseObservations(CalibrationContext &ctx);
bool FinishUniverseObservations(CalibrationContext &ctx, double now);

// Applies a reference-space delta to every profile-owned spatial value.
bool ApplyCalibrationDelta(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation,
	bool snap, double now);

// Rebinds physical-HMD and raw-universe ownership after a successful base
// calibration and disarms a room snapshot that no longer belongs to it.
void RebindCalibrationUniverse(CalibrationContext &ctx,
	const std::string &hmdSerial, bool priorUniverseUnsafe);

} // namespace questcal
