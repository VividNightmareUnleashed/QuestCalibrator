#pragma once
#include "PoseTransform.h"
#include "AlignmentField.h"
#include "../common/NumericValidation.h"

namespace questcal { namespace driverpose {
inline bool IsUsableCalibrationPose(const vr::DriverPose_t &pose,
    const double (&world)[3], double scale)
{
    return pose.poseIsValid &&
        (pose.deviceIsConnected || pose.result == vr::TrackingResult_Running_OK) &&
        numeric::IsBoundedVector3(world, scale * protocol::limits::MaxAbsPosePositionMeters);
}

// Called after raw-ring publication, under the slot's pose mutex. Checked
// directly by the portable contracts; OS capture and hook lifetime are separate.
inline void ApplyRuntimePose(vr::DriverPose_t &pose,
    const protocol::SetDeviceTransform &cal, const protocol::FrameCorrection &frame,
    const protocol::SetAlignmentField &field, double nowSeconds,
    alignfield::EvalState &baseState, alignfield::EvalState &fieldState)
{
	if (cal.enabled)
	{
		// The raw ring was published above. Cancel only this device's frame
		// motion, before scale, base slew, and the field's spatial lookup.
		questcal::driverpose::Apply(pose, frame.rotation, frame.translation.v, 1.0, 0.0);
		vr::HmdVector3d_t scaledPosition = questcal::driverpose::Scale(pose.vecPosition, cal.scale);
		vr::HmdVector3d_t rotatedPosition = questcal::driverpose::RotateVector(
			pose.qWorldFromDriverRotation, scaledPosition.v);
		vr::HmdVector3d_t scaledOrigin = questcal::driverpose::Scale(
			pose.vecWorldFromDriverTranslation, cal.scale);
		vr::HmdVector3d_t rawWorld = questcal::driverpose::Add(rotatedPosition.v, scaledOrigin.v);
		// Standable hides physical trackers by clearing their connection flag
		// while they still publish valid Running_OK poses. Keep both calibration
		// layers current for those poses; a disconnected device that is no longer
		// tracking must still leave the smoothing state alone.
		const bool usablePosition = IsUsableCalibrationPose(pose, rawWorld.v, cal.scale);

		// Base-calibration slew or snap, by generation (see Protocol.h). A read
		// that fell back to lastGood may briefly carry an older generation; the
		// next consistent read then snaps to where it was slewing, which is benign.
		auto &bs = baseState;
		if (usablePosition)
			alignfield::SlewTowardAt(cal.rotation, cal.translation.v, rawWorld.v, nowSeconds,
				alignfield::BaseSlewLimits, cal.generation, bs);
		else if (!bs.hasCurrent)
			alignfield::SlewToward(cal.rotation, cal.translation.v, nowSeconds,
				alignfield::BaseSlewLimits, cal.generation, bs);
		// Invalid positions must not enter persistent smoothing state. Hold the
		// previous transform and clock; recovery after a long gap snaps normally.
		vr::HmdQuaternion_t baseRot = bs.rot;
		vr::HmdVector3d_t baseTrans{ { bs.trans[0], bs.trans[1], bs.trans[2] } };

		// Spatial correction field: blend the anchor deltas at this device's
		// own base-calibrated position and left-compose the result onto the
		// base calibration (world = delta(base(raw))). Blending on the
		// device's own position keeps a static tripod tracker static while
		// the user walks, and needs no cross-thread HMD position cache.
		vr::HmdQuaternion_t calRot = baseRot;
		vr::HmdVector3d_t calTrans = baseTrans;

		auto &fs = fieldState;
		if (!field.enabled || field.anchorCount == 0)
		{
			fs.hasCurrent = false;   // re-enabling later snaps
		}
		else if (usablePosition)
		{
			vr::HmdVector3d_t rotatedRawWorld =
				questcal::driverpose::RotateVector(baseRot, rawWorld.v);
			vr::HmdVector3d_t basePos =
				questcal::driverpose::Add(rotatedRawWorld.v, baseTrans.v);

			alignfield::Evaluate(field, basePos.v, nowSeconds, fs);
		}
		// Invalid pose: keep the previous delta without advancing the
		// slew clock; Evaluate's gap check snaps after a long loss.

		if (fs.hasCurrent)
		{
			calRot = questcal::driverpose::Multiply(fs.rot, baseRot);
			vr::HmdVector3d_t rotatedBase =
				questcal::driverpose::RotateVector(fs.rot, baseTrans.v);
			calTrans = questcal::driverpose::Add(rotatedBase.v, fs.trans);
		}

		// Apply the exact solver model to the composed raw-world pose, including
		// scale on worldFromDriver translation and every linear derivative.
		questcal::driverpose::Apply(
			pose, calRot, calTrans.v, cal.scale, cal.timeOffset);
	}
	else
	{
		// Re-enabling later must snap, not slew from a stale state.
		baseState.hasCurrent = false;
		fieldState.hasCurrent = false;
	}

}
} }
