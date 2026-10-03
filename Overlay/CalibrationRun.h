#pragma once

#include "CalibrationEngine.h"
#include "ChaperoneMath.h"
#include "LocalPoseContinuity.h"

#include <cstdint>
#include <string>
#include <vector>

namespace questcal
{

struct CalibrationRun
{
	struct Universe
	{
		bool valid = false;
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };

		bool Accept(const Eigen::Quaterniond &sampleRotation,
			const Eigen::Vector3d &sampleTranslation)
		{
			if (!valid)
			{
				rotation = sampleRotation;
				translation = sampleTranslation;
				valid = true;
				return true;
			}
			return !WorldFromDriverChanged(
				rotation, translation, sampleRotation, sampleTranslation);
		}
	};

	// The target's lighthouse frame while collecting. SteamVR re-solving the
	// base station a tracker is reported against moves every pose in that
	// frame by one exact delta while the local pose stays on its trajectory
	// (LighthouseFrameWatch.h); on 2026-09-26 it did so every minute or two
	// for half an hour (6 to 12 cm each), and each such move stopped a run as
	// a tracking reset. Now the samples after a move are put back in the frame
	// the run began in (ToStart). A fresh solve is carried to the ending frame
	// (CarryCalibration); a live-profile solve is converted to its existing
	// normalized space instead. A frame change the pose
	// jumped with still stops the run; one that left the pose where it was
	// (the same pose in another station's frame) changes nothing.
	struct TargetFrame
	{
		enum class Verdict { Same, Moved, Changed };

		bool valid = false;
		ringpose::DriverLocalPoseSample local;   // the last sample's driver-local pose
		Eigen::Quaterniond wfdRot{ 1, 0, 0, 0 };
		Eigen::Vector3d wfdTrans{ 0, 0, 0 };
		// From the current frame to the one the run began in.
		Eigen::Quaterniond toStartRot{ 1, 0, 0, 0 };
		Eigen::Vector3d toStartTrans{ 0, 0, 0 };
		int moves = 0;

		Verdict Accept(const ringpose::DriverLocalPoseSample &sample,
			const Eigen::Quaterniond &sampleWfdRot, const Eigen::Vector3d &sampleWfdTrans)
		{
			Verdict verdict = Verdict::Same;
			if (valid && WorldFromDriverChanged(wfdRot, wfdTrans, sampleWfdRot, sampleWfdTrans))
			{
				const bool worldKept = Kept(
					ringpose::ComposeWithWorldFromDriver(local, wfdRot, wfdTrans),
					ringpose::ComposeWithWorldFromDriver(sample, sampleWfdRot, sampleWfdTrans));
				if (!worldKept)
				{
					if (!Kept(local, sample))
						return Verdict::Changed;
					// F = new o old^-1, and toStart becomes toStart o F^-1.
					const Eigen::Quaterniond fRot = (sampleWfdRot * wfdRot.conjugate()).normalized();
					const Eigen::Vector3d fTrans = sampleWfdTrans - fRot * wfdTrans;
					const Eigen::Quaterniond fInv = fRot.conjugate();
					toStartTrans -= toStartRot * (fInv * fTrans);
					toStartRot = (toStartRot * fInv).normalized();
					++moves;
					verdict = Verdict::Moved;
				}
			}
			valid = true;
			local = sample;
			wfdRot = sampleWfdRot;
			wfdTrans = sampleWfdTrans;
			return verdict;
		}

		// A composed (world) pose, put back in the frame the run began in.
		void ToStart(PoseSample &s) const
		{
			s.rot = (toStartRot * s.rot).normalized();
			s.pos = toStartRot * s.pos + toStartTrans;
			s.vel = toStartRot * s.vel;
			s.angVel = toStartRot * s.angVel;
		}

		// A calibration C(x) = R (s x) + t solved in the start frame, as it
		// maps the current one: C o toStart.
		void CarryCalibration(Eigen::Quaterniond &rotation, Eigen::Vector3d &translation, double scale) const
		{
			translation += scale * (rotation * toStartTrans);
			rotation = (rotation * toStartRot).normalized();
		}

	private:
		// On its trajectory, or unchanged for a device at rest, as
		// LighthouseFrameWatch holds it.
		static bool Kept(const ringpose::DriverLocalPoseSample &was, const ringpose::DriverLocalPoseSample &is)
		{
			return ringpose::IsDriverLocalPoseContinuous(was, is) ||
				((is.position - was.position).norm() <= ringpose::RestPositionMeters &&
					is.rotation.angularDistance(was.rotation) <= ringpose::RestRotationRadians);
		}
	};

	uint32_t referenceId = UINT32_MAX;
	uint32_t targetId = UINT32_MAX;
	std::string referenceSystem;
	std::string targetSystem;
	std::string referenceSerial;
	std::string targetSerial;
	// Model names, read once at Begin: the player knows "VIVE Tracker 3.0",
	// not "the target device", so every stop reason names the hardware.
	std::string referenceModel;
	std::string targetModel;
	std::string hmdSerial;
	bool anchor = false;
	bool usesPoseRing = false;
	// A same-active-profile recalibration changes C, retaining each device's N.
	// Captured at the first accepted target sample, in the solve's start frame.
	bool preserveTrackerFrames = false;
	bool normalizationCaptured = false;
	Eigen::Quaterniond targetNormalizationRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d targetNormalizationTranslation{ 0, 0, 0 };
	// Raw collection: the hub's session-boundary count when it began, and the
	// short source gaps it rode through.
	uint64_t streamBoundariesAtStart = 0;
	uint64_t toleratedLoss = 0;
	uint64_t toleratedGaps = 0;
	uint64_t neutralizationSequence = 0;
	// Each device's live lighthouse restart count when collection began
	// (LighthouseVisibility::Device::liveRestarts; 0 for other systems).
	uint32_t referenceRestartsAtStart = 0;
	uint32_t targetRestartsAtStart = 0;
	Universe referenceUniverse;
	Universe targetUniverse;
	TargetFrame targetFrame;
	// Begin waits for the pair to be measurable (see CalibrationTick): since
	// when, and what the guide shows meanwhile. Empty while not waiting.
	double waitStart = -1.0;
	std::string waitInstruction;
	std::string waitNote;
	std::vector<PoseSample> referenceSamples;
	std::vector<PoseSample> targetSamples;
	double collectionStart = 0.0;
	double lastReferenceSample = 0.0;
	double lastTargetSample = 0.0;
	double lastIdentityCheck = -1e9;

    template<class Release> bool End(Release release)
    {
        const bool held = neutralizationSequence != 0;
        if (held) release();
        Reset();
        return held;
    }

	void Reset()
	{
		*this = CalibrationRun{};
	}

	bool AcceptUniverse(uint32_t deviceId, const Eigen::Quaterniond &rotation,
		const Eigen::Vector3d &translation)
	{
		if (deviceId == referenceId)
			return referenceUniverse.Accept(rotation, translation);
		if (deviceId == targetId)
			return targetUniverse.Accept(rotation, translation);
		return true;
	}
};

} // namespace questcal
