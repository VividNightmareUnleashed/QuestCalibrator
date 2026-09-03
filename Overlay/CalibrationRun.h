#pragma once

#include "CalibrationEngine.h"
#include "ChaperoneMath.h"

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
	uint64_t neutralizationSequence = 0;
	Universe referenceUniverse;
	Universe targetUniverse;
	std::vector<PoseSample> referenceSamples;
	std::vector<PoseSample> targetSamples;
	double collectionStart = 0.0;
	double lastReferenceSample = 0.0;
	double lastTargetSample = 0.0;
	double lastIdentityCheck = -1e9;

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
