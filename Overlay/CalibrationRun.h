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
	std::string hmdSerial;
	bool anchor = false;
	bool usesPoseRing = false;
	bool driverNeutralized = false;
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
		referenceId = UINT32_MAX;
		targetId = UINT32_MAX;
		referenceSystem.clear();
		targetSystem.clear();
		referenceSerial.clear();
		targetSerial.clear();
		hmdSerial.clear();
		anchor = false;
		usesPoseRing = false;
		driverNeutralized = false;
		referenceUniverse = Universe();
		targetUniverse = Universe();
		referenceSamples.clear();
		targetSamples.clear();
		collectionStart = 0.0;
		lastReferenceSample = 0.0;
		lastTargetSample = 0.0;
		lastIdentityCheck = -1e9;
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
