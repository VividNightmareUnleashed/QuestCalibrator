#pragma once

#include <openvr.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

struct CalibrationContext;

namespace questcal
{

struct DriverNeutralizationResult
{
	uint64_t sequence = 0;
	bool succeeded = false;
};

void StartCalibrationDriver();
void StopCalibrationDriver();
void SynchronizeCalibrationDriver(CalibrationContext &ctx);
void PollCalibrationDriver(CalibrationContext &ctx);
std::optional<DriverNeutralizationResult> TakeCalibrationNeutralization(
	uint64_t sequence);
uint64_t NeutralizeCalibrationDevices(
	const std::array<uint32_t, 2> &deviceIds, double time);
void ReleaseCalibrationDeviceNeutralization();

bool ReadTrackedDeviceString(uint32_t id,
	vr::ETrackedDeviceProperty property, std::string &value);
bool ReadCurrentHmdIdentity(std::string &trackingSystem, std::string &serial);

} // namespace questcal
