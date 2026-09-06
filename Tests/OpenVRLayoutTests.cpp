#include "../Driver/OpenVRHookLayout.h"
#include "../lib/openvr/compat/ivrserverdriverhost_005.h"

#include <cstddef>
#include <cstring>
#include <type_traits>

namespace
{
using LegacyPose = openvr_1_10_30::DriverPose_t;
static_assert(sizeof(vr::HmdQuaternion_t) == 4 * sizeof(double));
static_assert(sizeof(vr::DriverPose_t) == 280); // OpenVR 1.10.30 Win64 layout.
static_assert(std::is_standard_layout_v<vr::DriverPose_t>);
static_assert(std::is_standard_layout_v<LegacyPose>);
static_assert(sizeof(vr::DriverPose_t) == sizeof(LegacyPose));
static_assert(alignof(vr::DriverPose_t) == alignof(LegacyPose));
#define CHECK_POSE_FIELD(field) \
	static_assert(offsetof(vr::DriverPose_t, field) == offsetof(LegacyPose, field)); \
	static_assert(std::is_same_v<decltype(vr::DriverPose_t::field), decltype(LegacyPose::field)>);
CHECK_POSE_FIELD(poseTimeOffset)
CHECK_POSE_FIELD(qWorldFromDriverRotation)
CHECK_POSE_FIELD(vecWorldFromDriverTranslation)
CHECK_POSE_FIELD(qDriverFromHeadRotation)
CHECK_POSE_FIELD(vecDriverFromHeadTranslation)
CHECK_POSE_FIELD(vecPosition)
CHECK_POSE_FIELD(vecVelocity)
CHECK_POSE_FIELD(vecAcceleration)
CHECK_POSE_FIELD(qRotation)
CHECK_POSE_FIELD(vecAngularVelocity)
CHECK_POSE_FIELD(vecAngularAcceleration)
CHECK_POSE_FIELD(result)
CHECK_POSE_FIELD(poseIsValid)
CHECK_POSE_FIELD(willDriftInYaw)
CHECK_POSE_FIELD(shouldApplyHeadModel)
CHECK_POSE_FIELD(deviceIsConnected)
#undef CHECK_POSE_FIELD

template<class Interface, class Pose>
struct Host final : Interface
{
	bool received = false;
	bool TrackedDeviceAdded(const char *, vr::ETrackedDeviceClass, vr::ITrackedDeviceServerDriver *) override { return false; }
	void TrackedDevicePoseUpdated(uint32_t device, const Pose &pose, uint32_t size) override
	{
		received = device == 17 && size == sizeof(Pose) && pose.vecPosition[0] == 1.25 && pose.deviceIsConnected;
	}
	void VsyncEvent(double) override {}
	void VendorSpecificEvent(uint32_t, vr::EVREventType, const vr::VREvent_Data_t &, double) override {}
	bool IsExiting() override { return false; }
	bool PollNextEvent(vr::VREvent_t *, uint32_t) override { return false; }
	void GetRawTrackedDevicePoses(float, vr::TrackedDevicePose_t *, uint32_t) override {}
	void RequestRestart(const char *, const char *, const char *, const char *) override {}
	uint32_t GetFrameTimings(vr::Compositor_FrameTiming *, uint32_t) override { return 0; }
	// The display methods differ between _005 and _006; neither precedes the pose callback.
	void TrackedDeviceDisplayTransformUpdated(uint32_t, vr::HmdMatrix34_t, vr::HmdMatrix34_t) {}
	void SetDisplayEyeToHead(uint32_t, const vr::HmdMatrix34_t &, const vr::HmdMatrix34_t &) {}
	void SetDisplayProjectionRaw(uint32_t, const vr::HmdRect2_t &, const vr::HmdRect2_t &) {}
	void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) {}
};

template<class Interface, class Pose>
bool DispatchPose()
{
	Host<Interface, Pose> host;
	Pose pose{};
	pose.vecPosition[0] = 1.25;
	pose.deviceIsConnected = true;
	// Exercise the same MSVC x64 vtable access used by the driver hook.
	using Callback = void (*)(void *, uint32_t, const Pose &, uint32_t);
	auto table = *reinterpret_cast<void ***>(&host);
	auto callback = reinterpret_cast<Callback>(table[openvr_hook::PoseUpdateSlot]);
	callback(&host, 17, pose, sizeof(pose));
	return host.received;
}

struct Context final : vr::IVRDriverContext
{
	void *GetGenericInterface(const char *version, vr::EVRInitError *error) override
	{
		*error = vr::VRInitError_None;
		return std::strcmp(version, "IVRServerDriverHost_006") == 0 ? this : nullptr;
	}
	vr::DriverHandle_t GetDriverHandle() override { return 0; }
};
}

void RunOpenVRLayoutScenarios(void (*check)(const char *, bool, const char *))
{
	check("OpenVR: _005 pose ABI", DispatchPose<openvr_1_10_30::IVRServerDriverHost, LegacyPose>(), "");
	check("OpenVR: _006 pose ABI", DispatchPose<vr::IVRServerDriverHost, vr::DriverPose_t>(), "");
	Context context;
	auto table = *reinterpret_cast<void ***>(&context);
	using Callback = void *(*)(void *, const char *, vr::EVRInitError *);
	auto callback = reinterpret_cast<Callback>(table[openvr_hook::GetGenericInterfaceSlot]);
	vr::EVRInitError error = vr::VRInitError_Unknown;
	check("OpenVR: context ABI", callback(&context, vr::IVRServerDriverHost_Version, &error) == &context &&
		error == vr::VRInitError_None, "");
}
