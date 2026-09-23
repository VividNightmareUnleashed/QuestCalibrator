#include "../Driver/InterfaceHookInjector.h"
#include "../Driver/Logging.h"
#include "../Driver/OpenVRHookLayout.h"
#include "../Driver/ServerTrackedDeviceProvider.h"
#include "../common/Protocol.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

// The real detours, installed with MinHook on stand-ins for vrserver's driver
// context and host. DisableHooks suspends every other thread in this process
// while it walks their stacks, and it only proves quiescence once no other
// thread has a frame in this executable, so these scenarios must not run
// while another test thread is alive.
namespace
{
using Check = void (*)(const char *, bool, const char *);

std::atomic<int> ContextCalls{ 0 };
std::atomic<int> HostPoseCalls{ 0 };
std::atomic<double> HostReceivedOriginX{ 0.0 };

// Distinct bodies, so identical-code folding cannot merge a hooked target with
// an unrelated function.
struct FakeHost
{
	virtual __declspec(noinline) void Unused(uint32_t device)
	{
		HostPoseCalls.fetch_sub(static_cast<int>(device) + 7);
	}
	virtual __declspec(noinline) void TrackedDevicePoseUpdated(uint32_t device,
		const vr::DriverPose_t &pose, uint32_t size)
	{
		HostPoseCalls.fetch_add(static_cast<int>(device + size) | 1);
		HostReceivedOriginX.store(pose.vecWorldFromDriverTranslation[0]);
	}
};

struct FakeContext
{
	virtual __declspec(noinline) void *GetGenericInterface(const char *version,
		vr::EVRInitError *error);
};

FakeHost Host;
FakeContext Context;

void *FakeContext::GetGenericInterface(const char *version, vr::EVRInitError *error)
{
	ContextCalls.fetch_add(1);
	if (error)
		*error = vr::VRInitError_None;
	return std::strcmp(version, "IVRServerDriverHost_006") == 0 ? &Host : nullptr;
}

// Through the vtable slot, as InitServerDriverContext and other drivers do; a
// direct call could be devirtualised past the patched function.
void RequestHostInterface()
{
	using Fn = void *(*)(void *, const char *, vr::EVRInitError *);
	void **vtable = *reinterpret_cast<void ***>(&Context);
	auto fn = reinterpret_cast<Fn>(vtable[openvr_hook::GetGenericInterfaceSlot]);
	vr::EVRInitError error = vr::VRInitError_None;
	fn(&Context, "IVRServerDriverHost_006", &error);
}

// A tracking pose for device 0 through the host's vtable slot, as a device
// driver sends it; returns the world-from-driver origin the host received.
double SendPoseThroughHost()
{
	using Fn = void (*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t);
	vr::DriverPose_t pose{};
	pose.poseIsValid = pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.qRotation.w = pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = 1;
	void **vtable = *reinterpret_cast<void ***>(&Host);
	auto fn = reinterpret_cast<Fn>(vtable[openvr_hook::PoseUpdateSlot]);
	HostReceivedOriginX.store(0.0);
	fn(&Host, 0, pose, sizeof pose);
	return HostReceivedOriginX.load();
}

// Moves device 0's world origin, so a pose that went through the detour
// reaches the host with a non-zero origin.
bool CalibrateDeviceZero(ServerTrackedDeviceProvider &provider)
{
	protocol::SetRuntimeState state;
	state.enabledMask = 1;
	state.transform.translation.v[0] = 0.2;
	state.field.enabled = 1;
	state.field.anchorCount = 1;
	state.field.anchors[0].position[0] = 0.2;
	state.field.anchors[0].translationDelta[0] = -0.1;
	state.transform.generation = state.field.generation = 1;
	return provider.TrySetRuntimeState(state);
}

std::atomic<bool> ParkArmed{ false };
std::atomic<bool> Parked{ false };

// Holds a TryInstallPoseHook call past its accept check, with the setup mutex
// held, until teardown has cleared the ready flag or half a second passes.
void ParkUntilReadyCleared()
{
	if (!ParkArmed.exchange(false))
		return;
	Parked.store(true);
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
	while (PoseUpdateHookMask() != 0 && std::chrono::steady_clock::now() < deadline)
		std::this_thread::yield();
}

// A detour that has passed its accept check when teardown begins must not
// leave a ready flag behind: the next Init in this process would then skip
// installing the pose hook and still report it installed.
void HookReadyFlagTeardownRaceScenario(Check check)
{
	const char *name = "hooks: teardown leaves no stale ready flag";
	if (!LogFile)
		LogFile = stderr;   // the LOG macro writes unconditionally
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	auto *context = reinterpret_cast<vr::IVRDriverContext *>(&Context);
	const bool calibrated = CalibrateDeviceZero(*provider);
	if (!InjectHooks(provider.get(), context))
	{
		check(name, false, "InjectHooks failed");
		return;
	}
	RequestHostInterface();
	const uint32_t installed = PoseUpdateHookMask();
	const double firstOrigin = SendPoseThroughHost();

	Parked.store(false);
	ParkArmed.store(true);
	TryInstallAfterAcceptCheckForTest = &ParkUntilReadyCleared;
	std::thread request(&RequestHostInterface);
	while (!Parked.load())
		std::this_thread::yield();
	const bool quiesced = DisableHooks();
	request.join();
	TryInstallAfterAcceptCheckForTest = nullptr;
	const uint32_t after = PoseUpdateHookMask();

	char detail[160];
	std::snprintf(detail, sizeof detail,
		"mask after install %u (host origin x %.3f), after teardown %u, quiesced %d",
		installed, firstOrigin, after, quiesced ? 1 : 0);
	check(name, calibrated && installed == protocol::PoseHook006 && firstOrigin != 0.0 &&
		after == 0 && quiesced, detail);
	if (!quiesced)
		return;   // the module stays pinned with its hooks disabled

	// The next Init in the same process installs the hook afresh: a pose sent
	// through the host reaches it calibrated.
	const bool reinjected = InjectHooks(provider.get(), context);
	if (reinjected)
		RequestHostInterface();
	const uint32_t again = PoseUpdateHookMask();
	const double origin = reinjected ? SendPoseThroughHost() : 0.0;
	const bool cleaned = !reinjected || DisableHooks();
	std::snprintf(detail, sizeof detail,
		"reinjected %d, mask %u, host origin x %.3f, torn down %d",
		reinjected ? 1 : 0, again, origin, cleaned ? 1 : 0);
	check("hooks: a second Init installs the pose hook again",
		reinjected && again == protocol::PoseHook006 &&
		origin != 0.0 && cleaned && PoseUpdateHookMask() == 0, detail);
}
} // namespace

void RunHookInjectorScenarios(Check check)
{
	HookReadyFlagTeardownRaceScenario(check);
}
