#pragma once

#include "DriverSession.h"

#include <array>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace questcal
{

struct DriverSyncJob
{
	DriverApplyRequest request;
	std::array<SyncDevice, vr::k_unMaxTrackedDeviceCount> devices;
	double time = 0.0;
};

struct DriverSyncCompletion
{
	uint64_t sequence = 0;
	DriverApplyResult result;
	std::string error;
	bool clearError = false;
};

struct DriverSyncSubmission
{
	uint64_t sequence = 0;
	bool stateChanged = false;
};

class DriverSyncWorker
{
public:
	~DriverSyncWorker() { Stop(); }

	void Start(DriverTransport transport)
	{
		Stop();
		session.SetTransport(std::move(transport));
		stopping = false;
		worker = std::thread([this] { Run(); });
	}

	void Stop()
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			stopping = true;
		}
		wake.notify_one();
		if (worker.joinable())
			worker.join();
	}

	DriverSyncSubmission Submit(const DriverSyncJob &job)
	{
		std::lock_guard<std::mutex> lock(mutex);
		DriverSyncSubmission submission;
		submission.sequence = ++nextSequence;
		submission.stateChanged = !hasLastSubmission || !SameState(lastSubmission, job);
		lastSubmission = job;
		hasLastSubmission = true;
		pendingJob = job;
		pendingSequence = submission.sequence;
		pending = true;
		wake.notify_one();
		return submission;
	}

	bool Poll(DriverSyncCompletion &out)
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (!completed)
			return false;
		out = std::move(completion);
		completed = false;
		return true;
	}

private:
	static bool SameTransform(const DriverSyncDesired &a,
		const DriverSyncDesired &b)
	{
		return a.referenceTrackingSystem == b.referenceTrackingSystem &&
			a.targetTrackingSystem == b.targetTrackingSystem &&
			a.rotation.coeffs() == b.rotation.coeffs() &&
			a.translationMeters == b.translationMeters &&
			a.scale == b.scale && a.timeShift == b.timeShift &&
			a.baseGeneration == b.baseGeneration &&
			a.continuousArmed == b.continuousArmed &&
			a.hideMountedTracker == b.hideMountedTracker &&
			a.continuousTrackerSerial == b.continuousTrackerSerial;
	}

	static bool SameField(const protocol::SetAlignmentField &a,
		const protocol::SetAlignmentField &b)
	{
		if (a.enabled != b.enabled || a.generation != b.generation ||
			a.anchorCount != b.anchorCount || a.sigmaMeters != b.sigmaMeters)
			return false;
		for (uint32_t i = 0; i < a.anchorCount; ++i)
		{
			const auto &x = a.anchors[i];
			const auto &y = b.anchors[i];
			for (int axis = 0; axis < 3; ++axis)
			{
				if (x.position[axis] != y.position[axis] ||
					x.translationDelta[axis] != y.translationDelta[axis])
					return false;
			}
			if (x.rotationDelta.w != y.rotationDelta.w ||
				x.rotationDelta.x != y.rotationDelta.x ||
				x.rotationDelta.y != y.rotationDelta.y ||
				x.rotationDelta.z != y.rotationDelta.z)
				return false;
		}
		return true;
	}

	static bool SameDevice(const SyncDevice &a, const SyncDevice &b)
	{
		return a.id == b.id && a.deviceClass == b.deviceClass &&
			a.trackingSystemKnown == b.trackingSystemKnown &&
			a.trackingSystem == b.trackingSystem &&
			a.serialKnown == b.serialKnown && a.serial == b.serial;
	}

	static bool SameState(const DriverSyncJob &a, const DriverSyncJob &b)
	{
		if (a.request.enabled != b.request.enabled ||
			!SameTransform(a.request.desired, b.request.desired) ||
			!SameField(a.request.field, b.request.field))
			return false;
		for (size_t i = 0; i < a.devices.size(); ++i)
			if (!SameDevice(a.devices[i], b.devices[i]))
				return false;
		return true;
	}

	void Run()
	{
		for (;;)
		{
			DriverSyncJob job;
			uint64_t sequence = 0;
			{
				std::unique_lock<std::mutex> lock(mutex);
				wake.wait(lock, [this] { return stopping || pending; });
				if (stopping && !pending)
					return;
				job = std::move(pendingJob);
				sequence = pendingSequence;
				pending = false;
			}

			DriverSyncCompletion done;
			done.sequence = sequence;
			session.SetDeviceEnumerator([&job](uint32_t id, const DriverSyncDesired &)
			{
				return id < job.devices.size() ? job.devices[id] : SyncDevice();
			});
			session.SetErrorSink(
				[&done](const std::string &message) { done.error = message; },
				[&done] { done.clearError = true; });
			DriverBatch batch = session.Begin(job.time);
			done.result = session.Apply(batch, job.request, job.time);

			{
				std::lock_guard<std::mutex> lock(mutex);
				completion = std::move(done);
				completed = true;
			}
		}
	}

	DriverSession session;
	std::thread worker;
	std::mutex mutex;
	std::condition_variable wake;
	bool stopping = false;
	bool pending = false;
	bool completed = false;
	bool hasLastSubmission = false;
	uint64_t nextSequence = 0;
	uint64_t pendingSequence = 0;
	DriverSyncJob pendingJob;
	DriverSyncJob lastSubmission;
	DriverSyncCompletion completion;
};

} // namespace questcal
