#pragma once

#include "DriverSession.h"

#include <array>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace questcal
{

struct DriverStateJob
{
	DriverApplyRequest request;
	std::array<SyncDevice, vr::k_unMaxTrackedDeviceCount> devices;
	double time = 0.0;

	bool operator==(const DriverStateJob &other) const
	{
		return request == other.request && devices == other.devices;
	}
};

enum class DriverWorkKind
{
	Synchronize,
	Neutralize,
};

struct DriverCompletion
{
	DriverWorkKind kind = DriverWorkKind::Synchronize;
	uint64_t sequence = 0;
	DriverApplyResult result;
	std::string error;
	bool clearError = false;
	bool succeeded = false;
};

struct DriverStateSubmission
{
	uint64_t sequence = 0;
	bool stateChanged = false;
};

class DriverWorker
{
public:
	~DriverWorker() { Stop(); }

	void Start(DriverTransport transport)
	{
		Stop();
		std::lock_guard<std::mutex> lock(mutex);
		session.SetTransport(std::move(transport));
		pendingState.reset();
		pendingNeutralization.reset();
		lastSubmission.reset();
		completions.clear();
		nextSequence = 0;
		stopping = false;
		worker = std::thread([this] { Run(); });
	}

	void Stop()
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			stopping = true;
			neutralizationHeld = false;
		}
		wake.notify_one();
		if (worker.joinable())
			worker.join();
	}

	DriverStateSubmission Submit(const DriverStateJob &job)
	{
		std::lock_guard<std::mutex> lock(mutex);
		DriverStateSubmission submission;
		submission.sequence = ++nextSequence;
		submission.stateChanged = !lastSubmission || !(*lastSubmission == job);
		lastSubmission = job;
		pendingState = SequencedState{ submission.sequence, job };
		wake.notify_one();
		return submission;
	}

	uint64_t Neutralize(const std::array<uint32_t, 2> &deviceIds, double time)
	{
		std::lock_guard<std::mutex> lock(mutex);
		const uint64_t sequence = ++nextSequence;
		neutralizationHeld = true;
		pendingNeutralization = Neutralization{ sequence, deviceIds, time };
		wake.notify_one();
		return sequence;
	}

	void ReleaseNeutralization()
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			neutralizationHeld = false;
		}
		wake.notify_one();
	}

	bool Poll(DriverCompletion &out)
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (completions.empty())
			return false;
		out = std::move(completions.front());
		completions.pop_front();
		return true;
	}

private:
	struct SequencedState
	{
		uint64_t sequence = 0;
		DriverStateJob job;
	};

	struct Neutralization
	{
		uint64_t sequence = 0;
		std::array<uint32_t, 2> deviceIds;
		double time = 0.0;
	};

	void Run()
	{
		for (;;)
		{
			std::optional<SequencedState> state;
			std::optional<Neutralization> neutralization;
			{
				std::unique_lock<std::mutex> lock(mutex);
				wake.wait(lock, [this]
				{
					return stopping || pendingNeutralization ||
						(pendingState && !neutralizationHeld);
				});

				if (pendingNeutralization)
				{
					neutralization = std::move(pendingNeutralization);
					pendingNeutralization.reset();
				}
				else if (pendingState && !neutralizationHeld)
				{
					state = std::move(pendingState);
					pendingState.reset();
				}
				else if (stopping)
				{
					return;
				}
			}

			DriverCompletion done;
			session.SetErrorSink(
				[&done](const std::string &message) { done.error = message; },
				[&done] { done.clearError = true; });

			if (neutralization)
			{
				done.kind = DriverWorkKind::Neutralize;
				done.sequence = neutralization->sequence;
				done.succeeded = true;
				for (uint32_t id : neutralization->deviceIds)
				{
					if (!session.DisableDeviceTransform(id, neutralization->time))
					{
						done.succeeded = false;
						break;
					}
				}
			}
			else
			{
				done.kind = DriverWorkKind::Synchronize;
				done.sequence = state->sequence;
				auto &job = state->job;
				session.SetDeviceEnumerator([&job](uint32_t id, const DriverSyncDesired &)
				{
					return job.devices[id];
				});
				done.result = session.Apply(job.request, job.time);
				done.succeeded = done.result.synchronized;
			}

			{
				std::lock_guard<std::mutex> lock(mutex);
				completions.push_back(std::move(done));
			}
		}
	}

	DriverSession session;
	std::thread worker;
	std::mutex mutex;
	std::condition_variable wake;
	bool stopping = false;
	bool neutralizationHeld = false;
	uint64_t nextSequence = 0;
	std::optional<SequencedState> pendingState;
	std::optional<Neutralization> pendingNeutralization;
	std::optional<DriverStateJob> lastSubmission;
	std::deque<DriverCompletion> completions;
};

} // namespace questcal
