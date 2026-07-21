#include "stdafx.h"
#include "PoseStreamHub.h"

PoseStreamHub::~PoseStreamHub()
{
	Stop();
}

void PoseStreamHub::Start(const char *shmemName)
{
	if (drainThread.joinable())
		return;

	history.resize(static_cast<size_t>(HistoryCapacity));
	stopRequested.store(false, std::memory_order_release);
	drainThread = std::thread(&PoseStreamHub::DrainLoop, this, std::string(shmemName));
}

void PoseStreamHub::Stop()
{
	stopRequested.store(true, std::memory_order_release);
	if (drainThread.joinable())
		drainThread.join();
}

int PoseStreamHub::CreateConsumer()
{
	std::lock_guard<std::mutex> lock(mutex);
	cursors.push_back(head);
	return static_cast<int>(cursors.size()) - 1;
}

uint64_t PoseStreamHub::Drain(int consumer, std::vector<protocol::DevicePoseSample> &out)
{
	out.clear();

	std::lock_guard<std::mutex> lock(mutex);
	if (consumer < 0 || consumer >= static_cast<int>(cursors.size()))
		return 0;

	uint64_t &cursor = cursors[consumer];
	uint64_t dropped = 0;

	uint64_t oldest = head > HistoryCapacity ? head - HistoryCapacity : 0;
	if (cursor < oldest)
	{
		dropped = oldest - cursor;
		cursor = oldest;
	}

	out.reserve(static_cast<size_t>(head - cursor));
	for (; cursor < head; ++cursor)
		out.push_back(history[cursor % HistoryCapacity]);

	return dropped;
}

void PoseStreamHub::DiscardBacklog(int consumer)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (consumer >= 0 && consumer < static_cast<int>(cursors.size()))
		cursors[consumer] = head;
}

void PoseStreamHub::DrainLoop(std::string shmemName)
{
	protocol::PoseRingReader reader;
	std::vector<protocol::DevicePoseSample> scratch;
	ULONGLONG lastOpenAttempt = 0;

	while (!stopRequested.load(std::memory_order_acquire))
	{
		if (!reader.IsOpen())
		{
			ULONGLONG now = GetTickCount64();
			if (now - lastOpenAttempt >= 1000)
			{
				lastOpenAttempt = now;
				reader.Open(shmemName.c_str());
				ringOpen.store(reader.IsOpen(), std::memory_order_release);
			}
			if (!reader.IsOpen())
			{
				Sleep(50);
				continue;
			}
		}

		scratch.clear();
		reader.Drain([&](const protocol::DevicePoseSample &s) { scratch.push_back(s); });

		if (!scratch.empty())
		{
			std::lock_guard<std::mutex> lock(mutex);
			for (const auto &s : scratch)
			{
				history[head % HistoryCapacity] = s;
				++head;
			}
		}

		Sleep(2);
	}
}
