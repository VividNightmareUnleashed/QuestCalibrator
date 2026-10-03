#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

namespace questcal {
namespace hooks {

// Counts the callbacks inside code that teardown is about to release, so it
// can wait for them to leave.
class CallbackActivity
{
public:
	class Guard
	{
	public:
		explicit Guard(CallbackActivity &owner) : owner(owner)
		{ owner.count.fetch_add(1, std::memory_order_seq_cst); }
		~Guard() { owner.count.fetch_sub(1, std::memory_order_seq_cst); }
		Guard(const Guard &) = delete;
		Guard &operator=(const Guard &) = delete;
	private:
		CallbackActivity &owner;
	};
	uint32_t Count() const { return count.load(std::memory_order_seq_cst); }

	// True once no callback is counted; false if one still is after `timeout`.
	bool WaitUntilIdle(std::chrono::milliseconds timeout) const
	{
		const auto deadline = std::chrono::steady_clock::now() + timeout;
		while (Count() != 0)
		{
			if (std::chrono::steady_clock::now() >= deadline)
				return false;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		return true;
	}
private:
	// Simultaneous admitted callbacks must fit uint32_t, as they do in the
	// supported process/thread domain.
	std::atomic<uint32_t> count{0};
};

} // namespace hooks
} // namespace questcal
