#pragma once

#include <atomic>
#include <cstdint>

namespace questcal {
namespace hooks {

struct ModuleRange
{
	uintptr_t begin = 0, end = 0;
	bool IsValid() const { return begin != 0 && end > begin; }
	bool Contains(uintptr_t address) const { return address >= begin && address < end; }
};

// This decision requires the caller's targets to have been disabled. The OS
// stack scan covers the entry/epilogue windows outside the counted C++ scope.
inline bool CanRelease(bool inspected, bool referencesModule, uint32_t active)
{
	return inspected && !referencesModule && active == 0;
}

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
private:
	// Simultaneous admitted callbacks must fit uint32_t, as they do in the
	// supported process/thread domain. Module lifetime outlives every Guard.
	std::atomic<uint32_t> count{0};
};

} // namespace hooks
} // namespace questcal
