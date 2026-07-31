#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

// Scalar building blocks for coherent snapshots shared between the IPC thread
// and vrserver's pose threads. The surrounding sequence counter still detects
// mixed generations, but every payload access is itself atomic, so a rejected
// snapshot is not a formal C++ data race. QuestCalibrator only builds an x64
// driver; fail the build rather than quietly introduce a blocking atomic on a
// pose thread if that platform contract ever changes.
namespace questcal
{
namespace atomicsnapshot
{

static_assert(sizeof(uint64_t) == sizeof(double), "double snapshot requires 64-bit storage");
static_assert(ATOMIC_LLONG_LOCK_FREE == 2, "64-bit snapshot atoms must always be lock-free");
static_assert(ATOMIC_INT_LOCK_FREE == 2, "32-bit snapshot atoms must always be lock-free");

class Double
{
public:
	explicit Double(double initial = 0.0) noexcept : bits(ToBits(initial)) { }

	void Store(double value, std::memory_order order = std::memory_order_release) noexcept
	{
		bits.store(ToBits(value), order);
	}

	double Load(std::memory_order order = std::memory_order_acquire) const noexcept
	{
		return FromBits(bits.load(order));
	}

private:
	static uint64_t ToBits(double value) noexcept
	{
		uint64_t result;
		std::memcpy(&result, &value, sizeof(result));
		return result;
	}

	static double FromBits(uint64_t value) noexcept
	{
		double result;
		std::memcpy(&result, &value, sizeof(result));
		return result;
	}

	std::atomic<uint64_t> bits;
};

struct Vector3
{
	Double values[3];

	void Store(const double (&source)[3]) noexcept
	{
		for (int i = 0; i < 3; ++i)
			values[i].Store(source[i]);
	}

	void Load(double (&destination)[3]) const noexcept
	{
		for (int i = 0; i < 3; ++i)
			destination[i] = values[i].Load();
	}
};

struct Quaternion
{
	Double w{ 1.0 };
	Double x;
	Double y;
	Double z;

	template <class QuaternionType>
	void Store(const QuaternionType &source) noexcept
	{
		w.Store(source.w);
		x.Store(source.x);
		y.Store(source.y);
		z.Store(source.z);
	}

	template <class QuaternionType>
	QuaternionType Load() const noexcept
	{
		return { w.Load(), x.Load(), y.Load(), z.Load() };
	}
};

} // namespace atomicsnapshot
} // namespace questcal
