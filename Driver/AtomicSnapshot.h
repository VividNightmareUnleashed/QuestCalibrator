#pragma once

#include <atomic>
#include <cstdint>

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

// Only Store and Load are ever performed on these scalars, which is exactly
// what std::atomic<double> defines - none of the float-comparison caveats that
// would justify hand-rolled bit punning apply, so the library type is the
// direct mechanism and a reader no longer has to verify a memcpy round trip
// before trusting the seqlock payload.
//
// CAVEAT: std::atomic has no value-initialising default constructor before
// C++20, so every declaration of this type - here and in the seqlock slots -
// must carry an explicit initialiser. The neutral non-zero defaults (identity
// quaternion, scale 1.0) are contract: a zeroed quaternion is degenerate, not
// neutral.
using Double = std::atomic<double>;

static_assert(std::atomic<double>::is_always_lock_free,
	"64-bit snapshot atoms must always be lock-free");
static_assert(ATOMIC_INT_LOCK_FREE == 2, "32-bit snapshot atoms must always be lock-free");

// The explicit release/acquire orders below keep the previous semantics: the
// library default is seq_cst, which would add fences to every pose-thread read.
struct Vector3
{
	Double values[3]{ { 0.0 }, { 0.0 }, { 0.0 } };

	void Store(const double (&source)[3]) noexcept
	{
		for (int i = 0; i < 3; ++i)
			values[i].store(source[i], std::memory_order_release);
	}

	void Load(double (&destination)[3]) const noexcept
	{
		for (int i = 0; i < 3; ++i)
			destination[i] = values[i].load(std::memory_order_acquire);
	}
};

struct Quaternion
{
	Double w{ 1.0 };
	Double x{ 0.0 };
	Double y{ 0.0 };
	Double z{ 0.0 };

	template <class QuaternionType>
	void Store(const QuaternionType &source) noexcept
	{
		w.store(source.w, std::memory_order_release);
		x.store(source.x, std::memory_order_release);
		y.store(source.y, std::memory_order_release);
		z.store(source.z, std::memory_order_release);
	}

	template <class QuaternionType>
	QuaternionType Load() const noexcept
	{
		return { w.load(std::memory_order_acquire), x.load(std::memory_order_acquire),
			y.load(std::memory_order_acquire), z.load(std::memory_order_acquire) };
	}
};

} // namespace atomicsnapshot
} // namespace questcal
