#pragma once

#include <stdint.h>

// Compare bounded queue tickets modulo 2^64 without implementation-defined
// conversion to a signed integer. Live tickets must remain less than 2^63 apart.
static inline int questcal_pose_sequence_relation(uint64_t sequence, uint64_t expected)
{
	const uint64_t distance = sequence - expected;
	return distance == 0 ? 0 : ((distance >> 63) != 0 ? -1 : 1);
}

static inline uint64_t questcal_pose_pending_drop_count(uint64_t word)
{
	return word & UINT64_C(0xffffffff);
}

static inline uint64_t questcal_pose_after_drop_harvest(uint64_t word)
{
	return ((word >> 32) + 1) << 32;
}
