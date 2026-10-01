#pragma once
#include <cstdint>

namespace questcal { namespace update {
// The owner holds its mutex while reading these fields and, for a handoff,
// through process creation. uint64 revision must not wrap while work is live.
inline bool CurrentRevision(bool enabled, bool stopping,
	uint64_t current, uint64_t offered)
{
	return enabled && !stopping && current == offered;
}
template<class Launch>
bool LaunchCurrentRevision(bool enabled, bool stopping, uint64_t current,
	uint64_t offered, bool ready, Launch launch)
{
	return ready && CurrentRevision(enabled, stopping, current, offered) && launch();
}
} }
