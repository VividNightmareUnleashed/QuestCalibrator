#pragma once
#include <cstdint>

namespace questcal {
// Shared by presentation and command entry points. The system-name validator
// is a separate input contract; this gate protects the active run's ownership.
inline bool MayStartManualRun(bool idle, uint32_t referenceId, uint32_t targetId,
	uint32_t deviceLimit)
{
	return idle && referenceId < deviceLimit && targetId < deviceLimit &&
		referenceId != targetId;
}
inline bool MayStartAnchorRun(bool validProfile, bool universeUnsafe, bool sameSystems)
{
	return validProfile && !universeUnsafe && sameSystems;
}
}
