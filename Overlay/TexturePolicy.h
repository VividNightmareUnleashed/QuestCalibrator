#pragma once
#include <cstdint>

namespace questcal {
struct TextureAllocation { uint32_t stride = 0, bytes = 0; };
inline bool PlanTextureAllocation(uint32_t width, uint32_t height, bool guide,
	TextureAllocation &out)
{
	const bool dimensions = guide
		? ((width == 4608 && height == 5880) || (width == 5120 && height == 5760))
		: (width > 0 && height > 0 && width <= 1024 && height <= 1024);
	if (!dimensions) return false;
	// Only the validated dimensions reach multiplication. The largest atlas
	// is 112.5 MiB; both stride and bytes fit WIC's uint32 API parameters.
	out = {width * 4, width * height * 4};
	return true;
}
}
