#pragma once
#include <cmath>
#include <cstddef>
namespace questcal {
constexpr size_t MaxRecordBytes = 16u * 1024u * 1024u;
constexpr size_t MaxChaperoneQuads = 16384;
inline bool IsValidRecordByteCount(size_t bytes) { return bytes <= MaxRecordBytes; }
inline bool IsValidJsonDepth(int depth) { return depth <= 16; }
inline bool IsValidChaperoneGeometryLength(size_t floats) { return floats % 12 == 0 && floats / 12 <= MaxChaperoneQuads; }
inline bool IsValidSettingsVersion(double version) { return std::isfinite(version) && version >= 1. && version <= 100. && std::floor(version) == version; }
}
