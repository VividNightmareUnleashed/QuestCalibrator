#pragma once

namespace openvr_hook
{
// Verified against the vendored 2.15.6 _006 declaration and the preserved
// 1.10.30 _005 declaration. Tests dispatch through these slots on both layouts.
inline constexpr int PoseUpdateSlot = 1;
inline constexpr int GetGenericInterfaceSlot = 0;
}
