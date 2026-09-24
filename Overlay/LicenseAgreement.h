#pragma once

// The end user license agreement (LICENSE at the repository root), embedded
// in the executable so the overlay can show it and ask for agreement when no
// installer did: a manual install, or terms that changed since the player
// last agreed.
//
// Agreement is recorded as the SHA-256 of the exact LICENSE bytes, the same
// hash Install.ps1 takes with Get-FileHash. The installer records it under
// HKLM\Software\QuestCalibrator\License; the overlay, which isn't elevated,
// under HKCU\Software\Classes\Local Settings\Software\QuestCalibrator\License.
// Either one matching the embedded text counts.

#include <string>

namespace questcal::license
{

// The agreement as the player reads it (line endings normalised to \n), or
// empty if the resource is missing from the build.
const std::string &Text();

// True once the player, through the installer or the overlay, has agreed to
// exactly this text. Read from the registry on the first call and kept.
bool Accepted();

// Records agreement to this text for the current Windows user. On failure
// Accepted() still turns true for this session, so the player isn't stuck,
// and detail says what went wrong, for the session log.
bool RecordAcceptance(std::string &detail);

// Uppercase hex SHA-256, as Get-FileHash writes it; empty if CNG refuses.
std::string Sha256Hex(const std::string &data);

} // namespace questcal::license
