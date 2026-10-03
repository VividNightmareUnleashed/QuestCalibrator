#pragma once

namespace questcal {
enum class ChaperonePermission { OwnerUnavailable, ForeignOwner, Unsettled, Allowed };
inline ChaperonePermission ChaperoneRestorePermission(bool ownerKnown,
	bool ownerMatches, bool baselineCurrent)
{
	if (!ownerKnown) return ChaperonePermission::OwnerUnavailable;
	if (!ownerMatches) return ChaperonePermission::ForeignOwner;
	if (!baselineCurrent) return ChaperonePermission::Unsettled;
	return ChaperonePermission::Allowed;
}
enum ChaperoneRestoreResult
{
	ChaperoneRestored = 0,
	ChaperoneCommitRejected = 1,
	ChaperoneReadbackMismatch = 2,
};
// Caller has validated snapshot plausibility and permission before entering.
// Owns the complete revert/stage/commit/readback order; platform calls are
// injected operations. Failed commit and readback never report restoration.
template<class Revert, class Stage, class Commit, class Verify>
ChaperoneRestoreResult RestoreChaperoneTransaction(Revert revert, Stage stage, Commit commit, Verify verify)
{
	revert();
	stage();
	if (!commit()) { revert(); return ChaperoneCommitRejected; }
	revert(); // read the committed live copy, not our uncommitted working data
	return verify() ? ChaperoneRestored : ChaperoneReadbackMismatch;
}
}
