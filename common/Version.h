#pragma once

// Included by the .rc files as well as C++, so keep this header free of
// anything the resource compiler's preprocessor can't handle.
#define QUESTCAL_VERSION_MAJOR 1
#define QUESTCAL_VERSION_MINOR 2
#define QUESTCAL_VERSION_PATCH 0

// Prerelease identity, and the thing that puts this build on the hand-installed
// lane rather than the stable one. A final release leaves the label empty and
// the ordinal 0; a prerelease sets both, so "alpha" with 3 is 1.2.0-alpha.3.
// A build with a label never asks the stable feed for anything: the updater
// says which prerelease this is and leaves the move to the tester.
//
// Before this the alphas declared the bare 1.2.0, which is the number of the
// release they precede, so every one of them reported a version it was not.
//
// Making a final release means clearing these two back to "" and 0 in the
// same commit that drops the suffix from the string below.
#define QUESTCAL_VERSION_PRERELEASE_LABEL "alpha"
#define QUESTCAL_VERSION_PRERELEASE_ORDINAL 3

// The resource compiler cannot build this from the numbers above, so it is
// written out by hand; the solver harness asserts the two agree.
#define QUESTCAL_VERSION_STRING "1.2.0-alpha.3"
