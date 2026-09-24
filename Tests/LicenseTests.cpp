// The license gate's fingerprint: the overlay and Install.ps1 must write the
// same hash for the same LICENSE, or an installed player is asked again in
// the overlay and an overlay agreement never satisfies the installer.
#include "../Overlay/LicenseAgreement.h"

#include <string>

namespace
{
using Check = void (*)(const char *, bool, const char *);
using namespace questcal::license;

// FIPS 180-2's test vectors, in Get-FileHash's uppercase hex.
void HashMatchesGetFileHash(Check check)
{
	check("license: SHA-256 of \"abc\"",
		Sha256Hex("abc") == "BA7816BF8F01CFEA414140DE5DAE2223B00361A396177A9CB410FF61F20015AD",
		Sha256Hex("abc").c_str());
	check("license: SHA-256 of nothing",
		Sha256Hex("") == "E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855",
		Sha256Hex("").c_str());
	const std::string crlf = "line\r\n";
	check("license: line endings are hashed as they are",
		Sha256Hex(crlf) != Sha256Hex("line\n"), "CRLF and LF hashed alike");
}

// The test binary carries no LICENSE_TEXT resource. Without the text there is
// nothing to have agreed to, so no recorded hash may count as agreement.
void MissingTextIsNeverAccepted(Check check)
{
	check("license: no embedded text, no text shown", Text().empty(), "Text() not empty");
	check("license: no embedded text, never accepted", !Accepted(), "Accepted() true");
}

} // namespace

void RunLicenseScenarios(Check check)
{
	HashMatchesGetFileHash(check);
	MissingTextIsNeverAccepted(check);
}
