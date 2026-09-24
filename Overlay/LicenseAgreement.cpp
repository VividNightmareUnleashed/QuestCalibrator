#include "stdafx.h"
#include "LicenseAgreement.h"

#include <bcrypt.h>
#include <ctime>
#pragma comment(lib, "bcrypt.lib")

namespace questcal::license
{

namespace
{

// The same path under both roots: HKLM for the installer, the per-user Local
// Settings root (where the overlay keeps its records) for the overlay.
const char *const Key = "Software\\QuestCalibrator\\License";

// The resource's bytes exactly as LICENSE had them when the build ran: the
// hash has to be the one the installer takes from the same file.
const std::string &RawText()
{
	static const std::string raw = []() -> std::string {
		HRSRC resource = FindResourceA(nullptr, "LICENSE_TEXT", MAKEINTRESOURCEA(10));
		if (!resource)
			return {};
		const char *bytes = static_cast<const char *>(LockResource(LoadResource(nullptr, resource)));
		if (!bytes)
			return {};
		return std::string(bytes, SizeofResource(nullptr, resource));
	}();
	return raw;
}

} // namespace

std::string Sha256Hex(const std::string &data)
{
	BCRYPT_ALG_HANDLE algorithm = nullptr;
	if (!BCRYPT_SUCCESS(BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0)))
		return {};
	unsigned char digest[32] = {};
	const NTSTATUS status = BCryptHash(algorithm, nullptr, 0,
		reinterpret_cast<PUCHAR>(const_cast<char *>(data.data())), static_cast<ULONG>(data.size()),
		digest, sizeof digest);
	BCryptCloseAlgorithmProvider(algorithm, 0);
	if (!BCRYPT_SUCCESS(status))
		return {};
	static const char hex[] = "0123456789ABCDEF";
	std::string out;
	for (unsigned char b : digest)
	{
		out += hex[b >> 4];
		out += hex[b & 0xF];
	}
	return out;
}

namespace
{

const std::string &TextHash()
{
	static const std::string hash = RawText().empty() ? std::string() : Sha256Hex(RawText());
	return hash;
}

std::string ReadHash(HKEY root, const char *key)
{
	char value[128] = {};
	DWORD size = sizeof value;
	if (RegGetValueA(root, key, "AcceptedSha256", RRF_RT_REG_SZ, nullptr, value, &size) != ERROR_SUCCESS)
		return {};
	return value;
}

bool SameHash(const std::string &recorded)
{
	return !TextHash().empty() && _stricmp(recorded.c_str(), TextHash().c_str()) == 0;
}

bool s_accepted = false;
bool s_read = false;

} // namespace

const std::string &Text()
{
	static const std::string text = [] {
		std::string out;
		out.reserve(RawText().size());
		for (char c : RawText())
			if (c != '\r')
				out += c;
		return out;
	}();
	return text;
}

bool Accepted()
{
	if (!s_read)
	{
		s_read = true;
		s_accepted = SameHash(ReadHash(HKEY_LOCAL_MACHINE, Key)) ||
			SameHash(ReadHash(HKEY_CURRENT_USER_LOCAL_SETTINGS, Key));
	}
	return s_accepted;
}

bool RecordAcceptance(std::string &detail)
{
	s_read = true;
	s_accepted = true;
	if (TextHash().empty())
	{
		detail = RawText().empty() ? "the LICENSE_TEXT resource is missing" : "SHA-256 of LICENSE_TEXT failed";
		return false;
	}

	char when[32] = {};
	const std::time_t now = std::time(nullptr);
	std::tm utc = {};
	if (gmtime_s(&utc, &now) == 0)
		std::strftime(when, sizeof when, "%Y-%m-%dT%H:%M:%SZ", &utc);

	HKEY key = nullptr;
	LSTATUS result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, Key, 0, nullptr, 0,
		KEY_SET_VALUE, nullptr, &key, nullptr);
	if (result == ERROR_SUCCESS)
	{
		result = RegSetValueExA(key, "AcceptedSha256", 0, REG_SZ,
			reinterpret_cast<const BYTE *>(TextHash().c_str()), static_cast<DWORD>(TextHash().size() + 1));
		if (result == ERROR_SUCCESS)
			result = RegSetValueExA(key, "AcceptedUtc", 0, REG_SZ,
				reinterpret_cast<const BYTE *>(when), static_cast<DWORD>(strlen(when) + 1));
		RegCloseKey(key);
	}
	if (result != ERROR_SUCCESS)
	{
		detail = "writing HKCU Local Settings\\" + std::string(Key) + ": Windows error " +
			std::to_string(result);
		return false;
	}
	return true;
}

} // namespace questcal::license
