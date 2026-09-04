#include "stdafx.h"
#include "Updater.h"
#include "../common/Version.h"

#include <bcrypt.h>
#include <winhttp.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

#pragma comment(lib, "bcrypt.lib")
#pragma comment(lib, "winhttp.lib")

namespace questcal
{
namespace update
{
namespace
{

class InternetHandle
{
public:
	InternetHandle() = default;
	explicit InternetHandle(HINTERNET value) : value(value) {}
	~InternetHandle() { if (value) WinHttpCloseHandle(value); }
	InternetHandle(const InternetHandle &) = delete;
	InternetHandle &operator=(const InternetHandle &) = delete;
	InternetHandle(InternetHandle &&other) noexcept : value(other.value)
	{
		other.value = nullptr;
	}
	InternetHandle &operator=(InternetHandle &&other) noexcept
	{
		if (this != &other)
		{
			if (value) WinHttpCloseHandle(value);
			value = other.value;
			other.value = nullptr;
		}
		return *this;
	}
	operator HINTERNET() const { return value; }
	explicit operator bool() const { return value != nullptr; }

private:
	HINTERNET value = nullptr;
};

struct HttpRequest
{
	InternetHandle connection;
	InternetHandle request;
	operator HINTERNET() const { return request; }
};

std::runtime_error NetworkError(const char *operation)
{
	return std::runtime_error(std::string(operation) + " failed (Windows error " +
		std::to_string(GetLastError()) + ").");
}

std::wstring Utf8ToWide(const std::string &text)
{
	if (text.empty()) return std::wstring();
	const int chars = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
		text.data(), static_cast<int>(text.size()), nullptr, 0);
	if (chars <= 0) throw std::runtime_error("GitHub returned invalid UTF-8.");
	std::wstring wide(static_cast<size_t>(chars), L'\0');
	if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
		static_cast<int>(text.size()), &wide[0], chars) != chars)
		throw std::runtime_error("GitHub returned invalid UTF-8.");
	return wide;
}

std::filesystem::path LocalUpdateRoot()
{
	DWORD chars = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);
	if (chars <= 1)
		throw std::runtime_error("Windows did not provide a local app-data folder.");
	std::wstring value(static_cast<size_t>(chars), L'\0');
	DWORD written = GetEnvironmentVariableW(L"LOCALAPPDATA", &value[0], chars);
	if (written == 0 || written >= chars)
		throw std::runtime_error("Windows did not provide a local app-data folder.");
	value.resize(written);
	return std::filesystem::path(value) / L"QuestCalibrator" / L"updates";
}

InternetHandle OpenSession()
{
	InternetHandle session(WinHttpOpen(L"QuestCalibrator/" QUESTCAL_VERSION_STRING,
		WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME,
		WINHTTP_NO_PROXY_BYPASS, 0));
	if (!session) throw NetworkError("Opening the update connection");
	if (!WinHttpSetTimeouts(session, 5000, 5000, 5000, 10000))
		throw NetworkError("Setting update timeouts");
	return session;
}

HttpRequest OpenGet(HINTERNET session, const wchar_t *host,
	INTERNET_PORT port, const wchar_t *path)
{
	HttpRequest result;
	result.connection = InternetHandle(WinHttpConnect(session, host, port, 0));
	if (!result.connection) throw NetworkError("Connecting to GitHub");
	result.request = InternetHandle(WinHttpOpenRequest(result.connection, L"GET", path, nullptr,
		WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE));
	if (!result.request) throw NetworkError("Creating the GitHub request");
	DWORD redirectPolicy = WINHTTP_OPTION_REDIRECT_POLICY_DISALLOW_HTTPS_TO_HTTP;
	if (!WinHttpSetOption(result.request, WINHTTP_OPTION_REDIRECT_POLICY,
		&redirectPolicy, sizeof redirectPolicy))
		throw NetworkError("Securing GitHub redirects");
	return result;
}

void SendGet(HINTERNET request, const wchar_t *headers = WINHTTP_NO_ADDITIONAL_HEADERS)
{
	const DWORD headerLength = headers == WINHTTP_NO_ADDITIONAL_HEADERS ? 0 : -1L;
	if (!WinHttpSendRequest(request, headers, headerLength,
		WINHTTP_NO_REQUEST_DATA, 0, 0, 0) || !WinHttpReceiveResponse(request, nullptr))
		throw NetworkError("Requesting the update");

	DWORD status = 0, bytes = sizeof status;
	if (!WinHttpQueryHeaders(request,
		WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
		WINHTTP_HEADER_NAME_BY_INDEX, &status, &bytes, WINHTTP_NO_HEADER_INDEX))
		throw NetworkError("Reading the update response");
	if (status != 200)
		throw std::runtime_error("GitHub returned HTTP " + std::to_string(status) + ".");
}

std::string ReadResponse(HINTERNET request, size_t maxBytes,
	const std::function<bool()> &cancelled)
{
	std::string body;
	std::array<char, 16384> buffer;
	for (;;)
	{
		if (cancelled()) throw std::runtime_error("Update check cancelled.");
		DWORD read = 0;
		if (!WinHttpReadData(request, buffer.data(),
			static_cast<DWORD>(buffer.size()), &read))
			throw NetworkError("Reading the update response");
		if (read == 0) break;
		if (body.size() + read > maxBytes)
			throw std::runtime_error("GitHub returned an unexpectedly large release list.");
		body.append(buffer.data(), read);
	}
	return body;
}

std::string FetchReleaseFeed(HINTERNET session,
	const std::function<bool()> &cancelled)
{
	HttpRequest request = OpenGet(session, L"api.github.com",
		INTERNET_DEFAULT_HTTPS_PORT,
		L"/repos/VividNightmareUnleashed/QuestCalibrator/releases?per_page=20");
	SendGet(request,
		L"Accept: application/vnd.github+json\r\n"
		L"X-GitHub-Api-Version: 2022-11-28\r\n");
	return ReadResponse(request, 1024 * 1024, cancelled);
}

struct DownloadTarget
{
	std::wstring host;
	std::wstring path;
	INTERNET_PORT port = INTERNET_DEFAULT_HTTPS_PORT;
};

DownloadTarget ParseDownloadTarget(const std::string &url)
{
	std::wstring wide = Utf8ToWide(url);
	URL_COMPONENTS parts{};
	parts.dwStructSize = sizeof parts;
	parts.dwHostNameLength = static_cast<DWORD>(-1);
	parts.dwUrlPathLength = static_cast<DWORD>(-1);
	parts.dwExtraInfoLength = static_cast<DWORD>(-1);
	if (!WinHttpCrackUrl(wide.c_str(), static_cast<DWORD>(wide.size()), 0, &parts))
		throw std::runtime_error("The release package URL is invalid.");
	if (parts.nScheme != INTERNET_SCHEME_HTTPS)
		throw std::runtime_error("The release package URL is not HTTPS.");

	DownloadTarget target;
	target.host.assign(parts.lpszHostName, parts.dwHostNameLength);
	if (_wcsicmp(target.host.c_str(), L"github.com") != 0)
		throw std::runtime_error("The release package URL does not use github.com.");
	target.path.assign(parts.lpszUrlPath, parts.dwUrlPathLength);
	if (parts.dwExtraInfoLength)
		target.path.append(parts.lpszExtraInfo, parts.dwExtraInfoLength);
	target.port = parts.nPort;
	return target;
}

bool HashFile(const std::filesystem::path &path,
	std::array<unsigned char, 32> &digest)
{
	BCRYPT_ALG_HANDLE algorithm = nullptr;
	BCRYPT_HASH_HANDLE hash = nullptr;
	std::vector<unsigned char> object;
	bool ok = false;
	do
	{
		if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM,
			nullptr, 0) < 0) break;
		DWORD objectBytes = 0, resultBytes = 0;
		if (BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH,
			reinterpret_cast<PUCHAR>(&objectBytes), sizeof objectBytes,
			&resultBytes, 0) < 0 || objectBytes == 0) break;
		object.resize(objectBytes);
		if (BCryptCreateHash(algorithm, &hash, object.data(), objectBytes,
			nullptr, 0, 0) < 0) break;

		std::ifstream input(path, std::ios::binary);
		if (!input) break;
		std::array<char, 65536> buffer;
		while (input)
		{
			input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
			const std::streamsize count = input.gcount();
			if (count > 0 && BCryptHashData(hash,
				reinterpret_cast<PUCHAR>(buffer.data()),
				static_cast<ULONG>(count), 0) < 0) break;
		}
		if (!input.eof()) break;
		if (BCryptFinishHash(hash, digest.data(),
			static_cast<ULONG>(digest.size()), 0) < 0) break;
		ok = true;
	} while (false);
	if (hash) BCryptDestroyHash(hash);
	if (algorithm) BCryptCloseAlgorithmProvider(algorithm, 0);
	return ok;
}

bool FileMatches(const std::filesystem::path &path, uint64_t size,
	const std::array<unsigned char, 32> &digest)
{
	std::error_code ec;
	if (!std::filesystem::is_regular_file(path, ec) || ec ||
		std::filesystem::file_size(path, ec) != size || ec)
		return false;
	std::array<unsigned char, 32> actual{};
	return HashFile(path, actual) && actual == digest;
}

void DownloadPackage(HINTERNET session, const ReleaseCandidate &release,
	const std::filesystem::path &partPath,
	const std::function<bool()> &cancelled,
	const std::function<void(uint64_t)> &progress)
{
	const DownloadTarget target = ParseDownloadTarget(release.downloadUrl);
	HttpRequest request = OpenGet(session, target.host.c_str(),
		target.port, target.path.c_str());
	SendGet(request);

	std::ofstream output(partPath, std::ios::binary | std::ios::trunc);
	if (!output)
		throw std::runtime_error("The update package could not be saved.");
	std::array<char, 65536> buffer;
	uint64_t total = 0;
	for (;;)
	{
		if (cancelled()) throw std::runtime_error("Update download cancelled.");
		DWORD read = 0;
		if (!WinHttpReadData(request, buffer.data(),
			static_cast<DWORD>(buffer.size()), &read))
			throw NetworkError("Downloading the update package");
		if (read == 0) break;
		if (total + read > release.size)
			throw std::runtime_error("The update package is larger than GitHub reported.");
		output.write(buffer.data(), read);
		if (!output)
			throw std::runtime_error("The update package could not be saved.");
		total += read;
		progress(total);
	}
	output.close();
	if (total != release.size)
		throw std::runtime_error("The update package download was incomplete.");
}

const char *UpdateHelperScript()
{
	return R"PS1(param(
    [Parameter(Mandatory=$true)][int]$ParentPid,
    [Parameter(Mandatory=$true)][string]$ZipPath,
    [Parameter(Mandatory=$true)][string]$ExpectedSha256
)
$ErrorActionPreference = 'Stop'
try {
    Wait-Process -Id $ParentPid -ErrorAction SilentlyContinue
    $actual = (Get-FileHash -LiteralPath $ZipPath -Algorithm SHA256).Hash
    if ($actual -ine $ExpectedSha256) { throw 'The downloaded package no longer matches its verified SHA-256 digest.' }

    $updateDir = Split-Path -Parent $ZipPath
    $extractDir = Join-Path $updateDir 'package'
    if (Test-Path -LiteralPath $extractDir) { Remove-Item -LiteralPath $extractDir -Recurse -Force }
    New-Item -ItemType Directory -Path $extractDir | Out-Null

    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $root = [IO.Path]::GetFullPath($extractDir + [IO.Path]::DirectorySeparatorChar)
    $archive = [IO.Compression.ZipFile]::OpenRead($ZipPath)
    try {
        foreach ($entry in $archive.Entries) {
            $destination = [IO.Path]::GetFullPath((Join-Path $extractDir $entry.FullName))
            if (-not $destination.StartsWith($root, [StringComparison]::OrdinalIgnoreCase)) {
                throw 'The update package contains an unsafe path.'
            }
        }
    } finally { $archive.Dispose() }
    Expand-Archive -LiteralPath $ZipPath -DestinationPath $extractDir -Force

    $installers = @(Get-ChildItem -LiteralPath $extractDir -Filter Install.ps1 -File -Recurse)
    if ($installers.Count -ne 1) { throw 'The update package does not contain exactly one installer.' }
    $packageDir = $installers[0].Directory.FullName
    foreach ($required in @('app\QuestCalibrator.exe', 'driver\01questcalibrator\bin\win64\driver_01questcalibrator.dll', 'Uninstall.ps1')) {
        if (-not (Test-Path -LiteralPath (Join-Path $packageDir $required))) {
            throw "The update package is incomplete: $required is missing."
        }
    }

    Write-Host 'QuestCalibrator is ready to update.' -ForegroundColor Cyan
    Write-Host 'Close Steam completely, including its system-tray icon. Installation starts when Steam has stopped.'
    while (Get-Process -Name 'steam','vrserver','vrmonitor','vrcompositor' -ErrorAction SilentlyContinue) {
        Start-Sleep -Seconds 2
    }

    $arguments = '-NoProfile -ExecutionPolicy Bypass -File "' + $installers[0].FullName + '"'
    $installer = Start-Process -FilePath 'powershell.exe' -Verb RunAs -ArgumentList $arguments -WorkingDirectory $packageDir -Wait -PassThru
    if ($installer.ExitCode -ne 0) { throw "The installer exited with code $($installer.ExitCode)." }
    exit 0
} catch {
    Write-Host ''
    Write-Host "Update failed: $_" -ForegroundColor Red
    Read-Host 'Press Enter to close'
    exit 1
}
)PS1";
}

std::wstring SystemPowerShell()
{
	std::vector<wchar_t> path(MAX_PATH);
	UINT written = GetSystemDirectoryW(path.data(), static_cast<UINT>(path.size()));
	if (written == 0 || written >= path.size())
		throw std::runtime_error("Windows PowerShell could not be located.");
	std::wstring executable(path.data(), written);
	executable += L"\\WindowsPowerShell\\v1.0\\powershell.exe";
	if (!std::filesystem::is_regular_file(executable))
		throw std::runtime_error("Windows PowerShell could not be located.");
	return executable;
}

std::wstring QuoteArgument(const std::wstring &value)
{
	// Windows paths cannot contain quotes. Every caller supplies either a path
	// under LOCALAPPDATA or a hexadecimal/numeric value.
	if (value.find(L'"') != std::wstring::npos)
		throw std::runtime_error("The update path contains an unsupported character.");
	return L"\"" + value + L"\"";
}

} // namespace

Updater AppUpdater;

Updater::~Updater()
{
	Shutdown();
}

void Updater::SetLogSink(LogSink sink)
{
	std::lock_guard<std::mutex> lock(mutex);
	logSink = std::move(sink);
}

void Updater::Log(const std::string &message) const
{
	try
	{
		LogSink sink;
		{
			std::lock_guard<std::mutex> lock(mutex);
			sink = logSink;
		}
		if (sink)
			sink(message);
	}
	catch (...)
	{
		// Diagnostics must never change updater behavior.
	}
}

void Updater::SetEnabled(bool value)
{
	bool start = false;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (stopping || enabled == value)
			return;
		enabled = value;
		++revision;
		readyRelease = ReleaseCandidate();
		readyPackagePath.clear();
		snapshot = Snapshot();
		snapshot.state = enabled ? State::Idle : State::Disabled;
		start = enabled;
	}
	Log(value ? "automatic updates enabled" : "automatic updates disabled");
	if (start)
		CheckNow();
}

bool Updater::CheckNow()
{
	std::thread finished;
	uint64_t checkRevision = 0;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (!enabled || stopping || running)
			return false;
		if (worker.joinable())
			finished = std::move(worker);
		running = true;
		checkRevision = ++revision;
		snapshot = Snapshot();
		snapshot.state = State::Checking;
		snapshot.message = "Checking GitHub for updates";
	}
	if (finished.joinable())
		finished.join();
	Log("check started (current " QUESTCAL_VERSION_STRING ")");
	try
	{
		worker = std::thread(&Updater::RunChecks, this, checkRevision);
	}
	catch (const std::exception &failure)
	{
		bool current = false;
		{
			std::lock_guard<std::mutex> lock(mutex);
			running = false;
			current = enabled && !stopping && revision == checkRevision;
			if (current)
			{
				snapshot.state = State::Failed;
				snapshot.message = "The update check could not start.";
			}
		}
		if (current)
			Log("check could not start: " + std::string(failure.what()));
		return false;
	}
	catch (...)
	{
		bool current = false;
		{
			std::lock_guard<std::mutex> lock(mutex);
			running = false;
			current = enabled && !stopping && revision == checkRevision;
			if (current)
			{
				snapshot.state = State::Failed;
				snapshot.message = "The update check could not start.";
			}
		}
		if (current)
			Log("check could not start: unknown thread error");
		return false;
	}
	return true;
}

Snapshot Updater::GetSnapshot() const
{
	std::lock_guard<std::mutex> lock(mutex);
	return snapshot;
}

bool Updater::IsCurrent(uint64_t checkRevision) const
{
	std::lock_guard<std::mutex> lock(mutex);
	return enabled && !stopping && revision == checkRevision;
}

bool Updater::Publish(uint64_t checkRevision, State state,
	const std::string &message, const std::string &version,
	uint64_t downloaded, uint64_t total)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (!enabled || stopping || revision != checkRevision)
		return false;
	snapshot.state = state;
	snapshot.message = message;
	snapshot.version = version;
	snapshot.downloadedBytes = downloaded;
	snapshot.totalBytes = total;
	return true;
}

void Updater::RunChecks(uint64_t checkRevision)
{
	for (;;)
	{
		RunCheck(checkRevision);
		std::lock_guard<std::mutex> lock(mutex);
		if (!enabled || stopping || revision == checkRevision)
		{
			running = false;
			return;
		}
		// Re-enabling while the previous network request is finishing queues a
		// fresh check on this worker; its cancelled result cannot publish.
		checkRevision = revision;
		snapshot = Snapshot();
		snapshot.state = State::Checking;
		snapshot.message = "Checking GitHub for updates";
	}
}

void Updater::RunCheck(uint64_t checkRevision)
{
	try
	{
		auto cancelled = [&]() { return !IsCurrent(checkRevision); };
		InternetHandle session;
		std::string feed;
#ifdef QUESTCAL_UPDATER_TEST_SEAM
		if (fetchForTest)
			feed = fetchForTest();
		else
#endif
		{
			session = OpenSession();
			feed = FetchReleaseFeed(session, cancelled);
		}
		ReleaseCandidate release;
		bool available = false;
		std::string policyError;
		const Version current{ QUESTCAL_VERSION_MAJOR,
			QUESTCAL_VERSION_MINOR, QUESTCAL_VERSION_PATCH };
		if (!SelectReleaseCandidate(feed, current, release, available, policyError))
			throw std::runtime_error(policyError);
		if (!available)
		{
			const std::string version = VersionString(current);
			if (Publish(checkRevision, State::UpToDate,
				"QuestCalibrator is up to date", version))
				Log("check completed: current " + version +
					"; no newer stable release");
		}
		else
		{
			const std::string version = VersionString(release.version);
			if (!Publish(checkRevision, State::Downloading,
				"Downloading QuestCalibrator " + version, version, 0, release.size))
				throw std::runtime_error("Update check cancelled.");
			Log("stable release " + version + " found (" +
				std::to_string(release.size) + " bytes)");
			std::array<unsigned char, 32> expectedDigest{};
			if (!ParseSha256Digest(release.digest, expectedDigest))
				throw std::runtime_error("The release package digest is invalid.");

			const std::filesystem::path directory = LocalUpdateRoot() /
				Utf8ToWide(version);
			std::error_code directoryError;
			std::filesystem::create_directories(directory, directoryError);
			if (directoryError)
				throw std::runtime_error("The update folder could not be created (system error " +
					std::to_string(directoryError.value()) + ").");
			const std::filesystem::path package = directory /
				Utf8ToWide(release.packageName);
			bool downloaded = false;
			if (!FileMatches(package, release.size, expectedDigest))
			{
				const std::filesystem::path part = package.wstring() + L".part";
				std::error_code ignored;
				std::filesystem::remove(part, ignored);
				if (IsCurrent(checkRevision))
					Log("download started for " + version);
				DownloadPackage(session, release, part, cancelled,
					[&](uint64_t bytes) {
						Publish(checkRevision, State::Downloading,
							"Downloading QuestCalibrator " + version,
							version, bytes, release.size);
					});
				if (!FileMatches(part, release.size, expectedDigest))
				{
					std::filesystem::remove(part, ignored);
					throw std::runtime_error(
						"The downloaded package failed SHA-256 verification.");
				}
				std::filesystem::remove(package, ignored);
				std::error_code renameError;
				std::filesystem::rename(part, package, renameError);
				if (renameError)
				{
					std::filesystem::remove(part, ignored);
					throw std::runtime_error("The verified update package could not be saved (system error " +
						std::to_string(renameError.value()) + ").");
				}
				downloaded = true;
			}
			if (!IsCurrent(checkRevision))
				throw std::runtime_error("Update download cancelled.");
			{
				std::lock_guard<std::mutex> lock(mutex);
				if (enabled && !stopping && revision == checkRevision)
				{
					readyRelease = release;
					readyPackagePath = package.wstring();
				}
			}
			if (Publish(checkRevision, State::Ready,
				"QuestCalibrator " + version + " is ready to install",
				version, release.size, release.size))
			{
				Log(std::string(downloaded ? "downloaded package " : "cached package ") +
					version + "; size and SHA-256 verified; ready to install");
			}
		}
	}
	catch (const std::exception &error)
	{
		if (Publish(checkRevision, State::Failed, error.what()))
			Log("check failed: " + std::string(error.what()));
	}
	catch (...)
	{
		if (Publish(checkRevision, State::Failed,
			"The update check failed unexpectedly."))
			Log("check failed unexpectedly");
	}
}

bool Updater::LaunchInstaller(std::string &error)
{
	ReleaseCandidate release;
	std::filesystem::path package;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (snapshot.state != State::Ready || readyPackagePath.empty())
		{
			error = "No verified update is ready to install.";
		}
		else
		{
			release = readyRelease;
			package = readyPackagePath;
		}
	}
	if (package.empty())
	{
		Log("installer handoff rejected: " + error);
		return false;
	}

	try
	{
		std::array<unsigned char, 32> digest{};
		if (!ParseSha256Digest(release.digest, digest) ||
			!FileMatches(package, release.size, digest))
			throw std::runtime_error("The downloaded update no longer passes SHA-256 verification.");

		const std::filesystem::path script = package.parent_path() / L"ApplyUpdate.ps1";
		std::ofstream output(script, std::ios::binary | std::ios::trunc);
		if (!output)
			throw std::runtime_error("The update helper could not be created.");
		output << UpdateHelperScript();
		output.close();
		if (!output)
			throw std::runtime_error("The update helper could not be created.");

		const std::wstring powershell = SystemPowerShell();
		const std::wstring digestHex = Utf8ToWide(release.digest.substr(7));
		std::wstring command = QuoteArgument(powershell) +
			L" -NoProfile -ExecutionPolicy Bypass -File " + QuoteArgument(script.wstring()) +
			L" -ParentPid " + std::to_wstring(GetCurrentProcessId()) +
			L" -ZipPath " + QuoteArgument(package.wstring()) +
			L" -ExpectedSha256 " + QuoteArgument(digestHex);
		std::vector<wchar_t> commandLine(command.begin(), command.end());
		commandLine.push_back(L'\0');
		STARTUPINFOW startup{};
		startup.cb = sizeof startup;
		PROCESS_INFORMATION process{};
		if (!CreateProcessW(powershell.c_str(), commandLine.data(), nullptr,
			nullptr, FALSE, CREATE_NEW_CONSOLE, nullptr,
			package.parent_path().c_str(), &startup, &process))
			throw NetworkError("Starting the update helper");
		CloseHandle(process.hThread);
		CloseHandle(process.hProcess);
		Log("installer handoff started for " + VersionString(release.version) +
			" after SHA-256 re-verification");
		return true;
	}
	catch (const std::exception &failure)
	{
		error = failure.what();
		Log("installer handoff failed for " + VersionString(release.version) +
			": " + error);
		return false;
	}
}

void Updater::Shutdown()
{
	std::thread active;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (stopping && !worker.joinable())
			return;
		stopping = true;
		enabled = false;
		++revision;
		if (worker.joinable())
			active = std::move(worker);
		snapshot.state = State::Disabled;
	}
	if (active.joinable())
		active.join();
	std::lock_guard<std::mutex> lock(mutex);
	running = false;
}

} // namespace update
} // namespace questcal
