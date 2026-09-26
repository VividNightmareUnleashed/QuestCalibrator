#pragma once

#include "UpdatePolicy.h"

#include <atomic>
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace questcal
{
namespace update
{

// Shared with diagnostics to identify installed binaries using the same
// streaming SHA-256 implementation used for verified downloads.
bool HashFileSha256(const std::filesystem::path &path, std::array<unsigned char, 32> &digest);

enum class State
{
	Disabled,
	// This build is a prerelease, so it is not on the stable lane and the
	// updater will not check. The tester installs a stable build by hand.
	Prerelease,
	Idle,
	Checking,
	Downloading,
	UpToDate,
	Ready,
	Failed
};

struct Snapshot
{
	State state = State::Disabled;
	std::string version;
	std::string message;
	uint64_t downloadedBytes = 0;
	uint64_t totalBytes = 0;
};

class Updater
{
public:
	Updater() = default;
#ifdef QUESTCAL_UPDATER_TEST_SEAM
	// The build identity is injectable so the check machinery can be driven as
	// a stable build from a harness that is itself compiled as a prerelease.
	Updater(std::function<std::string()> fetch, const Version &version)
		: build(version), fetchForTest(std::move(fetch)) {}
#endif
	~Updater();
	Updater(const Updater &) = delete;
	Updater &operator=(const Updater &) = delete;

	using LogSink = std::function<void(const std::string &)>;
	void SetLogSink(LogSink sink);
	void SetEnabled(bool enabled);
	bool CheckNow();
	Snapshot GetSnapshot() const;
	bool LaunchInstaller(std::string &error);
	void Shutdown();

private:
	void RunChecks(uint64_t revision);
	void RunCheck(uint64_t revision);
	bool IsCurrent(uint64_t revision) const;
	// Requires `mutex`.
	bool IsCurrentLocked(uint64_t checkRevision) const
	{
		return enabled && !stopping && revision == checkRevision;
	}
	bool Publish(uint64_t revision, State state, const std::string &message,
		const std::string &version = std::string(), uint64_t downloaded = 0,
		uint64_t total = 0);
	void Log(const std::string &message) const;

	const Version build = CurrentVersion();
	mutable std::mutex mutex;
	std::thread worker;
	bool enabled = false;
	bool running = false;
	bool stopping = false;
	uint64_t revision = 0;
	Snapshot snapshot;
	ReleaseCandidate readyRelease;
	std::wstring readyPackagePath;
	LogSink logSink;
#ifdef QUESTCAL_UPDATER_TEST_SEAM
	std::function<std::string()> fetchForTest;
#endif
};

extern Updater AppUpdater;

} // namespace update
} // namespace questcal
