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
	explicit Updater(std::function<std::string()> fetch) : fetchForTest(std::move(fetch)) {}
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
	bool Publish(uint64_t revision, State state, const std::string &message,
		const std::string &version = std::string(), uint64_t downloaded = 0,
		uint64_t total = 0);
	void Log(const std::string &message) const;

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
