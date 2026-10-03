#pragma once

// The overlay reads more than one clock, and a reading of one means nothing on
// another: QPC counts from boot and the app's clock from launch, hours apart,
// so comparing the two still compiles as doubles and gives a timer that never,
// or always, expires. A reading of each clock has its own type here, and
// changes type only where code names the clock: RingTime(seconds), .seconds.
//   RingTime  seconds on the QPC clock the driver stamps poses with
//             (RingSampleTime, RingCaptureTime): sample times, the continuous
//             loop and the jump detector.
//   UiTime    seconds on the app's clock (glfwGetTime), which runs the tick,
//             its timers and grace periods.
// Wall-clock seconds (std::time, the SteamVR log's stamps) stay doubles named
// for it, unixTime, and meet the ring clock in one place, where the lighthouse
// log's events are carried onto it.
namespace questcal
{

template <class Clock>
struct ClockTime
{
	double seconds = 0.0;

	constexpr ClockTime() = default;
	constexpr explicit ClockTime(double s) : seconds(s) {}

	// The time between two readings is plain seconds, and seconds move a
	// reading along its own clock.
	friend constexpr double operator-(ClockTime a, ClockTime b) { return a.seconds - b.seconds; }
	friend constexpr ClockTime operator+(ClockTime a, double d) { return ClockTime(a.seconds + d); }
	friend constexpr ClockTime operator-(ClockTime a, double d) { return ClockTime(a.seconds - d); }
	friend constexpr bool operator==(ClockTime a, ClockTime b) { return a.seconds == b.seconds; }
	friend constexpr bool operator!=(ClockTime a, ClockTime b) { return a.seconds != b.seconds; }
	friend constexpr bool operator<(ClockTime a, ClockTime b) { return a.seconds < b.seconds; }
	friend constexpr bool operator<=(ClockTime a, ClockTime b) { return a.seconds <= b.seconds; }
	friend constexpr bool operator>(ClockTime a, ClockTime b) { return a.seconds > b.seconds; }
	friend constexpr bool operator>=(ClockTime a, ClockTime b) { return a.seconds >= b.seconds; }
};

struct RingClock;
struct UiClock;
using RingTime = ClockTime<RingClock>;
using UiTime = ClockTime<UiClock>;

} // namespace questcal
