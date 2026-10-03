#include "../Overlay/PoseMath.h"
#include "../common/MathConstants.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

// The shared pose primitives, on the inputs their callers used to get wrong:
// a quaternion with w < 0 is the same rotation as with w > 0.
namespace
{
using Check = void (*)(const char *, bool, const char *);

void RunSignedYawScenario(Check check)
{
	// The twist of Ry(yaw) * Rx(tilt) about +Y is `yaw` for any tilt short of
	// a half turn, and must read the same from q and from -q.
	const double degrees[] = { 0.3, 25.0, 90.0, 179.0, -0.3, -170.0 };
	double worst = 0.0, widest = 0.0;
	for (double tiltDeg : { 0.0, 10.0, -35.0 })
	{
		for (double yawDeg : degrees)
		{
			const double yaw = yawDeg * questcal::Pi / 180.0;
			const Eigen::Quaterniond q = Eigen::Quaterniond(
				Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())) *
				Eigen::Quaterniond(Eigen::AngleAxisd(tiltDeg * questcal::Pi / 180.0,
					Eigen::Vector3d::UnitX()));
			for (const Eigen::Quaterniond &sign : { q, Eigen::Quaterniond(-q.coeffs()) })
			{
				const double read = questcal::SignedYawRadians(sign);
				worst = (std::max)(worst, std::abs(read - yaw));
				widest = (std::max)(widest, std::abs(read));
			}
		}
	}
	// A half turn about a horizontal axis has no heading.
	const double headless = questcal::SignedYawRadians(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0));
	char detail[128];
	snprintf(detail, sizeof detail, "worst %.2e rad, widest %.4f rad, half turn %.3f",
		worst, widest, headless);
	check("pose math: yaw reads the same from q and -q",
		worst < 1e-12 && widest <= questcal::Pi && headless == 0.0, detail);
}

void RunTiltResidualScenario(Check check)
{
	// The tilt the projection leaves out is reported with the yaw, so a log
	// line can name both without projecting twice.
	const Eigen::Quaterniond q = Eigen::Quaterniond(
		Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())) *
		Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
	double tilt = 0.0, tiltNegated = 0.0;
	const double yaw = questcal::SignedYawRadians(q, &tilt);
	const double yawNegated = questcal::SignedYawRadians(Eigen::Quaterniond(-q.coeffs()), &tiltNegated);
	char detail[128];
	snprintf(detail, sizeof detail, "yaw %.6f / %.6f, tilt %.6f / %.6f",
		yaw, yawNegated, tilt, tiltNegated);
	check("pose math: the tilt residual comes with the yaw",
		std::abs(yaw - 0.5) < 1e-12 && std::abs(yawNegated - 0.5) < 1e-12 &&
		std::abs(tilt - 0.1) < 1e-12 && std::abs(tiltNegated - 0.1) < 1e-12, detail);
}

void RunQuaternionMeanScenario(Check check)
{
	// Rotations scattered evenly about a known one, half of them stored as -q:
	// the mean is the known rotation whatever sign each one came with.
	const Eigen::Quaterniond center(Eigen::AngleAxisd(0.8, Eigen::Vector3d(0.3, 0.9, -0.2).normalized()));
	const Eigen::Vector3d axes[] = { Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ() };
	Eigen::Matrix4d sum = Eigen::Matrix4d::Zero();
	int index = 0;
	for (const auto &axis : axes)
	{
		for (double angle : { 0.05, -0.05 })
		{
			Eigen::Quaterniond q = (Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)) * center).normalized();
			if (index++ % 2 == 1)
				q.coeffs() = -q.coeffs();
			sum += q.coeffs() * q.coeffs().transpose();
		}
	}
	const double error = questcal::QuaternionMean(sum).angularDistance(center);
	char detail[96];
	snprintf(detail, sizeof detail, "mean off by %.2e rad", error);
	check("pose math: the quaternion mean ignores each quaternion's sign", error < 1e-9, detail);
}
}

void RunPoseMathScenarios(Check check)
{
	RunSignedYawScenario(check);
	RunTiltResidualScenario(check);
	RunQuaternionMeanScenario(check);
}
