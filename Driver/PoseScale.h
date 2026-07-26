#pragma once

namespace questcal
{

inline void ScaleLinearPose(double scale, double (&position)[3],
	double (&velocity)[3], double (&acceleration)[3])
{
	for (int i = 0; i < 3; ++i)
	{
		position[i] *= scale;
		velocity[i] *= scale;
		acceleration[i] *= scale;
	}
}

} // namespace questcal
