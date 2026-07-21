#pragma once

#ifdef QUESTCALIBRATORDRIVER_EXPORTS
#define QUESTCALIBRATORDRIVER_API extern "C" __declspec(dllexport)
#else
#define QUESTCALIBRATORDRIVER_API extern "C" __declspec(dllimport)
#endif
