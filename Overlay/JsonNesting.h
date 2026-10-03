#pragma once

// The one nesting guard every picojson parse of untrusted text goes through:
// the registry records and the GitHub release feed alike.

#include "RecordBounds.h"

#include <stdexcept>
#include <string>

namespace questcal
{

// picojson recurses with no depth limit, and a stack overflow is an SEH
// exception no catch sees: a deeply nested registry value would crash at
// startup with no log, and a deeply nested release feed would crash the update
// check. Reject that shape before parsing so it takes the caller's ordinary
// refusal path. The records nest at most three levels and the release feed
// about five; brackets inside strings do not count.
inline void RejectExcessiveJsonNesting(const std::string &text)
{
	int depth = 0;
	bool inString = false;
	bool escaped = false;
	for (char c : text)
	{
		if (inString)
		{
			if (escaped)
				escaped = false;
			else if (c == '\\')
				escaped = true;
			else if (c == '"')
				inString = false;
			continue;
		}

		if (c == '"')
			inString = true;
		else if (c == '[' || c == '{')
		{
			if (!IsValidJsonDepth(++depth))
				throw std::runtime_error("record nesting is too deep");
		}
		else if (c == ']' || c == '}')
			--depth;
	}
}

}
