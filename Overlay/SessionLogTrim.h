#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace questcal
{

// The session log's size budget. It keeps the session's first headBytes and a
// rolling tail of its latest lines: once the file reaches maxBytes it is
// rewritten as the head, a line saying how many lines were left out, and the
// tail. A long session's report then shows how it started and how it ended,
// where a plain cap kept only the start. maxBytes must leave room for both.
class SessionLogTrim
{
public:
	SessionLogTrim(size_t headBytes, size_t tailBytes, size_t maxBytes)
		: headBytes(headBytes), tailBytes(tailBytes), maxBytes(maxBytes) { }

	// Records a line as written to the file, newline included. True when the
	// file has reached its budget and should be rewritten with Compacted().
	bool Note(const std::string &line)
	{
		fileBytes += line.size();
		if (head.size() < headBytes)
		{
			head += line;
			return fileBytes >= maxBytes;
		}
		tail.push_back(line);
		tailSize += line.size();
		while (tailSize > tailBytes && tail.size() > 1)
		{
			tailSize -= tail.front().size();
			tail.pop_front();
			++leftOut;
		}
		return fileBytes >= maxBytes;
	}

	// What the file holds after the rewrite; the budget counts on from there.
	std::string Compacted()
	{
		std::string out = head;
		out += "[" + std::to_string(leftOut) + " lines left out]\n";
		for (const std::string &line : tail)
			out += line;
		fileBytes = out.size();
		return out;
	}

private:
	size_t headBytes;
	size_t tailBytes;
	size_t maxBytes;
	std::string head;
	std::deque<std::string> tail;
	size_t tailSize = 0;
	size_t fileBytes = 0;
	uint64_t leftOut = 0;
};

} // namespace questcal
