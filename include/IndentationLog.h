#pragma once

#include <vector>
#include <iostream>
#include <string>
#include "Timer.h"

//Alternative stream buffer that indents all output by a specified amount.
class IndentationLog : public std::streambuf
{
public:

	IndentationLog(std::ostream& dest);

	//Starts a block, such that all further output is indented.
	void startBlock(const std::string& message);

	//Ends an indentation block
	void endBlock();

protected:
	virtual int overflow(int ch);

private:
	std::streambuf*     dest;
	bool                isAtStartOfLine;
	std::ostream&       owner;
	std::string indent;

	struct BlockInfo
	{
		BlockInfo()
			: hasContent(false)
		{ }

		bool hasContent;
		Timer<> t;
	};

	bool internalOutput;
	std::vector<BlockInfo> blocks;
};
extern IndentationLog ilog;

//RAII style object that indents all output during its lifetime. Furthermore,
//measures the time until destruction and outputs it.
struct TimedBlock
{
	//s - Initial output of the block (usually used to name the current operation).
	//highPriority - if REDUCED_TIMINGS is defined, omits all blocks with highPriority=false
	TimedBlock(const std::string& s, bool highPriority = false);
	~TimedBlock();

	void earlyExit();
private:
	bool exited;
};