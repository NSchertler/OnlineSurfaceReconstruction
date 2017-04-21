#pragma once

#include "Data.h"

//Responsible for running the program in batch mode
class BatchSession
{
public:
	//Parses the given command line arguments and sets up all required states.
	void parseCommandLineArguments(int argc, char *argv[]);

	//Runs the program with the previously defined state.
	void run();

private:
	Data data;

	std::string inputFile;
	std::string outputFile;

	bool triangulate = false;
	float scaleMultiplier = 1.0f;
};