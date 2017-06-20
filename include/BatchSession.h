/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include "Data.h"

namespace osr
{

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
}