/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "BatchSession.h"

#include "meshio.h"

using namespace osr;

void BatchSession::parseCommandLineArguments(int argc, char * argv[])
{	
	for (int i = 1; i < argc; ++i)
	{
		try
		{
			if (strcmp(argv[i], "-i") == 0)
			{
				inputFile = argv[i + 1];
				++i;
			}
			else if (strcmp(argv[i], "-o") == 0)
			{
				outputFile = argv[i + 1];
				++i;
			}
			else if (strcmp(argv[i], "--sym") == 0)
			{
				int sym = atoi(argv[i + 1]); 
				++i;
				if (sym != 6 && sym != 4)
				{
					std::cerr << "Only symmetries of 4 or 6 are allowed. " << sym << " is not valid. Keeping the default value." << std::endl;
					continue;
				}				
				data.meshSettings.rosy = std::shared_ptr<IOrientationFieldTraits>(getOrientationFieldTraits(sym));
				data.meshSettings.posy = std::shared_ptr<IPositionFieldTraits>(getPositionFieldTraits(sym));
			}
			else if (strcmp(argv[i], "--scale") == 0)
			{
				float multiplier = atof(argv[i + 1]);
				++i;
				if (multiplier <= 0)
				{
					std::cerr << "Only positive scale multipliers are allowed. " << multiplier << " is not valid. Keeping the default value." << std::endl;
					continue;
				}
				scaleMultiplier = multiplier;
			}
			else if (strcmp(argv[i], "--smooth") == 0)
			{
				float smoothness = atof(argv[i + 1]);
				++i;
				if (smoothness < 0 || smoothness >= 1)
				{
					std::cerr << "Smoothness must be in the range [0, 1). " << smoothness << " is not valid. Keeping the default value." << std::endl;
					continue;
				}
				data.meshSettings.smoothness = smoothness;
			}
			else if (strcmp(argv[i], "--tri") == 0)
			{
				triangulate = true;
			}
		}
		catch (std::exception& e)
		{
			std::cerr << "Error reading command line parameter " << i << ": " << e.what() << std::endl;
		}
	}
}

void BatchSession::run()
{
	struct ScanDataSink
	{
		void AddScan(Scan* scan)
		{
			this->scan = scan;
		}

		Scan* scan = nullptr;
	} scanDataSink;
	
	std::cout << "Batch Processing." << std::endl;

	if (inputFile.length() == 0 || outputFile.length() == 0)
	{
		//show help
		std::cout << "Arguments:" << std::endl;
		std::cout << "  -i %path         Specify the path of the input file (.obj, .ply, .xyz)" << std::endl;
		std::cout << "  -o %path         Specify the path of the output file (.ply)" << std::endl;
		std::cout << "  --sym {4, 6}     Specify the field symmetry" << std::endl;
		std::cout << "  --scale %f       Specify a multiplier for the automatically determined scale" << std::endl;
		std::cout << "  --smooth [0, 1)  Specify the detail map smoothness" << std::endl;
		std::cout << "  --tri            Triangulate the final result" << std::endl;
		return;
	}

	std::cout << "---------------------------------------------" << std::endl;
	std::cout << "Input File: " << inputFile << std::endl;
	std::cout << "Output File: " << outputFile << std::endl;

	try
	{
		load_scan(inputFile, scanDataSink);
	}
	catch (std::exception& e)
	{
		std::cerr << "Error loading input file: " << e.what() << std::endl;
		return;
	}

	data.AddScan(scanDataSink.scan);
	data.meshSettings.setScale(scaleMultiplier * data.meshSettings.scale());
	
	std::cout << "Parameters: " << std::endl;
	std::cout << "RoSy: " << data.meshSettings.rosy->rosy() << std::endl;
	std::cout << "PoSy: " << data.meshSettings.posy->posy() << std::endl;
	std::cout << "Scale: " << data.meshSettings.scale() << " (using a user-provided multiplier of " << scaleMultiplier << ")" << std::endl;
	std::cout << "Smoothness: " << data.meshSettings.smoothness << std::endl;
	std::cout << "Triangulate result: " << (triangulate ? "true" : "false") << std::endl;
	std::cout << "---------------------------------------------" << std::endl;

	data.IntegrateScan(scanDataSink.scan);

	try
	{
		data.extractedMesh.saveFineToPLY(outputFile, triangulate);
	}
	catch (std::exception& e)
	{
		std::cerr << "Error writing output file: " << e.what() << std::endl;
		return;
	}
}
