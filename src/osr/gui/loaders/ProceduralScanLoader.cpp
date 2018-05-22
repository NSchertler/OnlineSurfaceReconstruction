/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/gui/loaders/ProceduralScanLoader.h"

#include <nanogui/button.h>
#include <thread>

#include "zmq/zmqPub.h"

using namespace osr::gui;
using namespace loaders;

void ProceduralScanLoader::setup(nanogui::Window* window)
{
	auto loadBtn = new nanogui::Button(window, "Load Procedural");
	loadBtn->setCallback([this]() { LoadData(); });
}

void ProceduralScanLoader::LoadData()
{
	zmqPub::getInstance()->connect();

	static int it = 0;
	const int resolution = 50;

	Matrix3Xf V, N;

	float lowerX = 2 * it;
	float upperX = 2 * (it + 1);

	int nextPoint = 0;

	int upperI = 1.1f * resolution;

	int nPoints = resolution * 4 * upperI;
	if (it == 0)
		nPoints += (resolution + 1) * (resolution + 1) - resolution * 4;
	V.resize(Eigen::NoChange, nPoints);
	N.resize(Eigen::NoChange, nPoints);

	for (int i = 0; i < upperI; ++i)
	{
		float x = lowerX + i * (upperX - lowerX) / resolution;
		float angle = x / 3;
		float s = sin(angle);
		float c = cos(angle);
		Vector3f corners[4] = { Vector3f(x, c, s), Vector3f(x, -s, c), Vector3f(x, -c, -s), Vector3f(x, s, -c) };

		float xNext = lowerX + (i + 1) * (upperX - lowerX) / resolution;
		float angleNext = xNext / 3;
		float sNext = sin(angleNext);
		float cNext = cos(angleNext);
		Vector3f cornersNext[4] = { Vector3f(xNext, cNext, sNext), Vector3f(xNext, -sNext, cNext), Vector3f(xNext, -cNext, -sNext), Vector3f(xNext, sNext, -cNext) };

		if (x == 0)
		{
			for (int j = 0; j <= resolution; ++j)
			{
				for (int k = 0; k <= resolution; ++k)
				{
					float a = (float)j / resolution;
					float b = (float)k / resolution;
					V.col(nextPoint) = (1 - a) * ((1 - b) * corners[0] + b * corners[1]) + a * ((1 - b) * corners[3] + b * corners[2]);
					N.col(nextPoint) = Vector3f(-1, 0, 0);
					nextPoint++;
				}
			}
		}
		else
		{
			for (int j = 0; j < 4; ++j)
			{
				for (int k = 0; k < resolution; ++k)
				{
					float t = (float)k / resolution;
					V.col(nextPoint) = (1 - t) * corners[j] + t * corners[(j + 1) % 4];
					Vector3f vNextX = (1 - t) * cornersNext[j] + t * cornersNext[(j + 1) % 4];
					Vector3f tangentX = vNextX - V.col(nextPoint);
					if (k == 0)
					{
						int prev = j - 1;
						if (prev < 0) prev += 4;
						N.col(nextPoint) = (corners[(j + 1) % 4] - corners[prev]).cross(tangentX);
					}
					else
					{
						N.col(nextPoint) = (corners[(j + 1) % 4] - corners[j]).cross(tangentX);
					}
					nextPoint++;
				}
			}
		}
	}

	NewScan(new Scan(V, N, Matrix3Xus(), MatrixXu(), "procedural_" + std::to_string(it)));
	directIntegrate();
	// test sending matrix
	Eigen::Affine3f trackerMatrix = Eigen::Affine3f::Identity();
	std::vector<Eigen::Affine3f> matrixs;
	matrixs.push_back(trackerMatrix);
	zmqPub::getInstance()->send("m", matrixs);
	++it;
}

void osr::gui::loaders::ProceduralScanLoader::directIntegrate()
{
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	std::string autoItPath = "D:\\Program Files (x86)\\AutoIt3\\AutoIt3.exe"; //TODO: Generalize
	std::string command = "\"" + autoItPath + "\" D:\\Projects\\OnlineSurfaceReconstruction\\build_msvc14_64\\ClickIntegrateBtn.au3";
	system(command.c_str());
}
