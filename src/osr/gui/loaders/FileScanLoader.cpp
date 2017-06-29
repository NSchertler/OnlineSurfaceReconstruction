/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/gui/loaders/FileScanLoader.h"

#include <nanogui/button.h>
#include "osr/meshio.h"
#include "osr/filehelper.h"

using namespace osr;
using namespace gui;
using namespace loaders;

void FileScanLoader::setup(nanogui::Window* window)
{
	auto loadBtn = new nanogui::Button(window, "Load Data");
	loadBtn->setCallback([this]() { LoadData(); });

	auto loadDirectoryBtn = new nanogui::Button(window, "Load Directory");
	loadDirectoryBtn->setCallback([this]() { LoadDirectory(); });
}

void FileScanLoader::LoadData()
{
	std::string filename = nanogui::file_dialog({
		{ "obj", "Wavefront OBJ" },
		{ "ply", "Stanford PLY" },
		{ "xyz", "Raw point cloud" },
		{ "3d", "Raw point cloud" },
		{ "aln", "Aligned point cloud" },
		{ "frames", "Aligned point cloud" }
	}, false);
	if (filename == "")
		return;

	load_scan(filename, *this);
}

void FileScanLoader::LoadDirectory()
{
	std::string filename = nanogui::file_dialog({
		{ "obj", "Wavefront OBJ" },
		{ "ply", "Stanford PLY" },
		{ "xyz", "Raw point cloud" },
		{ "3d", "Raw point cloud" },
		{ "aln", "Aligned point cloud" },
		{ "frames", "Aligned point cloud" }
	}, false);

	std::string directory = osr::parent_path(filename);
	std::vector<std::string> files;
	osr::files_in_dir(directory, files);
	for (auto& entry : files)
	{
		if (!is_directory(entry))
		{
			if (extension(entry) == ".3d")
			{
				std::string framesPath = replace_extension(entry, "frames");
				if(file_exists(framesPath))
					continue; //don't load a 3d file if there is an according frames file.		
			}
			TimedBlock b("Trying to load file " + entry);
			load_scan(entry, *this, Eigen::Matrix4f::Identity(), true);
		}
	}
}

void FileScanLoader::AddScan(Scan* s)
{
	NewScan(s);
}