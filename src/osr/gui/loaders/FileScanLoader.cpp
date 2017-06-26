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

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

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

	boost::filesystem::path p(filename);
	p = p.parent_path();
	for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(p), {}))
	{
		if (!boost::filesystem::is_directory(entry.path()))
		{
			if (entry.path().extension() == ".3d")
			{
				auto framesPath = entry.path();
				framesPath.replace_extension("frames");
				if(boost::filesystem::exists(framesPath))
					continue; //don't load a 3d file if there is an according frames file.		
			}
			TimedBlock b("Trying to load file " + entry.path().string());
			load_scan(entry.path().string(), *this, Eigen::Matrix4f::Identity(), true);
		}
	}
}

void FileScanLoader::AddScan(Scan* s)
{
	NewScan(s);
}