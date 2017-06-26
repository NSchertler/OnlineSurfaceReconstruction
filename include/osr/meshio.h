/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#pragma once

#include "osr/common.h"
#include "osr/Scan.h"
#include "osr/IndentationLog.h"

#include <boost/filesystem.hpp>
#include <fstream>

#ifndef _WIN32
#include <libgen.h>
#endif

namespace osr
{
	inline std::string str_tolower(std::string str)
	{
		std::transform(str.begin(), str.end(), str.begin(), ::tolower);
		return str;
	}

	// DataSink must have a public
	//   void AddScan(Scan*);
	template <typename DataSink>
	void load_scan(const std::string &filename, DataSink& dataSink, const Eigen::Matrix4f& transform = Eigen::Matrix4f::Identity(), bool ignoreUnknownFile = false);


	// ## Internals - not for public use ##

	extern void load_obj(const std::string &filename, MatrixXu &F, Matrix3Xf &V);

	extern void load_ply(const std::string &filename, MatrixXu &F, Matrix3Xf &V,
		Matrix3Xf &N, Matrix3Xus &C, bool pointcloud = false);

	extern void load_xyz(const std::string &filename, Matrix3Xf &V, Matrix3Xf &N);

	template <typename DataSink>
	void load_aln(const std::string &filename, DataSink& dataSink);

	template <typename DataSink>
	void load_frames(const std::string &filename, DataSink& dataSink);

	extern void write_mesh(const std::string &filename, const MatrixXu &F,
		const Matrix3Xf &V,
		const Matrix3Xf &N = Matrix3Xf(),
		const Matrix3Xf &Nf = Matrix3Xf(),
		const Matrix3Xf &UV = Matrix3Xf(),
		const Matrix3Xf &C = Matrix3Xf());

	extern void write_obj(const std::string &filename, const MatrixXu &F,
		const Matrix3Xf &V,
		const Matrix3Xf &N = Matrix3Xf(),
		const Matrix3Xf &Nf = Matrix3Xf(),
		const Matrix3Xf &UV = Matrix3Xf(),
		const Matrix3Xf &C = Matrix3Xf());

	extern void write_ply(const std::string &filename, const MatrixXu &F,
		const Matrix3Xf &V,
		const Matrix3Xf &N = Matrix3Xf(),
		const Matrix3Xf &Nf = Matrix3Xf(),
		const Matrix3Xf &UV = Matrix3Xf(),
		const Matrix3Xf &C = Matrix3Xf());


	// ---------- template implementation  ----------------

	template <typename DataSink>
	void load_scan(const std::string &filename, DataSink& dataSink, const Eigen::Matrix4f& transform, bool ignoreUnknownFile)
	{
		std::string extension;
		for (int i = 1; i < filename.length() - 1; ++i)
		{
			if (filename.size() > i && filename[filename.size() - i] == '.')
			{
				extension = str_tolower(filename.substr(filename.size() - i));
				break;
			}
		}

		if (extension == ".aln")
			load_aln(filename, dataSink);
		else if (extension == ".frames")
			load_frames(filename, dataSink);
		else
		{
			Matrix3Xf V, N;
			Matrix3Xus C;
			MatrixXu F;

			Scan* scan;
			std::string cachePath = filename + ".cache";
			boost::filesystem::path p(filename);

			if (boost::filesystem::exists(cachePath))
			{
				//load from cache

				FILE* f = fopen(cachePath.c_str(), "rb");
				size_t n;
				fread(&n, sizeof(size_t), 1, f);
				V.resize(3, n);
				fread(V.data(), sizeof(float), V.size(), f);

				fread(&n, sizeof(size_t), 1, f);
				N.resize(3, n);
				fread(N.data(), sizeof(float), N.size(), f);

				fread(&n, sizeof(size_t), 1, f);
				C.resize(3, n);
				fread(C.data(), sizeof(unsigned short), C.size(), f);
				fread(&n, sizeof(size_t), 1, f);
				F.resize(3, n);
				fread(F.data(), sizeof(uint32_t), F.size(), f);
				fclose(f);

				scan = new Scan(V, N, C, F, p.filename().string(), Eigen::Affine3f(transform));
			}
			else
			{
				if (extension == ".ply")
					load_ply(filename, F, V, N, C, false);
				else if (extension == ".obj")
					load_obj(filename, F, V);
				else if (extension == ".xyz" || extension == ".3d")
					load_xyz(filename, V, N);
				else
				{
					if (ignoreUnknownFile)
						return;
					else
						throw std::runtime_error("load_mesh_or_pointcloud: Unknown file extension.");
				}

				scan = new Scan(V, N, C, F, p.filename().string(), Eigen::Affine3f(transform));
				scan->calculateNormals();

				FILE* f = fopen(cachePath.c_str(), "wb");
				size_t n = V.cols();
				fwrite(&n, sizeof(size_t), 1, f);
				fwrite(V.data(), sizeof(float), V.size(), f);
				n = scan->N().cols();
				fwrite(&n, sizeof(size_t), 1, f);
				fwrite(scan->N().data(), sizeof(float), scan->N().size(), f);
				n = C.cols();
				fwrite(&n, sizeof(size_t), 1, f);
				fwrite(C.data(), sizeof(unsigned short), C.size(), f);
				n = F.cols();
				fwrite(&n, sizeof(size_t), 1, f);
				fwrite(F.data(), sizeof(uint32_t), F.size(), f);
				fclose(f);
			}

			dataSink.AddScan(scan);
		}
	}

	template <typename DataSink>
	void load_aln(const std::string& filename, DataSink& dataSink)
	{
		std::ifstream is(filename);
		if (is.fail())
			throw std::runtime_error("Unable to open ALN file \"" + filename + "\"!");

		Timer<> timer;
		std::istringstream line;

		auto fetch_line = [&]() {
			std::string line_str;
			do {
				std::getline(is, line_str);
				if (is.eof())
					throw std::runtime_error("Parser error while processing ALN file!");
			} while (line_str.empty() || line_str[0] == '#');
			line.clear();
			line.str(std::move(line_str));
		};

		auto fetch_string = [&](std::string &value) {
			while (!(line >> value)) {
				if (line.eof())
					fetch_line();
				else
					throw std::runtime_error("Parser error while processing ALN file!");
			}
		};
		auto fetch_uint = [&](uint32_t &value) {
			while (!(line >> value)) {
				if (line.eof())
					fetch_line();
				else
					throw std::runtime_error("Parser error while processing ALN file!");
			}
		};

		auto fetch_float = [&](Float &value) {
			while (!(line >> value)) {
				if (line.eof())
					fetch_line();
				else
					throw std::runtime_error("Parser error while processing ALN file!");
			}
		};

		uint32_t nFiles;
		fetch_uint(nFiles);
		std::string dir;

#if defined(_WIN32)
		char path_drive[_MAX_DRIVE];
		char path_dir[_MAX_DIR];
		char path_fname[_MAX_FNAME];
		char path_ext[_MAX_EXT];
		_splitpath(filename.c_str(), path_drive, path_dir, path_fname, path_ext);
		dir = std::string(path_drive) + std::string(path_dir);
#else
		char *path_dir = dirname((char *)filename.c_str());
		dir = std::string(path_dir);
#endif

		for (uint32_t i = 0; i < nFiles; ++i)
		{
			std::string filename_sub;
			fetch_string(filename_sub);
			MatrixXu F_sub;
			Matrix3Xf V_sub, N_sub;
			Eigen::Matrix<Float, 4, 4> M;
			for (uint32_t k = 0; k < 16; ++k)
				fetch_float(M.data()[k]);
			M.transposeInPlace();

			load_scan(dir + "/" + filename_sub, dataSink, M);
		}

		std::cout << "ALN loading finished. (took " << timeString(timer.value()) << ")" << std::endl;
	}

	template <typename DataSink>
	void load_frames(const std::string& filename, DataSink& dataSink)
	{
		//pair of .frames and .3d file
		std::string filename3D = filename.substr(0, filename.length() - 6) + "3d";

		Eigen::Matrix4f transform;
		std::ifstream framesFile(filename);
		framesFile
			>> transform(0, 0) >> transform(1, 0) >> transform(2, 0) >> transform(3, 0)
			>> transform(0, 1) >> transform(1, 1) >> transform(2, 1) >> transform(3, 1)
			>> transform(0, 2) >> transform(1, 2) >> transform(2, 2) >> transform(3, 2)
			>> transform(0, 3) >> transform(1, 3) >> transform(2, 3) >> transform(3, 3);

		load_scan(filename3D, dataSink, transform);
	}
}