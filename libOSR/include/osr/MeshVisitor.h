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

#include <string>
#include <fstream>
#include <Eigen/Dense>

#include "osr/OSRLibrary.h"

namespace osr
{

	// Abstract base class for visitors that process an extracted mesh.
	class OSR_EXPORT MeshVisitor
	{
	public:
		virtual void begin(unsigned int vertices, unsigned int faces) = 0;
		virtual void addVertex(const Eigen::Vector3f& position, const Eigen::Vector3f& color) = 0;
		virtual void addFace(unsigned int count, const uint32_t* indices) = 0;
		virtual void end() = 0;
	};

	class OSR_EXPORT WritePLYMeshVisitor : public MeshVisitor
	{
	public:
		WritePLYMeshVisitor(const std::string& path);

		void begin(unsigned int vertices, unsigned int faces);
		void addVertex(const Eigen::Vector3f& position, const Eigen::Vector3f& color);
		void addFace(unsigned int count, const uint32_t* indices);
		void end();

	private:
		std::ofstream ply;
	};
}