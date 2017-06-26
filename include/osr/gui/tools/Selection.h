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

#include "osr/common.h"
#include "osr/gui/GLBuffer.h"
#include "osr/gui/GLVertexArray.h"
#include "osr/gui/AbstractViewer.h"

namespace osr {
	namespace gui {
		namespace tools
		{

			//Rendering helper that maintains a set of user-defined selection spheres.
			class Selection
			{
			public:
				Selection();

				void init();

				void draw(const Matrix4f& mv, const Matrix4f& proj, float opacity, int begin = 0, int count = -1, const Vector3f& color = Vector3f(1, 0.8f, 0), bool gradient = true, bool additive = true);

				void addSphere(const Vector3f& position, float radius);
				void addSphere(const Vector4f& positionRadius);
				void resize(size_t);

				Matrix4Xf positions;

				void setFirstToScreenPos(const Eigen::Vector2i& p, float radius, AbstractViewer* viewer);

				void uploadData();

			private:

				GLBuffer positionsBuffer;
				GLVertexArray vao;
			};

		}
	}
}