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

#include "osr/Data.h"
#include "osr/gui/AbstractViewer.h"
#include "osr/gui/tools/Tool.h"
#include "osr/gui/tools/Selection.h"

namespace osr {
	namespace gui {
		namespace tools
		{

			//Represents a modification tool that removes points within a user-defined region.
			class RemoveTool : public Tool
			{
			public:
				RemoveTool(AbstractViewer* viewer, DataGL& data, float& selectionRadius);

				void enterTool();
				void exitTool();

				void draw(const Matrix4f& mv, const Matrix4f& proj);

				bool mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers);
				bool mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers);
				bool scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel);

			private:
				float& selectionRadius;

				DataGL& data;

				Selection selection;

				int pixelsMoved;

				AbstractViewer* viewer;
			};
		}
	}
}