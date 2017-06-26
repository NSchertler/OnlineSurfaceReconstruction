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
#include "osr/gui/GLBuffer.h"
#include "osr/gui/GLVertexArray.h"
#include "osr/gui/tools/Tool.h"
#include "osr/gui/tools/Selection.h"

#include <nanogui/slider.h>
#include <nanogui/window.h>

#include <thread>

namespace osr {
	namespace gui {
		namespace tools
		{

			//Represents a modification tool that smooths points within a user-defined region.
			class SmoothTool : public Tool
			{
			public:
				SmoothTool(AbstractViewer* viewer, DataGL& data, float& selectionRadius);

				void enterTool();
				void exitTool();

				void draw(const Matrix4f& mv, const Matrix4f& proj);

				bool mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers);
				bool mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers);
				bool scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel);

			private:
				float& selectionRadius;

				DataGL& data;

				AbstractViewer* viewer;

				enum State
				{
					Idle,
					Smoothing,
				};

				State state;

				Selection selection;

				GLBuffer dirPositionBuffer;
				GLBuffer dirColorBuffer;
				GLVertexArray dirVAO;

				Scan* tempScan;
				std::unordered_map<THierarchy::VertexIndex, int> hierarchyIndexToInt;
				std::vector<THierarchy::VertexIndex> originalIndices;
				int indexOf(const THierarchy::VertexIndex& idx);

				void updateNeighbors();

				struct SelectedPoint
				{
					int index;
					float weight;
					std::vector<int> neighbors;
					Eigen::Matrix3f toLocal;
				};
				std::vector<SelectedPoint> selectedPoints;
				void updateSelectedPoints();
				void updateGuidanceDirection(const THierarchy::VertexIndex& referencePoint);
				void uploadGuidanceDirection();
				void smooth(float amount);
				Vector3f guidanceDirection, guidanceDirNormal;


				std::chrono::high_resolution_clock::time_point lastSmooth;

				nanogui::Window* window;
				nanogui::Slider *smoothAmountU, *smoothAmountV, *smoothAmountN;

				bool neighborsDirty;
				bool tempScanDirty;
				bool selectionDirty;
				bool guidanceDirectionDirty;
				std::thread* workerThread;
				void work();
				bool working;
				std::mutex tempScanMutex;
			};
		}
	}
}