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

#include <nanogui/window.h>
#include <nanogui/label.h>

#include "osr/common.h"
#include "osr/gui/tools/Tool.h"
#include "osr/gui/tools/Selection.h"
#include "osr/gui/DataGL.h"

#include <nsessentials/gui/GLBuffer.h>
#include <nsessentials/gui/GLVertexArray.h>
#include <nsessentials/gui/Camera.h>
#include <nsessentials/gui/AbstractViewer.h>

namespace osr {
	namespace gui {
		namespace tools
		{

			//Represents a modification tool that fills a hole in a point cloud by plane fitting and RBF-based reconstruction
			class FillHoleTool : public Tool
			{
			public:
				FillHoleTool(nse::gui::AbstractViewer* viewer, DataGL& data, float& selectionRadius);

				void enterTool();
				void exitTool();

				void draw(const Matrix4f& mv, const Matrix4f& proj);

				bool mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers);
				bool mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers);
				bool scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel);

			private:

				const double fillRatePerSecond = 0.6; //the percentage of the selection circle that is filled within a second

				enum State
				{
					DefineSupport,
					AddPoints,
					AddingPoints,
				};

				State state;

				//returns the value of the underlying RBF
				float rbf(float);
				//returns the derivative of the underlying RBF
				float rbfDeriv(float);

				//calculates the supporting plane and RBF weights for reconstruction
				void calculateBasis();

				//generates n points in the given sphere
				void addPoints(const Vector3f& center, float radius, int n);

				void resetSupport();

				nse::gui::AbstractViewer* viewer;
				nanogui::Window* window;
				nanogui::Label* lblStatus;

				Selection support;

				nse::gui::GLBuffer planePositionsBuffer;
				nse::gui::GLVertexArray planeVAO;

				float& selectionRadius;

				DataGL& data;

				//Control points for the RBFs (subset of selected points)
				std::vector<THierarchy::VertexIndex> rbfCenters;
				Eigen::Matrix<double, Eigen::Dynamic, 4> weights;
				Vector3f centroid;
				Matrix3f eigenVectors;

				int pixelsMoved;

				std::chrono::high_resolution_clock::time_point lastPointsAdded;

				std::mt19937 rnd;
				Scan* scan;

				static int scanNr;
			};
		}
	}
}