/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/gui/tools/RemoveTool.h"

using namespace osr;
using namespace osr::gui;
using namespace osr::gui::tools;

RemoveTool::RemoveTool(nse::gui::AbstractViewer* viewer, DataGL & data, float & selectionRadius)
	: viewer(viewer), data(data), selectionRadius(selectionRadius)
{ }

void RemoveTool::enterTool()
{
	selection.init();
}

void RemoveTool::exitTool()
{
}

void RemoveTool::draw(const Matrix4f & mv, const Matrix4f & proj)
{
	selection.draw(mv, proj, 0.5f);
}

bool RemoveTool::mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers)
{
	if (down)
		pixelsMoved = 0;

	if (pixelsMoved > 1)
		return false;

	if (!down && button == GLFW_MOUSE_BUTTON_1)
	{
		if (!std::isnan(selection.positions.coeff(0, 0)))
		{
			std::vector<THierarchy::VertexIndex> points;
			data.hierarchy.findNearestPointsRadius(selection.positions.block<3, 1>(0, 0), selection.positions.coeff(3, 0), [&](const THierarchy::VertexIndex& point, float)	{ points.push_back(point); });
			data.hierarchy.removePoints(points);
			return true;
		}
	}

	return false;
}

bool RemoveTool::mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers)
{
	if (p.x() < 0 || p.y() < 0)
		return false;

	pixelsMoved += rel.cwiseAbs().sum();

	selection.setFirstToScreenPos(p, selectionRadius, viewer);

	return false;
}

bool RemoveTool::scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel)
{
	if (viewer->ctrlDown())
	{
		selectionRadius = std::max(0.01f, selectionRadius * (1 + 0.1f * rel.y()));
		selection.positions.coeffRef(3, 0) = selectionRadius;
		selection.uploadData();
		return true;
	}

	return false;
}
