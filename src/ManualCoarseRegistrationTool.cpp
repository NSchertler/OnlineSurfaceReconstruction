/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "ManualCoarseRegistrationTool.h"

#include <nanogui/layout.h>

using namespace osr;
using namespace osr::gui;
using namespace osr::gui::tools;

ManualCoarseRegistrationTool::ManualCoarseRegistrationTool(AbstractViewer * viewer, DataGL & data)
	: viewer(viewer), data(data)
{
}

void ManualCoarseRegistrationTool::setAffectedScan(Scan * s)
{
	scan = s;
}

void ManualCoarseRegistrationTool::enterTool()
{
	if (scan == nullptr)
		throw std::runtime_error("There is no scan selected for this tool.");

	window = new nanogui::Window(viewer, "Manual Coarse Registration");

	window->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 4, 4));

	lblStatus = new nanogui::Label(window, "Select a point on the scan with Ctrl + LMB");	

	viewer->performLayout();
	window->setPosition(Eigen::Vector2i(viewer->width() - 15 - window->width(), 15));
	viewer->performLayout();

	state = ClickOnScan;
}

void ManualCoarseRegistrationTool::exitTool()
{
	viewer->removeChild(window);
	window = nullptr;
}

void ManualCoarseRegistrationTool::draw(const Matrix4f & mv, const Matrix4f & proj)
{
	if (state == ClickOnScan)
		scan->draw(mv, proj);
	if(state == ClickOnHierarchy)
		data.extractedMesh.draw(mv, proj);
}

bool ManualCoarseRegistrationTool::mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers)
{
	if (viewer->ctrlDown() && !down)
	{
		Vector3f mousePos;
		float depth = viewer->get3DPosition(p, mousePos);
		if (depth != 0 && depth != 1)
		{
			if (state == ClickOnScan)
			{
				correspondenceScan = mousePos;
				state = ClickOnHierarchy;
				lblStatus->setCaption("Select a point on the hierarchy with Ctrl + LMB");
			}
			else if (state == ClickOnHierarchy)
			{
				scan->transform() = Eigen::Translation3f(mousePos - correspondenceScan) * scan->transform();
				finished();
			}
		}
		return true;
	}
	return false;
}
