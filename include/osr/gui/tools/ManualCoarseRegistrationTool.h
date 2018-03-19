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

#include "osr/gui/DataGL.h"
#include "osr/gui/tools/Tool.h"

#include <nsessentials/gui/AbstractViewer.h>

#include <boost/signals2.hpp>

namespace osr {
	namespace gui {
		namespace tools
		{

			//Represents a modification tool that calculates a translation to register a scan to the hierarchy by two-click interaction.
			class ManualCoarseRegistrationTool : public Tool
			{
			public:
				ManualCoarseRegistrationTool(nse::gui::AbstractViewer* viewer, DataGL& data);

				//Sets the scan that should be registered.
				void setAffectedScan(Scan* s);

				void enterTool();
				void exitTool();

				void draw(const Matrix4f& mv, const Matrix4f& proj);

				bool mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers);

				boost::signals2::signal<void()> finished;

			private:

				enum State
				{
					ClickOnScan,
					ClickOnHierarchy,
				};

				State state;

				nse::gui::AbstractViewer* viewer;
				nanogui::Window* window;
				nanogui::Label* lblStatus;

				Vector3f correspondenceScan;

				DataGL& data;

				Scan* scan;
			};
		}
	}
}