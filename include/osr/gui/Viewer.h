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
#include "osr/Scan.h"
#include "osr/Optimizer.h"
#include "osr/HierarchyCapabilities.h"

#include <nsessentials/gui/AbstractViewer.h>
#include <nsessentials/gui/GLBuffer.h>
#include <nsessentials/gui/GLVertexArray.h>

#include "osr/gui/HierarchyRenderer.h"
#include "osr/gui/loaders/ScanLoader.h"

#include "osr/gui/tools/Tool.h"
#include "osr/gui/tools/FillHoleTool.h"
#include "osr/gui/tools/SmoothTool.h"
#include "osr/gui/tools/RemoveTool.h"
#include "osr/gui/tools/ManualCoarseRegistrationTool.h"

#ifdef USE_DAVIDVIVE
#include "osr/gui/loaders/DavidViveScanLoader.h"
#endif

#include <map>

namespace osr {
	namespace gui
	{

		//Main class for the GUI
		class Viewer : public nse::gui::AbstractViewer
		{
		public:
			Viewer();
			~Viewer();

			void drawContents();

			virtual bool mouseButtonHook(const Eigen::Vector2i & p, int button, bool down, int modifiers);
			virtual bool mouseMotionHook(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers);
			virtual bool scrollHook(const Eigen::Vector2i & p, const Eigen::Vector2f & rel);

		private:

			void SetupGUI();

			void SetupScanGUI(Scan * mesh);

			void ScanAdded(Scan * s);
			void ScanRemoved(Scan * s);

			void render(const Eigen::Matrix4f& mv, const Eigen::Matrix4f& proj);

			DataGL data;

			HierarchyRenderer hierarchyRenderer;

			nanogui::Window* mainWindow;

			nanogui::CheckBox* showExtractChk;

			std::map<Scan*, nanogui::Widget*> scanWidgets;

			template <typename Hierarchy, bool HierarchySupportsLevelAccess = HierarchyCapabilities<Hierarchy>::AllowAccessToAllLevels>
			struct HierarchySpecific
			{
				void addLevelWidget(nanogui::Widget* parent);

				HierarchySpecific(Viewer& viewer);
			};

			template <typename Hierarchy, bool HierarchySupportsLevelAccess>
			friend struct HierarchySpecific;

			nanogui::TextBox* levelTxt;
			nanogui::Widget* scanPanel;
			nanogui::CheckBox* showScans;
			nanogui::CheckBox *autoIntegrateChk;
			nanogui::CheckBox *addAllRegister, *addAllClean;

			std::vector<loaders::ScanLoader*> scanLoader;
			tbb::concurrent_queue<Scan*> scansToIntegrate;

			//Tools
			nanogui::Widget* toolsWidget;
			float selectionRadius;
			std::unique_ptr<tools::FillHoleTool> fillHoleTool;
			std::unique_ptr<tools::SmoothTool> smoothTool;
			std::unique_ptr<tools::RemoveTool> removeTool;
			std::unique_ptr<tools::ManualCoarseRegistrationTool> manualCoarseRegistrationTool;
			
			tools::Tool* selectedTool;

			const int screenshotWidth = 4096;
			const int screenshotHeight = 3072;
			GLuint screenshotFramebuffer;
			GLuint screenshotColorTexture;
			GLuint screenshotDepthTexture;

#ifdef USE_DAVIDVIVE
			loaders::DavidViveScanLoader* davidVive;
#endif

			void SetupToolGUI(nanogui::Widget* parent, int icon, const std::string& tooltip, tools::Tool* tool);			
		};
	}
}