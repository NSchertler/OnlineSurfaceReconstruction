#pragma once

#include "AbstractViewer.h"
#include "Scan.h"

#include "Data.h"
#include "GLBuffer.h"
#include "GLVertexArray.h"
#include "HierarchyRenderer.h"
#include "HierarchyCapabilities.h"
#include "ScanLoader.h"

#include "Tool.h"
#include "FillHoleTool.h"
#include "SmoothTool.h"
#include "RemoveTool.h"
#include "ManualCoarseRegistrationTool.h"

#include "Optimizer.h"

#ifdef USE_DAVIDVIVE
#include "DavidViveScanLoader.h"
#endif

#include <map>

//Main class for the GUI
class Viewer : public AbstractViewer
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

	std::vector<ScanLoader*> scanLoader;
	tbb::concurrent_queue<Scan*> scansToIntegrate;

	//Tools
	nanogui::Widget* toolsWidget;
	float selectionRadius;
	FillHoleTool* fillHoleTool;
	SmoothTool* smoothTool;
	RemoveTool* removeTool;
	ManualCoarseRegistrationTool* manualCoarseRegistrationTool;

	const int screenshotWidth = 4096;
	const int screenshotHeight = 3072;
	GLuint screenshotFramebuffer;
	GLuint screenshotColorTexture;
	GLuint screenshotDepthTexture;

#ifdef USE_DAVIDVIVE
	DavidViveScanLoader* davidVive;
#endif

	void SetupToolGUI(nanogui::Widget* parent, int icon, const std::string& tooltip, Tool* tool);

	Tool* selectedTool;
};