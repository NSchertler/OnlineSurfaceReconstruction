/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "Viewer.h"

#include "meshio.h"

#include <iostream>

#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/slider.h>
#include <nanogui/textbox.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/toolbutton.h>
#include <nanogui/entypo.h>
#include <nanogui/popupbutton.h>

#include <glad/glad.h>

#include <tbb/tbb.h>

#include <boost/filesystem.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "ShaderPool.h"
#include "IndentationLog.h"

#include "FileScanLoader.h"
#include "ProceduralScanLoader.h"

using namespace osr;
using namespace osr::gui;

template <typename Hierarchy>
struct Viewer::HierarchySpecific<Hierarchy, true>
{
	void addLevelWidget(nanogui::Widget* parent)
	{
		nanogui::Widget* levelPanel = new nanogui::Widget(parent);
		levelPanel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 10, 20));
		auto levelSlider = new nanogui::Slider(levelPanel);
		levelSlider->setFixedWidth(100);
		viewer.levelTxt = new nanogui::TextBox(levelPanel);
		viewer.levelTxt->setValue("0");
		viewer.levelTxt->setFixedSize(Vector2i(50, 25));
		auto viewerPtr = &viewer;
		levelSlider->setCallback([&, viewerPtr](float value)
		{
			if (viewerPtr->data.hierarchy.levels() == 0)
			{
				viewerPtr->levelTxt->setValue("0");
				viewerPtr->hierarchyRenderer.setLevel(0);
			}
			else
			{
				int level = value * viewerPtr->data.hierarchy.levels();
				if (level >= viewerPtr->data.hierarchy.levels())
					level = viewerPtr->data.hierarchy.levels() - 1;
				std::stringstream ss;
				ss << level;
				viewerPtr->levelTxt->setValue(ss.str());
				viewerPtr->hierarchyRenderer.setLevel(level);
			}
		});
	}

	HierarchySpecific(Viewer& viewer)
		: viewer(viewer)
	{ }

	Viewer& viewer;
};


template <typename Hierarchy>
struct Viewer::HierarchySpecific<Hierarchy, false>
{
	void addLevelWidget(nanogui::Widget* parent)
	{ }

	HierarchySpecific(Viewer& viewer)
	{ }
};

Viewer::Viewer()
	: mainWindow(nullptr), hierarchyRenderer(data.hierarchy)
{
	ShaderPool::Instance()->CompileAll();

	scanLoader.push_back(new loaders::FileScanLoader());
	scanLoader.push_back(new loaders::ProceduralScanLoader());
#ifdef USE_DAVIDVIVE
	davidVive = new loaders::DavidViveScanLoader(this);
	scanLoader.push_back(davidVive);
#endif		

	data.ScanAdded.connect(boost::bind(&Viewer::ScanAdded, this, _1));
	data.ScanRemoved.connect(boost::bind(&Viewer::ScanRemoved, this, _1));

	glGenFramebuffers(1, &screenshotFramebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, screenshotFramebuffer);
	glGenRenderbuffers(1, &screenshotColorTexture);
	glBindRenderbuffer(GL_RENDERBUFFER, screenshotColorTexture);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, screenshotWidth, screenshotHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, screenshotColorTexture);

	glGenRenderbuffers(1, &screenshotDepthTexture);
	glBindRenderbuffer(GL_RENDERBUFFER, screenshotDepthTexture);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, screenshotWidth, screenshotHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, screenshotDepthTexture);	

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	SetupGUI();

	hierarchyRenderer.initialize();

	selectionRadius = 0;

	for (auto& loader : scanLoader)
	{
		// The indirection via the scansToIntegrate queue is used to ensure that a valid OpenGL context
		// is active when scans are initialized (and their buffers a set up).
		loader->NewScan.connect([this](Scan* scan) { scansToIntegrate.push(scan); });
	}
}

Viewer::~Viewer()
{
	for (auto loader : scanLoader)
		delete loader;
}

void Viewer::SetupGUI()
{
	auto ctx = nvgContext();

	mainWindow = new nanogui::Window(this, "Online Surface Reconstruction");
	mainWindow->setPosition(Eigen::Vector2i(15, 15));
	mainWindow->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 4, 4));


	auto scaleWidget = new nanogui::Widget(mainWindow);
	scaleWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));	
	new nanogui::Label(scaleWidget, "Scale:");
	auto scaleTxt = new nanogui::TextBox(scaleWidget, "");
	auto updateScale = [this, scaleTxt](Float s) { std::stringstream ss; ss.precision(4); ss << data.meshSettings.scale(); scaleTxt->setValue(ss.str()); };
	updateScale(data.meshSettings.scale());
	data.meshSettings.ScaleChanged.connect(updateScale);
	scaleTxt->setCallback([this](const std::string& str) { try { data.meshSettings.setScale(std::stof(str)); return true; } catch (...) {} return false; });
	scaleTxt->setEditable(true);
	scaleTxt->setFixedWidth(90);	

	auto regErrorWidget = new nanogui::Widget(mainWindow);
	regErrorWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));
	new nanogui::Label(regErrorWidget, "Max Registration Error:");
	auto regErrorTxt = new nanogui::TextBox(regErrorWidget, "");
	auto updateRegError = [this, regErrorTxt](Float s) { std::stringstream ss; ss.precision(4); ss << data.meshSettings.maxRegistrationError(); regErrorTxt->setValue(ss.str()); };
	updateRegError(data.meshSettings.maxRegistrationError());
	data.meshSettings.RegistrationErrorChanged.connect(updateRegError);
	regErrorTxt->setCallback([this](const std::string& str) { try { data.meshSettings.setMaxRegistrationError(std::stof(str)); return true; } catch (...) {} return false; });
	regErrorTxt->setEditable(true);
	regErrorTxt->setFixedWidth(90);

	auto undoRegistrationBtn = new nanogui::Button(mainWindow, "Undo Last Registration");
	undoRegistrationBtn->setCallback([this]() {data.UndoLastRegistration(); });

	auto smoothnessWidget = new nanogui::Widget(mainWindow);
	new nanogui::Label(smoothnessWidget, "Smoothness:");
	smoothnessWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));
	auto smoothnessSlider = new nanogui::Slider(smoothnessWidget);
	smoothnessSlider->setFixedWidth(100);
	smoothnessSlider->setValue(data.meshSettings.smoothness);
	auto smoothnessTxt = new nanogui::TextBox(smoothnessWidget);
	smoothnessTxt->setFixedSize(Vector2i(50, 25));
	smoothnessSlider->setCallback([smoothnessTxt, this](float value)
	{
		std::stringstream ss;
		ss.precision(2);
		ss << value;
		smoothnessTxt->setValue(ss.str());
		data.meshSettings.smoothness = value;
	});
	smoothnessSlider->callback()(smoothnessSlider->value());

	auto advancedBtn = new nanogui::PopupButton(mainWindow, "Advanced Options");
	advancedBtn->popup()->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10, 4));

	auto saveBtn = new nanogui::Button(advancedBtn->popup(), "Save Project");
	saveBtn->setCallback([this]() 
	{
		std::string filename = nanogui::file_dialog({{ "osr", "Online Surface Reconstruction Project" }}, true);
		if (filename == "")
			return;
		if (filename.substr(filename.length() - 4) != ".osr")
			filename.append(".osr");
		data.saveToFile(filename);
	});
	auto chkFocus = new nanogui::CheckBox(advancedBtn->popup(), "Focus Camera on Project");
	chkFocus->setChecked(true);
	auto loadBtn = new nanogui::Button(advancedBtn->popup(), "Load Project");
	loadBtn->setCallback([this, chkFocus]()
	{
		std::string filename = nanogui::file_dialog({ { "osr", "Online Surface Reconstruction Project" } }, false);
		if (filename != "")
		{
			data.loadFromFile(filename);
			if(chkFocus->checked())
				_camera.FocusOnBBox(data.hierarchy.boundingBox());
			selectionRadius = 1.0f * data.meshSettings.scale();
		}
	});	
	auto screenshotBtn = new nanogui::Button(advancedBtn->popup(), "Take Screenshot");
	screenshotBtn->setCallback([this]()
	{
		std::string filename = nanogui::file_dialog({ { "png", "PNG" } }, true);
		if (filename == "")
			return;

		if (filename.substr(filename.length() - 4) != ".png")
			filename.append(".png");

		glBindFramebuffer(GL_FRAMEBUFFER, screenshotFramebuffer);

		glClearColor(1, 1, 1, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		glViewport(0, 0, screenshotWidth, screenshotHeight);
		Eigen::Matrix4f model, view, proj;
		_camera.ComputeCameraMatrices(model, view, proj, (float)screenshotWidth / screenshotHeight);

		Eigen::Matrix4f mv = view * model;

		render(mv, proj);

		std::vector<unsigned char> pixels(screenshotWidth * screenshotHeight * 4);
		glReadPixels(0, 0, screenshotWidth, screenshotHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

		glViewport(0, 0, width(), height());
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		
		stbi_write_png(filename.c_str(), screenshotWidth, screenshotHeight, 4, pixels.data() + (screenshotHeight - 1) * screenshotWidth * 4 * sizeof(unsigned char), -screenshotWidth * 4 * sizeof(unsigned char));
	});

	auto scanCutoffChk = new nanogui::CheckBox(advancedBtn->popup(), "Cut off scans vertically", [this](bool value) { data.meshSettings.scanCutoff = value; });
	scanCutoffChk->setChecked(data.meshSettings.scanCutoff);

	auto scanCutoffWidget = new nanogui::Widget(advancedBtn->popup());
	scanCutoffWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));
	new nanogui::Label(scanCutoffWidget, "Cut-off height:");
	auto scanCutoffTxt = new nanogui::TextBox(scanCutoffWidget, "");
	auto updateScanCutoff = [this, scanCutoffTxt](Float s) { std::stringstream ss; ss.precision(4); ss << data.meshSettings.scanCutoffHeight; scanCutoffTxt->setValue(ss.str()); };
	updateScanCutoff(data.meshSettings.scanCutoff);
	scanCutoffTxt->setCallback([this](const std::string& str) { try { data.meshSettings.scanCutoffHeight = std::stof(str); return true; } catch (...) {} return false; });
	scanCutoffTxt->setEditable(true);
	scanCutoffTxt->setFixedWidth(90);
	
	auto subsampleWidget = new nanogui::Widget(advancedBtn->popup());
	new nanogui::Label(subsampleWidget, "Subsampling of scans:");
	subsampleWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));
	auto subsampleSlider = new nanogui::Slider(subsampleWidget);
	subsampleSlider->setFixedWidth(100);
	subsampleSlider->setValue(data.meshSettings.scanSubsample);
	auto subsampleTxt = new nanogui::TextBox(subsampleWidget);
	subsampleTxt->setFixedSize(Vector2i(50, 25));
	subsampleSlider->setCallback([subsampleTxt, this](float value)
	{
		std::stringstream ss;
		ss << (int)std::round(value * 100) << "%";
		subsampleTxt->setValue(ss.str());
		data.meshSettings.scanSubsample = value;
	});
	subsampleSlider->callback()(subsampleSlider->value());

	addAllRegister = new nanogui::CheckBox(advancedBtn->popup(), "Register Scans When Integrating All");
	addAllClean = new nanogui::CheckBox(advancedBtn->popup(), "Clean Scans When Integrating All");
	addAllClean->setChecked(true);	

	auto optimizeButton = new nanogui::Button(advancedBtn->popup(), "Optimize All");
	optimizeButton->setCallback([this]() { data.hierarchy.optimizeFull(); });

	auto resetBtn = new nanogui::Button(advancedBtn->popup(), "Reset");
	resetBtn->setCallback([this]()
	{
		updateFocus(nullptr);

		data.Reset();
		selectionRadius = 0;

		for (auto widget : toolsWidget->children())
		{
			auto btn = dynamic_cast<nanogui::ToolButton*>(widget);
			if (btn && btn->pushed())
				btn->setPushed(false);

			if (selectedTool)
				selectedTool->exitTool();
			selectedTool = nullptr;
		}
	});

	for (auto& loader : scanLoader)
		loader->setup(mainWindow);

	auto inputChk = new nanogui::CheckBox(advancedBtn->popup(), "Show Input Point Cloud", [this](bool checked) { hierarchyRenderer.showInput = checked; });
	inputChk->setChecked(hierarchyRenderer.showInput);
	auto normalChk = new nanogui::CheckBox(advancedBtn->popup(), "Show Normals", [this](bool checked) { hierarchyRenderer.showNormals = checked; });
	normalChk->setChecked(hierarchyRenderer.showNormals);
	auto adjCheck = new nanogui::CheckBox(advancedBtn->popup(), "Show Adjacency", [this](bool checked) { hierarchyRenderer.showAdjacency = checked; });
	adjCheck->setChecked(hierarchyRenderer.showAdjacency);
	auto orientationChk = new nanogui::CheckBox(advancedBtn->popup(), "Show Orientation Field", [this](bool checked) { hierarchyRenderer.showOrientationField = checked; });
	orientationChk->setChecked(hierarchyRenderer.showOrientationField);
	auto positionChk = new nanogui::CheckBox(advancedBtn->popup(), "Show Position Field", [this](bool checked) { hierarchyRenderer.showPositionField = checked; });
	positionChk->setChecked(hierarchyRenderer.showPositionField);
	auto boundaryChk = new nanogui::CheckBox(advancedBtn->popup(), "Highlight Boundary", [this](bool checked) { data.extractedMesh.highlightBoundary = checked; });
	boundaryChk->setChecked(data.extractedMesh.highlightBoundary);

	auto saveCoarse = new nanogui::Button(advancedBtn->popup(), "Export Coarse Mesh");
	saveCoarse->setCallback([this]()
	{
		std::string filename = nanogui::file_dialog({ { "ply", "PLY model" } }, true);
		if (filename == "")
			return;

		if (filename.substr(filename.length() - 4) != ".ply")
			filename.append(".ply");

		data.extractedMesh.saveCoarseToPLY(filename);
	});

	auto saveWireframe = new nanogui::Button(advancedBtn->popup(), "Export Coarse Tessellated Wireframe");
	saveWireframe->setCallback([this]()
	{
		std::string filename = nanogui::file_dialog({ { "ply", "PLY model" } }, true);
		if (filename == "")
			return;

		if (filename.substr(filename.length() - 4) != ".ply")
			filename.append(".ply");

		data.extractedMesh.saveWireframeToPLY(filename);
	});

	auto saveFine = new nanogui::Button(advancedBtn->popup(), "Export Fine Mesh");
	saveFine->setCallback([this]()
	{
		std::string filename = nanogui::file_dialog({ { "ply", "PLY model" } }, true);
		if (filename == "")
			return;

		if (filename.substr(filename.length() - 4) != ".ply")
			filename.append(".ply");

		data.extractedMesh.saveFineToPLY(filename);
	});

	HierarchySpecific<THierarchy>(*this).addLevelWidget(mainWindow);	

	auto scanScroll = new nanogui::VScrollPanel(mainWindow);
	scanScroll->setFixedHeight(220);	
	scanPanel = new nanogui::Widget(scanScroll);
	scanPanel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10, 4));

	showScans = new nanogui::CheckBox(mainWindow, "Show Scans");
	showScans->setChecked(true);

	showExtractChk = new nanogui::CheckBox(mainWindow, "Show Extracted Mesh", [this](bool checked) { data.extractedMesh.drawExtracted = checked; });
	showExtractChk->setChecked(data.extractedMesh.drawExtracted);

	auto showWireframeChk = new nanogui::CheckBox(mainWindow, "... as wireframe", [this](bool checked) { data.extractedMesh.wireframe = checked; });
	showWireframeChk->setChecked(data.extractedMesh.wireframe);
	auto showCoarseWireframeChk = new nanogui::CheckBox(mainWindow, "Coarse Wireframe Overlay", [this](bool checked) { data.extractedMesh.coarseWireframe = checked; });
	showCoarseWireframeChk->setChecked(data.extractedMesh.coarseWireframe);

#ifdef DEBUG_VISUALIZATION
	auto showModified = new nanogui::CheckBox(advancedBtn->popup(), "Show Modified Data", [this](bool checked) { data.extractedMesh.drawModified = checked; });
	showModified->setChecked(data.extractedMesh.drawModified);

	auto showNewAdj = new nanogui::CheckBox(advancedBtn->popup(), "Show Adjacency of Added Data", [this](bool checked) { data.extractedMesh.drawAdjacency = checked; });
	showNewAdj->setChecked(data.extractedMesh.drawAdjacency);

	auto showCollapsed = new nanogui::CheckBox(advancedBtn->popup(), "Show Collapsed Extraction Graph", [this](bool checked) { data.extractedMesh.drawCollapsed = checked; });
	showCollapsed->setChecked(data.extractedMesh.drawCollapsed);
#endif

	auto integrateAllBtn = new nanogui::Button(mainWindow, "Integrate All");
	integrateAllBtn->setCallback([this]()
	{
		TimedBlock b("Integrating all scans ..");
		std::ofstream stats("integrationStats.csv");
		stats << "new points,optimized points,time (ms)" << std::endl;
		while(data.scans.size() > 0)
		{			
			if (data.hierarchy.vertexCount() > 0)
			{
				if (addAllRegister->checked())
					data.RegisterScan(data.scans[0]);
				if (addAllClean->checked())
					data.scans[0]->cleanOverlap(data.hierarchy, data.hierarchy.averagePointSpacing());
			}

			size_t totalNewPoints = data.scans[0]->V().cols();
			Timer<> timer;			
			data.IntegrateScan(data.scans[0]);
			size_t optimizedPoints = data.hierarchy.optimizedPoints;
			
			stats << totalNewPoints << "," << optimizedPoints << "," << timer.value() << std::endl;			
		}
		stats.close();
	});

	fillHoleTool = std::make_unique<tools::FillHoleTool>(this, data, selectionRadius);
	smoothTool = std::make_unique<tools::SmoothTool>(this, data, selectionRadius);
	removeTool = std::make_unique<tools::RemoveTool>(this, data, selectionRadius);
	manualCoarseRegistrationTool = std::make_unique<tools::ManualCoarseRegistrationTool>(this, data);
	manualCoarseRegistrationTool->finished.connect([this]() {selectedTool->exitTool(); selectedTool = nullptr; });

	toolsWidget = new nanogui::Widget(mainWindow);
	toolsWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));
	new nanogui::Label(toolsWidget, "Tools:    ");

	SetupToolGUI(toolsWidget, ENTYPO_ICON_CIRCLED_PLUS, "Fill Holes Tool", fillHoleTool.get());
	SetupToolGUI(toolsWidget, ENTYPO_ICON_CIRCLED_MINUS, "Delete Points Tool", removeTool.get());
	SetupToolGUI(toolsWidget, ENTYPO_ICON_FEATHER, "Smooth Tool", smoothTool.get());

	performLayout(ctx);

	for (auto mesh : data.scans)
	{
		SetupScanGUI(mesh);
	}	

	selectedTool = nullptr;
}

void Viewer::SetupToolGUI(nanogui::Widget* parent, int icon, const std::string& tooltip, tools::Tool* tool)
{
	auto toolBtn = new nanogui::ToolButton(parent, icon);
	toolBtn->setTooltip(tooltip);
	toolBtn->setChangeCallback(
		[this, tool](bool enabled)
	{
		if (enabled)
		{
			selectedTool = tool;
			selectedTool->enterTool();
		}
		else
		{
			tool->exitTool();
			if (selectedTool == tool)
				selectedTool = nullptr;
		}
	});
}

void Viewer::SetupScanGUI(Scan* scan)
{
	auto widget = new nanogui::Widget(scanPanel);
	widget->setLayout(new nanogui::GroupLayout(0, 4, 6, 10));
	scanWidgets.emplace(scan, widget);

	new nanogui::Label(widget, scan->getName());

	auto toolsWidget = new nanogui::Widget(widget);
	toolsWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));

	auto posBtn = new nanogui::Button(toolsWidget, "P");
	posBtn->setFlags(nanogui::Button::Flags::ToggleButton);
	posBtn->setFixedSize(Vector2i(25, 25));
	posBtn->setTooltip("Show Points");
	posBtn->setChangeCallback([scan](bool checked) { scan->showInput = checked; });
	posBtn->setPushed(scan->showInput);

	auto normalsBtn = new nanogui::Button(toolsWidget, "N");
	normalsBtn->setFlags(nanogui::Button::Flags::ToggleButton);
	normalsBtn->setFixedSize(Vector2i(25, 25));
	normalsBtn->setTooltip("Show Normals");
	normalsBtn->setChangeCallback([scan](bool checked) { scan->showNormals = checked; });
	normalsBtn->setPushed(scan->showNormals);

	auto btnCoarseReg = new nanogui::Button(toolsWidget, "MR");
	btnCoarseReg->setFixedSize(Vector2i(25, 25));
	btnCoarseReg->setTooltip("Manual Coarse Registration");
	btnCoarseReg->setCallback([this, scan]()
	{
		if (data.hierarchy.vertexCount() == 0)
			return; 
		manualCoarseRegistrationTool->setAffectedScan(scan);
		selectedTool = manualCoarseRegistrationTool.get();
		selectedTool->enterTool();
	});

	auto btnReg = new nanogui::Button(toolsWidget, "R");
	btnReg->setFixedSize(Vector2i(25, 25));
	btnReg->setTooltip("Fine-Register To Hierarchy");
	btnReg->setCallback([this, scan]() { if (data.hierarchy.vertexCount() == 0) return; data.RegisterScan(scan); });

	auto btnClean = new nanogui::Button(toolsWidget, "C");
	btnClean->setFixedSize(Vector2i(25, 25));
	btnClean->setTooltip("Clean Overlap With Hierarchy");
	btnClean->setCallback([this, scan]() { if (data.hierarchy.vertexCount() == 0) return; scan->cleanOverlap(data.hierarchy, data.hierarchy.averagePointSpacing()); });

	auto integrateBtn = new nanogui::Button(toolsWidget, "", ENTYPO_ICON_SQUARED_PLUS);
	integrateBtn->setFixedSize(Vector2i(25, 25));
	integrateBtn->setTooltip("Integrate Scan");
	integrateBtn->setCallback([scan, this]() 
	{
		data.IntegrateScan(scan); 
	});	

	auto deleteBtn = new nanogui::Button(toolsWidget, "", ENTYPO_ICON_SQUARED_MINUS);
	deleteBtn->setFixedSize(Vector2i(25, 25));
	deleteBtn->setTooltip("Delete Scan");
	deleteBtn->setCallback([scan, this]() { data.RemoveScan(scan); });
}

void Viewer::ScanAdded(Scan* s)
{
	s->initialize();

	if(selectedTool == nullptr && data.scans.size() == 1 && data.hierarchy.vertexCount() == 0)
		_camera.FocusOnBBox(s->getTransformedBoundingBox());
	
	if(selectionRadius == 0)
		selectionRadius = 1.0f * data.meshSettings.scale();		

	data.scans.push_back(s);		

	SetupScanGUI(data.scans.back());
	performLayout(nvgContext());
}

void Viewer::ScanRemoved(Scan * scan)
{
	updateFocus(nullptr);

	auto mapIt = scanWidgets.find(scan);
	if (mapIt != scanWidgets.end())
	{
		scanPanel->removeChild(mapIt->second);
		scanWidgets.erase(mapIt);
		performLayout(nvgContext());
	}
}

void Viewer::render(const Eigen::Matrix4f& mv, const Eigen::Matrix4f& proj)
{
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	if (selectedTool != manualCoarseRegistrationTool.get())
	{
		if (showScans->checked())
			for (auto mesh : data.scans)
				mesh->draw(mv, proj);


		data.extractedMesh.draw(mv, proj);

		hierarchyRenderer.draw(mv, proj, data.meshSettings.rosy->rosy());
	}

	for (auto loader : scanLoader)
		loader->draw(mv, proj);

	if (selectedTool)
		selectedTool->draw(mv, proj);
}

void Viewer::drawContents()
{	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	
	Eigen::Matrix4f model, view, proj;
	_camera.ComputeCameraMatrices(model, view, proj);

	Eigen::Matrix4f mv = view * model;

	render(mv, proj);

	//process all scans in the queue, nothing renderin-related
	Scan* scan;
	while (scansToIntegrate.try_pop(scan))
		data.AddScan(scan);
}

bool Viewer::mouseButtonHook(const Eigen::Vector2i & p, int button, bool down, int modifiers)
{
	for (auto loader : scanLoader)
		if (loader->mouseButtonEvent(p, button, down, modifiers))
			return true;

	if (selectedTool && selectedTool->mouseButtonEvent(p, button, down, modifiers) && down)
		return true;
	
	return false;
}

bool Viewer::mouseMotionHook(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers)
{
	for (auto loader : scanLoader)
		if (loader->mouseMotionEvent(p, rel, button, modifiers))
			return true;

	if (selectedTool && selectedTool->mouseMotionEvent(p, rel, button, modifiers))
		return true;	

	return false;
}

bool Viewer::scrollHook(const Eigen::Vector2i & p, const Eigen::Vector2f & rel)
{
	for (auto loader : scanLoader)
		if (loader->scrollEvent(p, rel))
			return true;

	if (selectedTool && selectedTool->scrollEvent(p, rel))
		return true;
	
	return false;
}