#pragma once

#include <nanogui/window.h>
#include <nanogui/label.h>

#include "Data.h"
#include "AbstractViewer.h"
#include "Tool.h"

#include <boost/signals2.hpp>

//Represents a modification tool that calculates a translation to register a scan to the hierarchy by two-click interaction.
class ManualCoarseRegistrationTool : public Tool
{
public:
	ManualCoarseRegistrationTool(AbstractViewer* viewer, DataGL& data);

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

	AbstractViewer* viewer;
	nanogui::Window* window;
	nanogui::Label* lblStatus;

	Vector3f correspondenceScan;

	DataGL& data;

	Scan* scan;
};