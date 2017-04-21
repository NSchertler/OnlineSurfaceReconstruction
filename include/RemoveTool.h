#pragma once

#include "Tool.h"
#include "Data.h"
#include "Selection.h"
#include "AbstractViewer.h"

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