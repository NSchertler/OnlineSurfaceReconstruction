#pragma once

#include "Tool.h"
#include "AbstractViewer.h"
#include "Data.h"
#include "Selection.h"

#include "GLBuffer.h"
#include "GLVertexArray.h"

#include <nanogui/slider.h>
#include <nanogui/window.h>

#include <thread>

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
