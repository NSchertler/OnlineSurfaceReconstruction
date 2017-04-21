#include "SmoothTool.h"

#include <nanogui/layout.h>
#include <nanogui/label.h>

#include "ShaderPool.h"

SmoothTool::SmoothTool(AbstractViewer * viewer, DataGL & data, float & selectionRadius)
	: viewer(viewer), data(data), selectionRadius(selectionRadius), tempScan(nullptr), dirPositionBuffer(VertexBuffer), dirColorBuffer(VertexBuffer)
{
}

void SmoothTool::enterTool()
{
	state = Idle;

	selection.init();
	selection.addSphere(Vector4f(0, 0, 0, 0.75 * selectionRadius));

	window = new nanogui::Window(viewer, "Smooth");

	window->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 4, 4));

	auto uWidget = new nanogui::Widget(window);
	uWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 5, 20));
	new nanogui::Label(uWidget, "Smooth Amount U");
	smoothAmountU = new nanogui::Slider(uWidget);
	smoothAmountU->setFixedWidth(100);
	smoothAmountU->setValue(1.0f);

	auto vWidget = new nanogui::Widget(window);
	vWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 5, 20));
	new nanogui::Label(vWidget, "Smooth Amount V");
	smoothAmountV = new nanogui::Slider(vWidget);
	smoothAmountV->setFixedWidth(100);
	smoothAmountV->setValue(1.0f);

	auto nWidget = new nanogui::Widget(window);
	nWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 5, 20));
	new nanogui::Label(nWidget, "Smooth Amount N");
	smoothAmountN = new nanogui::Slider(nWidget);
	smoothAmountN->setFixedWidth(100);
	smoothAmountN->setValue(1.0f);

	viewer->performLayout();
	window->setPosition(Eigen::Vector2i(viewer->width() - 15 - window->width(), 15));
	viewer->performLayout();

	dirVAO.generate();
	dirVAO.bind();
	auto& shader = ShaderPool::Instance()->AdjacencyShader;
	shader.bind();
	Matrix3Xf dummy(3, 2);
	dirPositionBuffer.uploadData(dummy).bindToAttribute("position");
	dirColorBuffer.uploadData(dummy).bindToAttribute("color");
	dirVAO.unbind();

	guidanceDirection.setZero();

	working = true;
	selectionDirty = false;
	workerThread = new std::thread([this]() { work(); });
}

void SmoothTool::exitTool()
{
	working = false;
	workerThread->join();
	delete workerThread;

	viewer->removeChild(window);
	window = nullptr;
}

void SmoothTool::work()
{
	while (working)
	{
		float amount = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastSmooth).count() / 1000000.0f * 8.0f;
		lastSmooth = std::chrono::high_resolution_clock::now();

		if (state == Smoothing)
		{
			smooth(amount);
		}

		std::this_thread::sleep_until(lastSmooth + std::chrono::milliseconds(50));
	}
}

void SmoothTool::draw(const Matrix4f & mv, const Matrix4f & proj)
{
	if (state == Smoothing)
	{
		if (tempScanDirty)
		{
			std::lock_guard<std::mutex> lock(tempScanMutex);
			tempScan->uploadData();
			tempScanDirty = false;
		}
		tempScan->draw(mv, proj);
	}
		

	selection.draw(mv, proj, 0.25f, 0, 1);	
	selection.draw(mv, proj, 0.5f, 1, 1);
	if (!std::isnan(selection.positions.coeff(0, 0)))
	{
		if (guidanceDirectionDirty)
			uploadGuidanceDirection();
		dirVAO.bind(); 
		auto& shader = ShaderPool::Instance()->AdjacencyShader;
		shader.bind();
		Matrix4f mvp = proj * mv;
		shader.setUniform("mvp", mvp);
		glDrawArrays(GL_LINES, 0, 2);
		dirVAO.unbind();
	}
}

void SmoothTool::smooth(float amount)
{
	std::lock_guard<std::mutex> lock(tempScanMutex);

	if(selectionDirty)
		updateSelectedPoints();
	if (neighborsDirty)
		updateNeighbors();

	Matrix3Xf V2(3, selectedPoints.size());
	Matrix3Xf N2(3, selectedPoints.size());

#pragma omp parallel for
	for (int _i = 0; _i < selectedPoints.size(); ++_i)
	{
		SelectedPoint& point = selectedPoints[_i];
		Vector3f p; p.setZero();
		Vector3f n; n.setZero();
		float sumWeight = 0;
		for (auto j : point.neighbors)
		{
			Vector3f d = (tempScan->V().col(j) - tempScan->V().col(point.index));
			float weight = exp(-d.squaredNorm() / (selection.positions.coeff(3, 1) * selection.positions.coeff(3, 1)));
			p += weight * tempScan->V().col(j);
			n += weight * tempScan->N().col(j);
			sumWeight += weight;
		}
		if (sumWeight > 0)
		{
			Vector3f dp = point.toLocal * (p / sumWeight - tempScan->V().col(point.index));
			dp.x() *= smoothAmountU->value();
			dp.y() *= smoothAmountV->value();
			dp.z() *= smoothAmountN->value();
			dp = point.toLocal.transpose() * dp;
			V2.col(_i) = tempScan->V().col(point.index) + dp * amount * point.weight;
			N2.col(_i) = tempScan->N().col(point.index) + (n / sumWeight - tempScan->N().col(point.index)) * amount * point.weight;
		}
	}

#pragma omp parallel for
	for (int _i = 0; _i < selectedPoints.size(); ++_i)
	{
		int i = selectedPoints[_i].index;
		tempScan->V().col(i) = V2.col(_i);
		tempScan->N().col(i) = N2.col(_i);
		tempScan->N().col(i).normalize();
	}
	tempScanDirty = true;
}

bool SmoothTool::mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers)
{
	if (down && button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_CONTROL)
	{		
		tempScan = new Scan();
		tempScan->initialize();
		selectionDirty = true;
		state = Smoothing;
		return true;
	}

	if (state == Smoothing && !down)
	{
		state = Idle;
		std::lock_guard<std::mutex> lock(tempScanMutex);
		data.hierarchy.modifyPoints(originalIndices, tempScan->V(), tempScan->N(), tempScan->C());

		hierarchyIndexToInt.clear();
		originalIndices.clear();
		delete tempScan;
		tempScan = nullptr;
		state = Idle;
		return true;
	}

	return false;
}

bool SmoothTool::mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers)
{
	if (p.x() < 0 || p.y() < 0)
		return false;

	selection.setFirstToScreenPos(p, selectionRadius, viewer);
	selection.positions.block<3, 1>(0, 1) = selection.positions.block<3, 1>(0, 0);

	if (state == Smoothing)
	{
		selectionDirty = true;
	}
	else
	{
		Vector3f center = selection.positions.block<3, 1>(0, 0);
		float radius = selection.positions.coeff(3, 1);
		if (!std::isnan(center.x()))
		{
			float minDistSq = std::numeric_limits<float>::infinity();
			THierarchy::VertexIndex closestPoint;
			data.hierarchy.findNearestPointsRadius(center, radius, [&](const auto& point, float d)
			{
				auto& p = data.hierarchy.attribute<Position>(point);
				if (d < minDistSq)
				{
					closestPoint = point;
					minDistSq = d;
				}
			});

			if (minDistSq < std::numeric_limits<float>::infinity())
				updateGuidanceDirection(closestPoint);
			uploadGuidanceDirection();
		}
	}
		

	return false;
}

bool SmoothTool::scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel)
{
	if (viewer->ctrlDown())
	{
		if (viewer->shiftDown())
		{
			selection.positions.coeffRef(3, 1) = std::max(0.01f, selection.positions.coeff(3, 1) * (1 + 0.1f * rel.y()));
			selection.uploadData();
			return true;
		}
		else
		{
			selectionRadius = std::max(0.01f, selectionRadius * (1 + 0.1f * rel.y()));
			selection.positions.coeffRef(3, 0) = selectionRadius;
			selection.uploadData();
			return true;
		}		
	}

	if (viewer->shiftDown())
	{
		guidanceDirection = Eigen::AngleAxisf(2 * 3.14159f / data.meshSettings.rosy->rosy(), guidanceDirNormal) * guidanceDirection;
		uploadGuidanceDirection();
	}

	return false;
}

int SmoothTool::indexOf(const THierarchy::VertexIndex & idx)
{
	auto it = hierarchyIndexToInt.find(idx);
	if (it == hierarchyIndexToInt.end())
	{
		tempScan->V().conservativeResize(3, tempScan->V().cols() + 1);
		tempScan->N().conservativeResize(3, tempScan->V().cols() + 1);
		tempScan->C().conservativeResize(3, tempScan->V().cols() + 1);

		int intIdx = tempScan->V().cols() - 1;

		tempScan->V().col(intIdx) = data.hierarchy.attribute<Position>(idx);
		tempScan->N().col(intIdx) = data.hierarchy.attribute<Normal>(idx);
		tempScan->C().col(intIdx) = data.hierarchy.attribute<Color>(idx);

		hierarchyIndexToInt[idx] = intIdx;
		originalIndices.push_back(idx);

		return intIdx;
	}
	return it->second;
}

void SmoothTool::updateSelectedPoints()
{
	selectedPoints.clear();
	Vector3f center = selection.positions.block<3, 1>(0, 0);
	float radius = selection.positions.coeff(3, 0);

	float minDistSq = std::numeric_limits<float>::infinity();
	THierarchy::VertexIndex closestPoint;
	data.hierarchy.findNearestPointsRadius(center, radius, [&](const auto& point, float d)
	{
		selectedPoints.emplace_back();
		selectedPoints.back().index = this->indexOf(point);
		auto& p = data.hierarchy.attribute<Position>(point);
		if (d < minDistSq)
		{
			closestPoint = point;
			minDistSq = d;
		}
	});

	if (selectedPoints.size() > 0)
		updateGuidanceDirection(closestPoint);	


	neighborsDirty = true;
	selectionDirty = false;
}

void SmoothTool::updateNeighbors()
{
	tempScanDirty = true;
	neighborsDirty = false;

	if (selectedPoints.size() == 0)
		return;

	Vector3f center = selection.positions.block<3, 1>(0, 0);
	float radiusSelection = selection.positions.coeff(3, 0);
	float radiusKernel = selection.positions.coeff(3, 1);

#pragma omp parallel
	{
		std::vector<THierarchy::VertexIndex> neighbors;
		std::vector<int> pointsForCurrentThread;
		pointsForCurrentThread.reserve((int)ceil((float)selectedPoints.size() / omp_get_num_procs()));		

		int neighborsPerPointGuess = 20;

		auto processPoint = [&](int i)
		{
			pointsForCurrentThread.push_back(i);

			auto& point = selectedPoints[i];
			auto& p = data.hierarchy.attribute<Position>(originalIndices[point.index]);
			float d = exp(-5.0f * (p - center).squaredNorm() / (radiusSelection * radiusSelection));

			Matrix3Xf principalDirections(3, data.meshSettings.rosy->rosy() / 2);
			data.meshSettings.rosy->getPrincipalDirections(data.hierarchy.attribute<DirField>(originalIndices[point.index]), data.hierarchy.attribute<Normal>(originalIndices[point.index]), principalDirections);
			float bestScore = std::abs(guidanceDirection.dot(principalDirections.col(0)));
			int best = 0;
			for (int i = 1; i < principalDirections.cols(); ++i)
			{
				float score = std::abs(guidanceDirection.dot(principalDirections.col(i)));
				if (score > bestScore)
				{
					best = i;
					bestScore = score;
				}
			}
			point.toLocal.row(0) = principalDirections.col(best);
			point.toLocal.row(2) = data.hierarchy.attribute<Normal>(originalIndices[point.index]);
			point.toLocal.row(1) = point.toLocal.row(2).cross(point.toLocal.row(0));

			point.weight = d;
			point.neighbors.clear();
			point.neighbors.reserve(neighborsPerPointGuess);
			data.hierarchy.findNearestPointsRadius(p, radiusKernel, [&](const auto& n, float) { point.neighbors.push_back(neighbors.size()); neighbors.push_back(n); });
		};
#pragma omp single
		{
			processPoint(0);
			neighborsPerPointGuess = 1.5 * selectedPoints[0].neighbors.size();
			neighbors.reserve(neighborsPerPointGuess * pointsForCurrentThread.capacity());
		}
		
#pragma omp for
		for (int i = 1; i < selectedPoints.size(); ++i)
		{
			processPoint(i);
		}
#pragma omp critical
		{
			for (int i = 0; i < pointsForCurrentThread.size(); ++i)
			{
				auto& point = selectedPoints[pointsForCurrentThread[i]];
				for (int j = 0; j < point.neighbors.size(); ++j)
				{
					point.neighbors[j] = indexOf(neighbors[point.neighbors[j]]);
				}
				point.neighbors.shrink_to_fit();
			}
		}
	}
}

void SmoothTool::updateGuidanceDirection(const THierarchy::VertexIndex& referencePoint)
{	
	if (guidanceDirection.isZero())
		guidanceDirection = data.hierarchy.attribute<DirField>(referencePoint);
	else
	{
		Matrix3Xf principalDirections(3, data.meshSettings.rosy->rosy() / 2);
		data.meshSettings.rosy->getPrincipalDirections(data.hierarchy.attribute<DirField>(referencePoint), data.hierarchy.attribute<Normal>(referencePoint), principalDirections);
		float bestScore = std::abs(guidanceDirection.dot(principalDirections.col(0)));
		int best = 0;
		for (int i = 1; i < principalDirections.cols(); ++i)
		{
			float score = std::abs(guidanceDirection.dot(principalDirections.col(i)));
			if (score > bestScore)
			{
				best = i;
				bestScore = score;
			}
		}
		guidanceDirection = principalDirections.col(best);
		guidanceDirNormal = data.hierarchy.attribute<Normal>(referencePoint);
	}	

	guidanceDirectionDirty = true;
}

void SmoothTool::uploadGuidanceDirection()
{
	Vector3f center = selection.positions.block<3, 1>(0, 0);
	float radius = selection.positions.coeff(3, 0);

	Matrix3Xf dirPos(3, 2);
	Matrix3Xf dirCol(3, 2);
	dirPos.col(0) = center + 0.02f * radius * guidanceDirNormal - 0.5f * radius * guidanceDirection;
	dirPos.col(1) = center + 0.02f * radius * guidanceDirNormal + 0.5f * radius * guidanceDirection;
	dirCol.col(0) = Vector3f::UnitX();
	dirCol.col(1) = Vector3f::UnitX();
	dirPositionBuffer.uploadData(dirPos);
	dirColorBuffer.uploadData(dirCol);

	guidanceDirectionDirty = false;
}
