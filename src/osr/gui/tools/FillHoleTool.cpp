/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/gui/tools/FillHoleTool.h"
#include <nanogui/layout.h>
#include <nanogui/button.h>

#include "osr/gui/ShaderPool.h"
#include <nsessentials/util/IndentationLog.h>

#include <set>

using namespace osr;
using namespace osr::gui;
using namespace osr::gui::tools;

int FillHoleTool::scanNr(0);

FillHoleTool::FillHoleTool(nse::gui::AbstractViewer* viewer, DataGL& data, float& selectionRadius)
	: viewer(viewer), data(data), selectionRadius(selectionRadius),
	planePositionsBuffer(nse::gui::VertexBuffer)
{ }

void FillHoleTool::enterTool()
{
	//Set up the GUI
	window = new nanogui::Window(viewer, "Fill Holes");
	
	window->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 4, 4));

	lblStatus = new nanogui::Label(window, "Define the support region");

	auto calculateBasisBtn = new nanogui::Button(window, "Calculate Basis");
	calculateBasisBtn->setCallback([this]() { calculateBasis(); });
		
	auto resetBtn = new nanogui::Button(window, "Reset Tool");
	resetBtn->setCallback([this]() { resetSupport(); });

	viewer->performLayout();
	window->setPosition(Eigen::Vector2i(viewer->width() - 15 - window->width(), 15));
	viewer->performLayout();

	support.init();

	planeVAO.generate();
	planeVAO.bind();
	Eigen::Matrix<float, 3, 4> dummy;
	ShaderPool::Instance()->PhantomShader.bind();
	planePositionsBuffer.uploadData(dummy).bindToAttribute("position");
	planeVAO.unbind();

	state = State::DefineSupport;
}

float FillHoleTool::rbf(float distance)
{
	//bi-harmonic spline
	return distance * distance * std::log(distance);
}

float FillHoleTool::rbfDeriv(float distance)
{
	//bi-harmonic spline
	return distance + 2 * distance * std::log(distance);
}

void FillHoleTool::calculateBasis()
{
	//Find all points within the user selection
	std::set<THierarchy::VertexIndex> pointSet;
	for (int i = 1; i < support.positions.cols(); ++i)
	{
		data.hierarchy.findNearestPointsRadius(support.positions.block<3, 1>(0, i), support.positions.coeff(3, i), [&](const auto& point, float)
		{
			pointSet.insert(point);
		});
	}

	//Calculate the centroid and covariance of all included points
	centroid.setZero();
	for (auto& p : pointSet)
		centroid += data.hierarchy.attribute<Position>(p);
	centroid /= pointSet.size();
	Matrix3f covariance;
	covariance.setZero();
	float radiusSq = 0;
	Vector3f averageNormal; averageNormal.setZero();

	for (auto& p : pointSet)
	{
		auto& pos = data.hierarchy.attribute<Position>(p) - centroid;
		covariance += pos * pos.transpose();
		float rSq = pos.squaredNorm();
		if (radiusSq < rSq)
			radiusSq = rSq;
		averageNormal += data.hierarchy.attribute<Normal>(p);
	}

	//Compute the local tangent frame of the optimal plane
	eigenVectors = covariance.jacobiSvd(Eigen::ComputeFullU).matrixU();
	//Orient the plane such that its normal points in the same direction as the average point normal
	if (eigenVectors.col(2).dot(averageNormal) < 0)
		eigenVectors.col(2) *= -1;	

	//Find the centers of RBFs by approximate uniform sampling of the plane.
	rbfCenters.clear();
	std::map<std::pair<int, int>, int> cellOccupied; //Maps 2D plane coordinates (cells) to number of RBFs within the cell.
	for (auto& p : pointSet)
	{
		Vector3f q = eigenVectors.transpose() * (data.hierarchy.attribute<Position>(p) - centroid);
		auto cell = std::make_pair((int)std::floor(q.x() * data.meshSettings.scale() / 10.0f), (int)std::floor(q.y() * data.meshSettings.scale() / 10.0f));
		auto& cellEntry = cellOccupied[cell];
		if(cellEntry < 5) //Allow a maximum of 5 RBFs per cell
		{
			cellEntry++;
			rbfCenters.push_back(p);
		}
	}

	//Calculate the RBF weights
	Eigen::MatrixXd lhs(rbfCenters.size() + 3, rbfCenters.size() + 3);
	lhs.setZero();
	Eigen::Matrix<double, Eigen::Dynamic, 4> rhs(rbfCenters.size() + 3, 4);
	rhs.setZero();
#pragma omp parallel for
	for (int i = 0; i < rbfCenters.size(); ++i)
	{
		Vector3f p = eigenVectors.transpose() * (data.hierarchy.attribute<Position>(rbfCenters[i]) - centroid);
		for (int j = i + 1; j < rbfCenters.size(); ++j)
		{
			Vector3f q = eigenVectors.transpose() * (data.hierarchy.attribute<Position>(rbfCenters[j]) - centroid);
			double v = rbf((p.block<2, 1>(0, 0) - q.block<2, 1>(0, 0)).norm());
			lhs(i, j) = v;
			lhs(j, i) = v;
		}
		lhs(i, rbfCenters.size()) = 1;
		lhs(rbfCenters.size(), i) = 1;

		lhs(i, rbfCenters.size() + 1) = p.x();
		lhs(i, rbfCenters.size() + 2) = p.y();
		lhs(rbfCenters.size() + 1, i) = p.x();
		lhs(rbfCenters.size() + 2, i) = p.y();
		rhs(i, 0) = p.z();

		auto& color = data.hierarchy.attribute<Color>(rbfCenters[i]);
		rhs(i, 1) = color.x();
		rhs(i, 2) = color.y();
		rhs(i, 3) = color.z();
	}

	weights = lhs.colPivHouseholderQr().solve(rhs);	

	//Visualize the plane
	float r = sqrt(radiusSq);
	Eigen::Matrix<float, 3, 4> planePositions;
	planePositions.col(0) = centroid + eigenVectors * Vector3f(-r, -r, 0);
	planePositions.col(1) = centroid + eigenVectors * Vector3f(-r, +r, 0);
	planePositions.col(2) = centroid + eigenVectors * Vector3f(+r, -r, 0);
	planePositions.col(3) = centroid + eigenVectors * Vector3f(+r, +r, 0);
	planePositionsBuffer.uploadData(planePositions);

	state = AddPoints;
	lblStatus->setCaption("Add Points by Ctrl+LMB");

}

void FillHoleTool::addPoints(const Vector3f& center, float radius, int n)
{	
	if (n == 0)
		return;

	float radiusSq = radius * radius;

	Vector3f centerInPlaneSpace = eigenVectors.transpose() * (center - centroid);

	std::uniform_real_distribution<float> dist(-radius, radius);

	//Make more space for the new points in the data buffers
	int oldCount = scan->V().cols();
	if(scan->V().cols() == 0)
		scan->V().resize(3, n);
	else
		scan->V().conservativeResize(3, scan->V().cols() + n);

	if (scan->N().cols() == 0)
		scan->N().resize(3, n);
	else
		scan->N().conservativeResize(3, scan->N().cols() + n);

	if (scan->C().cols() == 0)
		scan->C().resize(3, n);
	else
		scan->C().conservativeResize(3, scan->C().cols() + n);

	//Evaluate the reconstructed height field at random sample locations
#pragma omp parallel for
	for (int i = 0; i < n; ++i)
	{
		bool valid = false;
		float x, y;
		while (!valid)
		{
			x = dist(rnd);
			y = dist(rnd);
			valid = x * x + y * y <= radiusSq;
		}
		x += centerInPlaneSpace.x();
		y += centerInPlaneSpace.y();

		//Reconstructed height field uses a linear function as its basis.
		Eigen::Matrix<double, 1, 4> solution = weights.row(rbfCenters.size()) + weights.row(rbfCenters.size() + 1) * x + weights.row(rbfCenters.size() + 2) * y;
		float dhdx = weights(rbfCenters.size() + 1);
		float dhdy = weights(rbfCenters.size() + 2);
		for (int j = 0; j < rbfCenters.size(); ++j)
		{
			auto localCoord = eigenVectors.transpose() * (data.hierarchy.attribute<Position>(rbfCenters[j]) - centroid);
			float dx = x - localCoord.x();
			float dy = y - localCoord.y();
			float r = sqrt(dx * dx + dy * dy);
			if (r < 0.001)
				continue;
			solution += weights.row(j) * rbf(r);
			dhdx += weights(j, 0) * rbfDeriv(r) * dx / r;
			dhdy += weights(j, 0) * rbfDeriv(r) * dy / r;
		}
		int myIdx = oldCount + i;
		scan->V().col(myIdx) = eigenVectors * Vector3f(x, y, (float)solution(0)) + centroid;
		scan->N().col(myIdx) = (eigenVectors * (Vector3f(1, 0, dhdx).cross(Vector3f(0, 1, dhdy)))).normalized();
		//repair colors
		for (int j = 1; j < 4; ++j)
		{
			if (solution.coeff(0, j) < 0) solution.coeffRef(0, j) = 0;
			if (solution.coeff(0, j) > 65535) solution.coeffRef(0, j) = 65535;
		}
		scan->C().col(myIdx) = solution.block<1, 3>(0, 1).cast<unsigned short>().transpose();
		if ((scan->V().col(myIdx) - center).squaredNorm() > radius * radius)
			scan->V().col(myIdx).setConstant(std::numeric_limits<float>::quiet_NaN());
	}
	scan->uploadData();
}

void FillHoleTool::exitTool()
{
	viewer->removeChild(window);
	window = nullptr;
}

void FillHoleTool::draw(const Matrix4f& mv, const Matrix4f& proj)
{
	Matrix4f mvp = proj * mv;

	if (state == AddingPoints)
	{
		scan->draw(mv, proj);
	}

	if (state != DefineSupport)
	{
		auto& shader = ShaderPool::Instance()->PhantomShader;
		shader.bind();
		planeVAO.bind();
		shader.setUniform("mvp", mvp);
		shader.setUniform("color", Vector4f(0.3f, 0.3f, 0.3f, 0.5f));
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		planeVAO.unbind();
	}	
	
	if (state == DefineSupport)
		support.draw(mv, proj, 0.5f);
	else
	{
		support.draw(mv, proj, 0.5f, 0, 1);
		support.draw(mv, proj, 0.25f, 1);

	}

	if (state == AddingPoints)
	{
		double circleFillAmount = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastPointsAdded).count() / 1000000.0 * fillRatePerSecond;
		int points = (int)(circleFillAmount * M_PI * support.positions.coeff(3, 0) * support.positions.coeff(3, 0) / (data.hierarchy.averagePointSpacing() * data.hierarchy.averagePointSpacing()));

		lastPointsAdded = std::chrono::high_resolution_clock::now();
		if (!std::isnan(support.positions.coeff(0, 0)))
			addPoints(support.positions.block<3, 1>(0, 0), support.positions.coeff(3, 0), points);
	}
}

bool FillHoleTool::mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers)
{
	if (down)
		pixelsMoved = 0;

	if (down && button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_CONTROL)
	{
		state = AddingPoints;
		scan = new Scan(Matrix3Xf(), Matrix3Xf(), Matrix3Xus(), MatrixXu(), "fill_" + std::to_string(scanNr++));
		scan->initialize();
		lastPointsAdded = std::chrono::high_resolution_clock::now();
		return true;
	}

	if (state == AddingPoints && !down)
	{
		data.AddScan(scan);
		state = AddPoints;
		return true;
	}

	if (pixelsMoved > 1)
		return false;

	if (!down && button == GLFW_MOUSE_BUTTON_1)
	{
		if (!std::isnan(support.positions.coeff(0, 0)))
		{
			if (state == DefineSupport)
			{
				support.addSphere(support.positions.col(0));
				return true;
			}
		}
	}

	return false;
}

bool FillHoleTool::mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers)
{
	if (p.x() < 0 || p.y() < 0)
		return false;

	pixelsMoved += rel.cwiseAbs().sum();

	if (state == DefineSupport)
	{
		support.setFirstToScreenPos(p, selectionRadius, viewer);
	}
	else
	{
		//ray/plane intersection

		Vector4f mouse(2.0f * (float)p.x() / viewer->width() - 1, -2.0f * p.y() / viewer->height() + 1, 0, 1);

		Eigen::Matrix4f model, view, proj;
		viewer->camera().ComputeCameraMatrices(model, view, proj);

		Matrix4f mvpInv = (proj * view * model).inverse();

		Vector4f mouseNear4 = mvpInv * mouse;
		Vector3f mouseNear = mouseNear4.block<3, 1>(0, 0) / mouseNear4.w();
		mouse.z() = 1;
		Vector4f mouseFar4 = mvpInv * mouse;
		Vector3f mouseFar = mouseFar4.block<3, 1>(0, 0) / mouseFar4.w();

		Vector3f dir = mouseFar - mouseNear;
		float t = (centroid - mouseNear).dot(eigenVectors.col(2)) / dir.dot(eigenVectors.col(2));
		support.positions.block<3, 1>(0, 0) = mouseNear + t * dir;
		support.positions(3, 0) = selectionRadius;

		support.uploadData();
	}	

	return false;
}

bool FillHoleTool::scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel)
{
	if (viewer->ctrlDown())
	{
		selectionRadius = std::max(0.01f, selectionRadius * (1 + 0.1f * rel.y()));
		support.positions.coeffRef(3, 0) = selectionRadius;
		support.uploadData();
		return true;
	}

	return false;
}

void FillHoleTool::resetSupport()
{
	support.positions.conservativeResize(4, 1);
	support.uploadData();
	state = DefineSupport;

	lblStatus->setCaption("Define the support region");
}
