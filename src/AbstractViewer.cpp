/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "AbstractViewer.h"

using namespace osr::gui;

AbstractViewer::AbstractViewer()
	: nanogui::Screen(Eigen::Vector2i(1280, 800), "Online Surface Reconstruction", true, false, 8, 8, 24, 8, 8),
	_camera(*this), _ctrlDown(false), _shiftDown(false)
{
	
}

bool AbstractViewer::keyboardEvent(int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_LEFT_CONTROL && action == 0)
		_ctrlDown = false;
	if (key == GLFW_KEY_LEFT_CONTROL && action == 1)
		_ctrlDown = true;

	if (key == GLFW_KEY_LEFT_SHIFT && action == 0)
		_shiftDown = false;
	if (key == GLFW_KEY_LEFT_SHIFT && action == 1)
		_shiftDown = true;

	return true;
}

bool AbstractViewer::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel)
{
	if (Screen::scrollEvent(p, rel))
		return true;

	if (scrollHook(p, rel))
		return true;

	if (!_ctrlDown && !_shiftDown)
	{
		_camera.Zoom(rel.y());
		return true;
	}

	return false;
}

bool AbstractViewer::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers)
{
	if (Screen::mouseButtonEvent(p, button, down, modifiers) && down)
		return true;

	if (mouseButtonHook(p, button, down, modifiers) && down)
		return true;

	return _camera.HandleMouseButton(p, button, down, modifiers);	
}

bool AbstractViewer::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel,
	int button, int modifiers)
{
	if (Screen::mouseMotionEvent(p, rel, button, modifiers))
		return true;

	if (mouseMotionHook(p, rel, button, modifiers))
		return true;

	return _camera.HandleMouseMove(p, rel, button, modifiers);
}

bool AbstractViewer::resizeEvent(const Eigen::Vector2i & s)
{
	Screen::resizeEvent(s);
	_camera.resize(s);
	return true;
}

float AbstractViewer::get3DPosition(const Eigen::Vector2i & screenPos, Eigen::Vector4f & pos)
{
	float depth;
	glReadPixels(screenPos.x(), height() - 1 - screenPos.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

	float ndcDepth = 2 * (depth - 0.5f);

	float x = 2 * ((float)screenPos.x() / width() - 0.5f);
	float y = 2 * ((float)-screenPos.y() / height() + 0.5f);

	Eigen::Matrix4f model, view, proj;
	camera().ComputeCameraMatrices(model, view, proj);

	Eigen::Matrix4f mvp = proj * view * model;
	Eigen::Matrix4f invMvp = mvp.inverse();

	pos = invMvp * Eigen::Vector4f(x, y, ndcDepth, 1);
	pos /= pos.w();

	return depth;
}

float AbstractViewer::get3DPosition(const Eigen::Vector2i & screenPos, Eigen::Vector3f & pos)
{
	Eigen::Vector4f pos4;
	float depth = get3DPosition(screenPos, pos4);
	pos.x() = pos4.x();
	pos.y() = pos4.y();
	pos.z() = pos4.z();
	return depth;
}
