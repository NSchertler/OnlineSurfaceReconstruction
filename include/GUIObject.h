#pragma once

#include "common.h"

//Interface for any object with rendering and interaction capabilities.
class GUIObject
{
public:
	virtual void draw(const Matrix4f& mv, const Matrix4f& proj) { };

	virtual bool mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers) { return false; };
	virtual bool mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers) { return false; };
	virtual bool scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel) { return false; };
};