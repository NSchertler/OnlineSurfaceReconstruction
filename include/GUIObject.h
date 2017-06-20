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

#include "common.h"

namespace osr {
	namespace gui
	{

		//Interface for any object with rendering and interaction capabilities.
		class GUIObject
		{
		public:
			virtual void draw(const Matrix4f& mv, const Matrix4f& proj) { };

			virtual bool mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers) { return false; };
			virtual bool mouseMotionEvent(const Eigen::Vector2i & p, const Eigen::Vector2i & rel, int button, int modifiers) { return false; };
			virtual bool scrollEvent(const Eigen::Vector2i & p, const Eigen::Vector2f & rel) { return false; };
		};
	}
}