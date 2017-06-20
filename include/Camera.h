/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#pragma once

#include <nanogui/glutil.h>
#include <nanogui/widget.h>
#include "BoundingBox.h"

namespace osr {
	namespace gui
	{

		//Provides a 3D camera with arcball interaction capabilities.
		class Camera
		{
		public:

			//Initialize the camera. parent is the widget onto which the scene is rendered and is only used to query its size.
			Camera(const nanogui::Widget& parent);

			//Computes model, view, and projection matrix for the current camera state. The model matrix contains the rotation,
			//scaling, and translation of the camera and is thus the actual view matrix.
			void ComputeCameraMatrices(
				Eigen::Matrix4f &model,
				Eigen::Matrix4f &view,
				Eigen::Matrix4f &proj,
				float customAspectRatio = 0) const;

			//Applies a zoom by scaling the scene. Positive values of amount increase object sizes.
			void Zoom(float amount);

			//Translates and zooms the camera in a way that it shows the entire bounding box while keeping the orientation.
			//The parameters are only estimated; it is not guaranteed that the bounding box actually fits in the viewport.
			void FocusOnBBox(const BoundingBox<float, 3>& bbox);

			//Forwarded mouse button event.
			bool HandleMouseButton(const Eigen::Vector2i &p, int button, bool down, int modifiers);
			//Forwarded mouse move event.
			bool HandleMouseMove(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers);
			//Forwarded resize event.
			void resize(const Eigen::Vector2i & s);

			struct CamParams
			{
				nanogui::Arcball arcball;

				float _zoom = 1.0f, fovy = 45.0f;
				float dnear = 0.05f, dfar = 100.0f;
				Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
				Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
				Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 5.0f);

				Eigen::Vector3f modelTranslation = Eigen::Vector3f::Zero();
				float modelZoom = 1.0f;
			};

			//Returns the current camera parameters
			const CamParams& saveParams() const { return params; }
			//Restores the camera parameters that have been previously saved.
			void restoreParams(const CamParams& params) { this->params = params; }

		private:
			const nanogui::Widget& parent;

			enum InteractionMode
			{
				None,
				Translate,
				Rotate
			} interactionMode = None;

			CamParams params;

			Eigen::Vector3f modelTranslation_start = Eigen::Vector3f::Zero();
			Eigen::Vector2i translation_start; //mouse position on the screen where translation started	
		};
	}
}