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

#include "osr/HierarchyDef.h"
#include "osr/HierarchyCapabilities.h"

#include <nsessentials/gui/GLBuffer.h>
#include <nsessentials/gui/GLVertexArray.h>


namespace osr {
	namespace gui
	{
		struct GLBufferState
		{
			GLBufferState(nse::gui::GLBufferType type)
				: buffer(type), dirty(true)
			{}

			nse::gui::GLBuffer buffer;
			bool dirty;
		};

		//Renders points of a hierarchy.
		class HierarchyRenderer
		{
		public:
			HierarchyRenderer(THierarchy& hierarchy);

			void initialize();

			void draw(const Eigen::Matrix4f & mv, const Eigen::Matrix4f & proj, int rosy);

			//Specify the level of the hierarchy to draw. The hierarchy must support
			//level access for this.
			void setLevel(int);

			bool showInput, showNormals, showOrientationField, showPositionField, showAdjacency;

		private:
			void assertBuffersUpdated();

			THierarchy& hierarchy;

			size_t adjacencyEdges;

			GLBufferState positionBuffer, normalBuffer, orientationFieldBuffer, positionFieldBuffer, indexBuffer, adjBuffer, adjColorBuffer, colorBuffer;
			nse::gui::GLVertexArray inputData, orientationField, positionField, adjacency;

			template <typename Hierarchy, bool HierarchySupportsLevelAccess = HierarchyCapabilities<Hierarchy>::AllowAccessToAllLevels>
			struct HierarchySpecific
			{
				void updateVertexCount();

				template<Attribute A>
				void uploadDataToBuffer(GLBufferState & buffer);

				void uploadAdjacency();

				HierarchySpecific(HierarchyRenderer& renderer);
			};

			template <typename Hierarchy, bool HierarchySupportsLevelAccess>
			friend struct HierarchySpecific;


			int level;
			int vertexCount;
		};
	}
}