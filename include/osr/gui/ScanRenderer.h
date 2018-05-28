#pragma once

#include <osr/Scan.h>

#include <nsessentials/gui/GLVertexArray.h>
#include <nsessentials/gui/GLBuffer.h>

namespace osr
{
	namespace gui
	{
		class ScanRenderer : public IScanRenderer
		{
		public:
			ScanRenderer();

			void initialize(Scan& scan);
			bool isInitialized() const { return inputMesh.valid(); }
			void updateData(const Scan& scan);
			void draw(const Scan& scan, const Eigen::Matrix4f & v, const Eigen::Matrix4f & proj) const;

		private:
			nse::gui::GLBuffer positionBuffer, normalBuffer, colorBuffer, indexBuffer;
			nse::gui::GLVertexArray inputMesh;

			int indexCount;
		};
	}
}