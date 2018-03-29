#pragma once

#include <osr/Data.h>
#include "ExtractedMeshGL.h"

namespace osr
{
	namespace gui
	{
		//Specialization of the DataBase class that holds a renderable final mesh.
		class DataGL : public DataBase
		{
		public:
			ExtractedMeshGL extractedMesh;			

			DataGL();

		protected:
			ExtractedMesh * getExtractedMesh() { return &extractedMesh; }
			const ExtractedMesh* getExtractedMesh() const { return &extractedMesh; }
		};
	}
}