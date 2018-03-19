#pragma once

#include <osr/Data.h>
#include "ExtractedMeshGL.h"

namespace osr
{
	namespace gui
	{
		//Specialization of the DataBase class that holds a renderable final mesh.
		class DataGL : public DataBase<DataGL>
		{
		public:
			ExtractedMeshGL extractedMesh;

			DataGL();
		};
	}
}