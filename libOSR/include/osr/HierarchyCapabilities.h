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

namespace osr
{
	//Can be specifialized for a certain hierarchy to describe its capabilities.
	template <typename Hierarchy>
	struct OSR_EXPORT HierarchyCapabilities
	{
		// Determines if data from all hierarchy levels can be accessed. 
		// This requires the implementation of:
		//   - T vertices(int level), where level 0 specifies the finest level
		//   - size_t vertexCount(int level) const
		//   - int levels() const;
		static const bool AllowAccessToAllLevels = false;
	};
}