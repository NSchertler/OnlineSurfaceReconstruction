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

//Declares the currently used hierarchy.

#include "osr/HierarchyOptions.h"

#if UsedHierarchy == HMortonMultiPass
namespace osr
{
	namespace HierarchyMortonMultiPass
	{
		class Hierarchy;
	}
}
typedef osr::HierarchyMortonMultiPass::Hierarchy THierarchy;
#endif