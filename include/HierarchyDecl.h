#pragma once

//Declares the currently used hierarchy.

#include "HierarchyOptions.h"

#if UsedHierarchy == HUnstructured
namespace HierarchyUnstructured
{
	class Hierarchy;
}
typedef HierarchyUnstructured::Hierarchy THierarchy;
#endif

#if UsedHierarchy == HRegularSubdiv
namespace HierarchyRegularSubdiv
{
	class Hierarchy;
}
typedef HierarchyRegularSubdiv::Hierarchy THierarchy;
#endif

#if UsedHierarchy == HOctree
namespace HierarchyOctree
{
	class Hierarchy;
}
typedef HierarchyOctree::Hierarchy THierarchy;
#endif

#if UsedHierarchy == HMortonMultiPass
namespace HierarchyMortonMultiPass
{
	class Hierarchy;
}
typedef HierarchyMortonMultiPass::Hierarchy THierarchy;
#endif