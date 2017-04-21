#pragma once

//Declares and defines the currently used hierarchy.

#include "HierarchyOptions.h"

#if UsedHierarchy == HUnstructured
#include "HierarchyUnstructured.h"
typedef HierarchyUnstructured::Hierarchy THierarchy;
#endif

#if UsedHierarchy == HRegularSubdiv
#include "HierarchyRegularSubdiv.h"
typedef HierarchyRegularSubdiv::Hierarchy THierarchy;
#endif

#if UsedHierarchy == HOctree
#include "HierarchyOctree.h"
typedef HierarchyOctree::Hierarchy THierarchy;
#endif

#if UsedHierarchy == HMortonMultiPass
#include "HierarchyMortonMultiPass.h"
typedef HierarchyMortonMultiPass::Hierarchy THierarchy;
#endif