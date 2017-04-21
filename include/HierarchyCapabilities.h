#pragma once

//Can be specifialized for a certain hierarchy to describe its capabilities.
template <typename Hierarchy>
struct HierarchyCapabilities
{
	// Determines if data from all hierarchy levels can be accessed. 
	// This requires the implementation of:
	//   - T vertices(int level), where level 0 specifies the finest level
	//   - size_t vertexCount(int level) const
	//   - int levels() const;
	static const bool AllowAccessToAllLevels = false;
};