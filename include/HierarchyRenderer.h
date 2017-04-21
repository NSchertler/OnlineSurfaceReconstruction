#pragma once

#include "HierarchyDef.h"

#include "GLBuffer.h"
#include "GLVertexArray.h"

#include "HierarchyCapabilities.h"

struct GLBufferState
{
	GLBufferState(GLBufferType type)
		: buffer(type), dirty(true)
	{}

	GLBuffer buffer;
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
	GLVertexArray inputData, orientationField, positionField, adjacency;	

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
