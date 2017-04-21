#pragma once

#include "ExtractedMesh.h"
#include "common.h"
#include "GLBuffer.h"
#include "GLVertexArray.h"

#include "Selection.h"

//Adds rendering functionality to the extracted mesh
class ExtractedMeshGL : public ExtractedMesh
{
public:
	ExtractedMeshGL(const MeshSettings& meshSettings);
	~ExtractedMeshGL();	
	
	void draw(const Eigen::Matrix4f& modelView, const Eigen::Matrix4f& projection);

	bool drawAdjacency;
	bool drawCollapsed;
	bool drawExtracted;
	bool drawModified;
	bool wireframe;
	bool coarseWireframe;
	bool highlightBoundary;

	void reset();

protected:
	void post_extract();
	void adjacencyData(std::vector<Vector3f>&& adj, std::vector<Vector3f>&& color);
	void collapsedGraphData(std::vector<Vector3f>&& adj, std::vector<Vector3f>&& color);
	void modifiedData(std::vector<Vector3f>& points);
private:	
	GLBuffer vertexBuffer, edgeBuffer, triBuffer, quadBuffer, colorBuffer;
	GLVertexArray mesh;
	
	GLBuffer edgesWireframeBuffer;
	GLVertexArray edgesWireframe;
	int edgesWireframeSize;

	GLVertexArray adjacency;

	std::vector<Vector3f> adjacencyVis;
	std::vector<Vector3f> adjacencyVisColor;
	GLBuffer adjColorBuffer, adjPositionBuffer;
	bool adjDirty, modifiedDirty;

	GLVertexArray collapsed;
	std::vector<Vector3f> collapsedVis;
	std::vector<Vector3f> collapsedVisColor;
	GLBuffer collapsedColorBuffer, collapsedPositionBuffer;
	bool collapsedDirty;

	Selection modifiedSet;
};