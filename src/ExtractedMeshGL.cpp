#include "ExtractedMeshGL.h"

#include "ShaderPool.h"

using namespace ExtractionHelper;

struct VertexData
{
	Vector4f position;
	Vector4f normal;
	Vector4f colorDisplacement;
};

struct EdgeData
{
	uint32_t v1;
	uint32_t v2;
	uint32_t cPtr;
	uint32_t isBoundary;
};

struct TriData
{
	int32_t e[3];
	uint32_t cPtr;
};

struct QuadData
{
	int32_t e[4];
	uint32_t cPtr;
	uint32_t pad[3];
};

ExtractedMeshGL::ExtractedMeshGL(const MeshSettings& meshSettings)
	: ExtractedMesh(meshSettings), adjDirty(true), drawAdjacency(false), drawCollapsed(false), drawExtracted(true), drawModified(false), wireframe(false), coarseWireframe(false), highlightBoundary(true),
	vertexBuffer(ShaderStorageBuffer), edgeBuffer(ShaderStorageBuffer), triBuffer(ShaderStorageBuffer), quadBuffer(ShaderStorageBuffer), colorBuffer(ShaderStorageBuffer), adjPositionBuffer(VertexBuffer), adjColorBuffer(VertexBuffer),
	collapsedPositionBuffer(VertexBuffer), collapsedColorBuffer(VertexBuffer),
	edgesWireframeBuffer(VertexBuffer)
{	
}

ExtractedMeshGL::~ExtractedMeshGL()
{
}

void ExtractedMeshGL::reset()
{
	ExtractedMesh::reset();

	adjacencyVis.clear();
	adjacencyVisColor.clear();

	edgesWireframeSize = 0;

	collapsedVis.clear();
	collapsedVisColor.clear();
}

void ExtractedMeshGL::draw(const Eigen::Matrix4f & mv, const Eigen::Matrix4f & proj)
{
	if (triangles.sizeNotDeleted() + quads.sizeNotDeleted() == 0 && adjacencyVis.size() == 0)
		return;

	Eigen::Matrix4f mvp = proj * mv;

	if (adjDirty && drawAdjacency)
	{
		ShaderPool::Instance()->AdjacencyShader.bind();
		adjacency.generate();
		adjacency.bind();
		adjPositionBuffer.uploadData(adjacencyVis.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(adjacencyVis.data()));
		adjPositionBuffer.bindToAttribute("position");
		adjColorBuffer.uploadData(adjacencyVisColor.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(adjacencyVisColor.data()));
		adjColorBuffer.bindToAttribute("color");
		adjDirty = false;		
	}

	if (modifiedDirty && drawModified)
	{
		modifiedSet.uploadData();
		modifiedDirty = false;
	}

	if (collapsedDirty && drawCollapsed)
	{
		ShaderPool::Instance()->AdjacencyShader.bind();
		collapsed.generate();
		collapsed.bind();
		collapsedPositionBuffer.uploadData(collapsedVis.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(collapsedVis.data()));
		collapsedPositionBuffer.bindToAttribute("position");
		collapsedColorBuffer.uploadData(collapsedVisColor.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(collapsedVisColor.data()));
		collapsedColorBuffer.bindToAttribute("color");
		collapsedDirty = false;
	}

	if (drawExtracted)
	{
		//Draw mesh
		GLint oldPolygonMode;
		glGetIntegerv(GL_POLYGON_MODE, &oldPolygonMode);
		if (wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		unsigned short lightness = 49151 + 10000 * sin(ms.count() / 200.0);
		unsigned short a = 53247;
		unsigned short b = 45000;
		Vector3f rgb = LabToRGB(Vector3us(lightness, a, b)).cast<float>() / 65535.0f;

		auto& shader = ShaderPool::Instance()->MeshColorsTriShader;
		shader.bind();
		shader.setUniform("mv", mv);
		shader.setUniform("mvp", mvp);
		shader.setUniform("R", R);
		shader.setUniform("boundaryColor", rgb);
		shader.setUniform("highlightBoundary", highlightBoundary ? 1 : 0);
		mesh.bind();
		vertexBuffer.bindBufferBase(0);
		edgeBuffer.bindBufferBase(1);
		triBuffer.bindBufferBase(2);
		colorBuffer.bindBufferBase(3);
		glPatchParameteri(GL_PATCH_VERTICES, 3);
		glDrawArrays(GL_PATCHES, 0, 3 * triangles.sizeNotDeleted());

		auto& shader2 = ShaderPool::Instance()->MeshColorsQuadShader;
		shader2.bind();
		shader2.setUniform("mv", mv);
		shader2.setUniform("mvp", mvp);
		shader2.setUniform("R", R);
		
		shader2.setUniform("boundaryColor", rgb);
		shader2.setUniform("highlightBoundary", highlightBoundary ? 1 : 0);
		quadBuffer.bindBufferBase(2);
		glPatchParameteri(GL_PATCH_VERTICES, 4);
		glDrawArrays(GL_PATCHES, 0, 4 * quads.sizeNotDeleted());
		mesh.unbind();

		glPolygonMode(GL_FRONT_AND_BACK, oldPolygonMode);
	}

	if(coarseWireframe)
	{
		glDepthRange(0.0, 0.99999f);

		auto& shader = ShaderPool::Instance()->TessellatedEdgesShader;
		shader.bind();
		shader.setUniform("mvp", mvp);
		shader.setUniform("R", R);
		edgesWireframe.bind();
		vertexBuffer.bindBufferBase(0);
		edgeBuffer.bindBufferBase(1);
		colorBuffer.bindBufferBase(3);				
		glPatchParameteri(GL_PATCH_VERTICES, 1);
		glDrawArrays(GL_PATCHES, 0, edgesWireframeSize);
		edgesWireframe.unbind();

		glDepthRange(0.0, 1.0f);
	}

	if (drawAdjacency && adjacencyVis.size() > 0)
	{
		auto& shader = ShaderPool::Instance()->AdjacencyShader;
		shader.bind();
		shader.setUniform("mvp", mvp);
		adjacency.bind(); 
		glDrawArrays(GL_LINES, 0, adjacencyVis.size());
		adjacency.unbind();
	}

	if (drawModified)
	{
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_EQUAL, 0, 0xFF);
		glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
		modifiedSet.draw(mv, proj, 0.6f, 0, -1, Vector3f(0.7f, 0.7f, 0.2f), false, false);
		glDisable(GL_STENCIL_TEST);
	}

	if (drawCollapsed && collapsedVis.size() > 0)
	{
		auto& shader = ShaderPool::Instance()->AdjacencyShader;
		shader.bind();
		shader.setUniform("mvp", mvp);
		collapsed.bind();
		glDrawArrays(GL_LINES, 0, collapsedVis.size());
		collapsed.unbind();
	}
}

Vector4f convertToRGB(const Vector4f& colorDisplacement)
{
	Eigen::Matrix<unsigned short, 3, 1> Lab;
	for (int i = 0; i < 3; ++i)
		Lab(i) = (unsigned short)std::min(65535.0f, std::max(0.0f, colorDisplacement(i)));
	auto rgb = LabToRGB(Lab).cast<float>() / 65535.0f;
	return Vector4f(rgb.x(), rgb.y(), rgb.z(), colorDisplacement.w());
}

void ExtractedMeshGL::post_extract()
{
	//upload to GPU for rendering

	TimedBlock b("Uploading to GPU ..");

	mesh.generate();
	mesh.bind();

	std::vector<Eigen::Vector4f> colors;
	
	std::vector<VertexData> vData(vertices.sizeWithGaps());
	for (int i = 0; i < vertices.sizeWithGaps(); ++i)
	{
		Vertex& v = vertices[i];
		vData[i].position = Eigen::Vector4f(v.position.x(), v.position.y(), v.position.z(), 1);
		vData[i].normal = Eigen::Vector4f(v.normal.x(), v.normal.y(), v.normal.z(), 0);
		vData[i].colorDisplacement = convertToRGB(v.colorDisplacement);
	}	
	vertexBuffer.uploadData(sizeof(VertexData) * vData.size(), vData.data());

	std::vector<EdgeData> eData(edges.sizeWithGaps());
	for (int i = 0; i < edges.sizeWithGaps(); ++i)
	{
		eData[i].v1 = edges[i].v.at(0);
		eData[i].v2 = edges[i].v.at(1);
		eData[i].cPtr = colors.size();
		eData[i].isBoundary = (edges[i].incidentQuads.size() + edges[i].incidentTriangles.size() == 1 ? 1 : 0);

		colors.resize(colors.size() + edges[i].colorDisplacement.size());
#pragma omp parallel for
		for (int j = 0; j < edges[i].colorDisplacement.size(); ++j)
			colors[eData[i].cPtr + j] = convertToRGB(edges[i].colorDisplacement[j]);
	}
	edgeBuffer.uploadData(sizeof(EdgeData) * eData.size(), eData.data());

	std::vector<TriData> triData(triangles.sizeNotDeleted());
	Eigen::Vector3f p;
	Eigen::Vector3f vx[4];
	size_t nextTri = 0;
	for (auto& tri : triangles)
	{
		for (int j = 0; j < 3; ++j)
		{
			triData[nextTri].e[j] = tri.edges[j];
			vx[j] = vertices[startVertex(tri.edges[j])].position;
		}
		triData[nextTri].cPtr = colors.size();
		colors.resize(colors.size() + tri.colorDisplacement.size());
#pragma omp parallel for
		for (int j = 0; j < tri.colorDisplacement.size(); ++j)
			colors[triData[nextTri].cPtr + j] = convertToRGB(tri.colorDisplacement[j]);
		++nextTri;
	}
	triBuffer.uploadData(triData.size() * sizeof(TriData), triData.data());

	std::vector<QuadData> quadData(quads.sizeNotDeleted());
	size_t nextQuad = 0;
	for (auto& quad : quads)
	{
		for (int j = 0; j < 4; ++j)
		{
			quadData[nextQuad].e[j] = quad.edges[j];
			vx[j] = vertices[startVertex(quad.edges[j])].position;
		}
		quadData[nextQuad].cPtr = colors.size();
		colors.resize(colors.size() + quad.colorDisplacement.size());
#pragma omp parallel for
		for (int j = 0; j < quad.colorDisplacement.size(); ++j)
			colors[quadData[nextQuad].cPtr + j] = convertToRGB(quad.colorDisplacement[j]);
		++nextQuad;
	}
	quadBuffer.uploadData(quadData.size() * sizeof(QuadData), quadData.data());

	colorBuffer.uploadData(colors.size() * sizeof(Vector4f), colors.data());

	mesh.unbind();

	edgesWireframe.generate();
	edgesWireframe.bind();

	Eigen::Matrix < int32_t, 1, -1> edgeIds(1, edges.sizeNotDeleted());
	int i = 0;
	for (auto it = edges.begin(); it != edges.end(); ++it)
		edgeIds(0, i++) = it.index();
	ShaderPool::Instance()->TessellatedEdgesShader.bind();
	edgesWireframeBuffer.uploadData(edgeIds).bindToAttribute("in_edgeID");
	edgesWireframeSize = edgeIds.cols();

	edgesWireframe.unbind();
}

void ExtractedMeshGL::adjacencyData(std::vector<Vector3f>&& adj, std::vector<Vector3f>&& color)
{
	adjacencyVis = adj;
	adjacencyVisColor = color;	

	adjDirty = true;
}

void ExtractedMeshGL::collapsedGraphData(std::vector<Vector3f>&& adj, std::vector<Vector3f>&& color)
{
	collapsedVis = adj;
	collapsedVisColor = color;
	collapsedDirty = true;
}

void ExtractedMeshGL::modifiedData(std::vector<Vector3f>& points)
{
	modifiedSet.init();
	modifiedSet.resize(points.size());
#pragma omp parallel for
	for (int i = 0; i < points.size(); ++i)
		modifiedSet.positions.col(i) = Vector4f(points[i].x(), points[i].y(), points[i].z(), meshSettings.scale() / 5);

	modifiedDirty = true;
}