/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/gui/HierarchyRenderer.h"
#include "osr/gui/ShaderPool.h"
#include "osr/Attributes.h"
#include "osr/Colors.h"

using namespace osr;
using namespace osr::gui;

template <Attribute A, typename Hierarchy, bool HierarchySupportsLevelAccess = HierarchyCapabilities<Hierarchy>::AllowAccessToAllLevels>
struct BufferUploader
{
	static void uploadData(GLBufferState & buffer, Hierarchy& hierarchy, int level) { }
};

template <Attribute A, typename Hierarchy>
struct BufferUploader<A, Hierarchy, true>
{
	static void uploadData(GLBufferState & buffer, Hierarchy& hierarchy, int level)
	{
		typename EigenAttributeTraits<A>::MatrixType data((size_t)EigenAttributeTraits<A>::MatrixType::RowsAtCompileTime, hierarchy.vertexCount(level));
		int i = 0;
		for (auto v : hierarchy.vertices(level))
			data.col(i++) = hierarchy.template attribute<A>(v).template cast<typename EigenAttributeTraits<A>::MatrixType::Scalar>();
		buffer.buffer.uploadData(data);
		std::cout << "Uploading " << data.cols() << " samples of " << AttributeTraits<A>::name() << "." << std::endl;
		buffer.dirty = false;
	}
};

template <typename Hierarchy>
struct BufferUploader<Color, Hierarchy, true>
{
	static void uploadData(GLBufferState & buffer, Hierarchy& hierarchy, int level)
	{
		EigenAttributeTraits<Color>::MatrixType data((size_t)EigenAttributeTraits<Color>::MatrixType::RowsAtCompileTime, hierarchy.vertexCount(level));
		int i = 0;
		for (auto v : hierarchy.vertices(level))
			data.col(i++) = LabToRGB(hierarchy.template attribute<Color>(v)).template cast<float>() / 65535.0f;
		buffer.buffer.uploadData(data);
		std::cout << "Uploading " << data.cols() << " samples of " << AttributeTraits<Color>::name() << "." << std::endl;
		buffer.dirty = false;
	}
};


template <Attribute A, typename Hierarchy>
struct BufferUploader<A, Hierarchy, false>
{
	static void uploadData(GLBufferState & buffer, Hierarchy& hierarchy, int)
	{
		typename EigenAttributeTraits<A>::MatrixType data((size_t)EigenAttributeTraits<A>::MatrixType::RowsAtCompileTime, hierarchy.vertexCount());
		int i = 0;
		for (auto v : hierarchy.vertices())
			data.col(i++) = hierarchy.template attribute<A>(v).template cast<typename EigenAttributeTraits<A>::MatrixType::Scalar>();
		buffer.buffer.uploadData(data);
		std::cout << "Uploading " << data.cols() << " samples of " << AttributeTraits<A>::name() << "." << std::endl;
		buffer.dirty = false;
	}
};

template <typename Hierarchy>
struct BufferUploader<Color, Hierarchy, false>
{
	static void uploadData(GLBufferState & buffer, Hierarchy& hierarchy, int)
	{
		EigenAttributeTraits<Color>::MatrixType data((size_t)EigenAttributeTraits<Color>::MatrixType::RowsAtCompileTime, hierarchy.vertexCount());
		int i = 0;
		for (auto v : hierarchy.vertices())
			data.col(i++) = LabToRGB(hierarchy.template attribute<Color>(v)).template cast<float>() / 65535.0f;
		buffer.buffer.uploadData(data);
		std::cout << "Uploading " << data.cols() << " samples of " << AttributeTraits<Color>::name() << "." << std::endl;
		buffer.dirty = false;
	}
};

template <typename Hierarchy>
struct HierarchyRenderer::HierarchySpecific<Hierarchy, true>
{
	void updateVertexCount()
	{
		if (renderer.hierarchy.levels() == 0)
			renderer.vertexCount = 0;
		else
		{
			if (renderer.level >= renderer.hierarchy.levels())
				renderer.level = renderer.hierarchy.levels() - 1;
			renderer.vertexCount = renderer.hierarchy.vertexCount(renderer.level);
		}
	}

	template<Attribute A>
	void uploadDataToBuffer(GLBufferState & buffer)
	{
		BufferUploader<A, Hierarchy, true>::uploadData(buffer, renderer.hierarchy, renderer.level);
	}

	void uploadAdjacency()
	{
		//Generate vertex buffer for adjacency
		std::vector<Vector3f> adjData; adjData.reserve(renderer.hierarchy.vertexCount() * 2 * 6);
		std::vector<Vector3f> adjColor; adjColor.reserve(renderer.hierarchy.vertexCount() * 2 * 6);
		for (auto v : renderer.hierarchy.vertices(renderer.level))
		{
			renderer.hierarchy.forEachNeighbor(v, [&](const auto& n)
			{
				adjData.push_back(renderer.hierarchy.attribute<Position>(v));
				adjData.push_back(renderer.hierarchy.attribute<Position>(n));

				adjColor.push_back(Vector3f(0.5f, 0.25f, 0.0f));
				adjColor.push_back(Vector3f(0.0f, 0.25f, 0.5f));
			});
		}
		renderer.adjacencyEdges = adjData.size() / 2;
		renderer.adjBuffer.buffer.uploadData(adjData.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(adjData.data()));
		renderer.adjColorBuffer.buffer.uploadData(adjColor.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(adjColor.data()));
		renderer.adjBuffer.dirty = false;
	}


	HierarchySpecific(HierarchyRenderer& renderer)
		:renderer(renderer)
	{ }

	HierarchyRenderer& renderer;
};

template <typename Hierarchy>
struct HierarchyRenderer::HierarchySpecific<Hierarchy, false>
{
	void updateVertexCount()
	{
		renderer.vertexCount = renderer.hierarchy.vertexCount();
	}

	template<Attribute A>
	void uploadDataToBuffer(GLBufferState & buffer)
	{
		BufferUploader<A, Hierarchy, true>::uploadData(buffer, renderer.hierarchy, 0);
	}

	void uploadAdjacency()
	{
		//Generate vertex buffer for adjacency
		std::vector<Vector3f> adjData; adjData.reserve(renderer.hierarchy.vertexCount() * 2 * 6);
		std::vector<Vector3f> adjColor; adjColor.reserve(renderer.hierarchy.vertexCount() * 2 * 6);
		for (auto v : renderer.hierarchy.vertices())
		{
			renderer.hierarchy.forEachNeighbor(v, [&](const auto& n)
			{
				adjData.push_back(renderer.hierarchy.attribute<Position>(v));
				adjData.push_back(renderer.hierarchy.attribute<Position>(n));

				adjColor.push_back(Vector3f(0.5f, 0.25f, 0.0f));
				adjColor.push_back(Vector3f(0.0f, 0.25f, 0.5f));
			});
		}
		renderer.adjacencyEdges = adjData.size() / 2;
		renderer.adjBuffer.buffer.uploadData(adjData.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(adjData.data()));
		renderer.adjColorBuffer.buffer.uploadData(adjColor.size() * 3, 3, sizeof(float), GL_FLOAT, false, reinterpret_cast<uint8_t*>(adjColor.data()));
		renderer.adjBuffer.dirty = false;
	}


	HierarchySpecific(HierarchyRenderer& renderer)
		:renderer(renderer)
	{ }

	HierarchyRenderer& renderer;
};

HierarchyRenderer::HierarchyRenderer(THierarchy & hierarchy)
	: hierarchy(hierarchy), positionBuffer(VertexBuffer), normalBuffer(VertexBuffer), indexBuffer(IndexBuffer),
	orientationFieldBuffer(VertexBuffer), positionFieldBuffer(VertexBuffer), adjBuffer(VertexBuffer), adjColorBuffer(VertexBuffer), colorBuffer(VertexBuffer),
	showInput(false), showNormals(false), showOrientationField(false), showPositionField(false), showAdjacency(false), adjacencyEdges(0), level(0)
{
	hierarchy.PositionsChanged.connect([this]() -> void { positionBuffer.dirty = true; });
	hierarchy.NormalsChanged.connect([this]() -> void { normalBuffer.dirty = true; });
	hierarchy.AdjacencyChanged.connect([this]() -> void { adjBuffer.dirty = true; });
	hierarchy.DirFieldChanged.connect([this]() -> void { orientationFieldBuffer.dirty = true; });
	hierarchy.PosFieldChanged.connect([this]() -> void { positionFieldBuffer.dirty = true; });
}

void HierarchyRenderer::initialize()
{
	Matrix3Xf dummy(3, 1);

	ShaderPool::Instance()->ObjectShader.bind();
	inputData.generate();	
	inputData.bind();	
	positionBuffer.buffer.uploadData(dummy).bindToAttribute("position");
	normalBuffer.buffer.uploadData(dummy).bindToAttribute("normal");
	colorBuffer.buffer.uploadData(dummy).bindToAttribute("color");
	//indexBuffer.buffer.bind();

	ShaderPool::Instance()->NormalShader.bind();
	positionBuffer.buffer.bindToAttribute("position");
	normalBuffer.buffer.bindToAttribute("normal");

	ShaderPool::Instance()->OrientationFieldShader.bind();
	orientationField.generate();	
	orientationField.bind();
	positionBuffer.buffer.bindToAttribute("position");
	normalBuffer.buffer.bindToAttribute("normal");
	orientationFieldBuffer.buffer.uploadData(dummy).bindToAttribute("tangent");

	ShaderPool::Instance()->PositionFieldShader.bind();	
	positionField.generate();
	positionField.bind();
	normalBuffer.buffer.bindToAttribute("normal");
	positionFieldBuffer.buffer.uploadData(dummy).bindToAttribute("position");
	positionField.unbind();	

	ShaderPool::Instance()->AdjacencyShader.bind();
	adjacency.generate();
	adjacency.bind();
	adjBuffer.buffer.uploadData(dummy).bindToAttribute("position");
	adjColorBuffer.buffer.uploadData(dummy).bindToAttribute("color");
	adjacency.unbind();
}

void HierarchyRenderer::assertBuffersUpdated()
{	
	HierarchySpecific<THierarchy> hierarchySpecific(*this);
	hierarchySpecific.updateVertexCount();
	if (vertexCount == 0)
		return;

	bool needPosition = showInput || showNormals || showOrientationField;
	bool needNormals = showInput || showNormals || showOrientationField || showPositionField;
	bool needOrientation = showOrientationField;
	bool needPosField = showPositionField;
	bool needAdjacency = showAdjacency;

	if (needPosition && positionBuffer.dirty)
	{
		hierarchySpecific.uploadDataToBuffer<Position>(positionBuffer);
		hierarchySpecific.uploadDataToBuffer<Color>(colorBuffer);
	}

	if (needNormals && normalBuffer.dirty)
		hierarchySpecific.uploadDataToBuffer<Normal>(normalBuffer);

	if (needOrientation && orientationFieldBuffer.dirty)
		hierarchySpecific.uploadDataToBuffer<DirField>(orientationFieldBuffer);

	if (needPosField && positionFieldBuffer.dirty)
		hierarchySpecific.uploadDataToBuffer<PosField>(positionFieldBuffer);

	if (needAdjacency && adjBuffer.dirty)
		hierarchySpecific.uploadAdjacency();
}

void HierarchyRenderer::draw(const Eigen::Matrix4f & mv, const Eigen::Matrix4f & proj, int rosy)
{
	if (!showInput && !showNormals && !showOrientationField && !showPositionField && !showAdjacency)
		return;	

	Eigen::Matrix4f mvp = proj * mv;

	assertBuffersUpdated();

	if (vertexCount == 0)
		return;

	if (showInput)
	{
		glPointSize(5);
		//Draw input Mesh
		auto& shader = ShaderPool::Instance()->ObjectShader;
		shader.bind();
		shader.setUniform("mv", mv);
		shader.setUniform("mvp", mvp);
		inputData.bind();

		glDrawArrays(GL_POINTS, 0, vertexCount);
		inputData.unbind();
	}

	if (showNormals)
	{
		auto& shader = ShaderPool::Instance()->NormalShader;
		inputData.bind();
		shader.bind();
		shader.setUniform("mvp", mvp);
		shader.setUniform("scale", 0.01f * hierarchy.boundingBox().diagonal().minCoeff());
		glDrawArrays(GL_POINTS, 0, vertexCount);
		inputData.unbind();
	}

	//Draw orientation field
	if (showOrientationField)
	{
		auto& shader = ShaderPool::Instance()->OrientationFieldShader;
		shader.bind();
		shader.setUniform("mvp", mvp);
		shader.setUniform("offset", 0.001f * hierarchy.boundingBox().diagonal().norm());
		shader.setUniform("scale", 0.004f * hierarchy.boundingBox().diagonal().norm());
		shader.setUniform("rosy", rosy);
		orientationField.bind();
		glDrawArrays(GL_POINTS, 0, vertexCount);
		orientationField.unbind();
	}

	//Draw position field
	if (showPositionField)
	{
		glPointSize(5);
		auto& shader = ShaderPool::Instance()->PositionFieldShader;
		shader.bind();
		shader.setUniform("mvp", mvp);
		shader.setUniform("offset", 0.00f);
		positionField.bind();
		glDrawArrays(GL_POINTS, 0, vertexCount);
		positionField.unbind();
	}
	
	if (showAdjacency)
	{
		GLboolean saveDepthWriteMask;
		GLint saveBlendSrc, saveBlendDst;
		glGetIntegerv(GL_BLEND_SRC_RGB, &saveBlendSrc);
		glGetIntegerv(GL_BLEND_DST_RGB, &saveBlendDst);
		glGetBooleanv(GL_DEPTH_WRITEMASK, &saveDepthWriteMask);

		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
		glDepthMask(false);

		auto& shader = ShaderPool::Instance()->AdjacencyShader;
		shader.bind();
		shader.setUniform("mvp", mvp);
		adjacency.bind();
		glDrawArrays(GL_LINES, 0, 2 * adjacencyEdges);
		adjacency.unbind();

		glDepthMask(saveDepthWriteMask);
		glBlendFunc(saveBlendSrc, saveBlendDst);
	}
}

void HierarchyRenderer::setLevel(int l)
{
	if (l != level)
	{
		level = l;
		if (HierarchyCapabilities<THierarchy>::AllowAccessToAllLevels)
		{
			positionBuffer.dirty = true;
			normalBuffer.dirty = true;
			adjBuffer.dirty = true;
			orientationFieldBuffer.dirty = true;
			positionFieldBuffer.dirty = true;
		}
	}
}
