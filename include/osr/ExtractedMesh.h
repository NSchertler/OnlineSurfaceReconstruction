/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#pragma once

#include "dset.h"

#include "osr/MeshSettings.h"
#include "osr/PreparedVertexSet.h"
#include "osr/HierarchyDef.h"
#include <nsessentials/data/PersistentIndexContainer.h>
#include <nsessentials/data/Parallelization.h>
#include <nsessentials/data/Serialization.h>
#include "osr/Colors.h"
#include "osr/ExtractionUtils.h"
#include "osr/ExtractionAttributeMapping.h"
#include "osr/MeshVisitor.h"

#include <set>
#include <array>
#include <map>
#include <unordered_map>

namespace nse
{
	namespace data
	{
		//Helper methods for project serialization
		template<> void saveToFile(const osr::ExtractionHelper::Triangle& object, FILE* f);
		template<> void saveToFile(const osr::ExtractionHelper::Quad& object, FILE* f);
		template<> void saveToFile(const osr::ExtractionHelper::Edge& object, FILE* f);
		template<> void saveToFile(const osr::ExtractionHelper::Vertex& object, FILE* f);
		template<> void loadFromFile(osr::ExtractionHelper::Triangle& object, FILE* f);
		template<> void loadFromFile(osr::ExtractionHelper::Quad& object, FILE* f);
		template<> void loadFromFile(osr::ExtractionHelper::Edge& object, FILE* f);
		template<> void loadFromFile(osr::ExtractionHelper::Vertex& object, FILE* f);
	}
}

namespace osr
{
	struct TaggedLink
	{
		uint32_t id;
		uint8_t flag;
		TaggedLink(uint32_t id) : id(id), flag(0) { }
		bool used() const { return flag & 1; }
		void markUsed() { flag |= 1; }
	};

	//Represents the final output of the extraction process. This class is also responsible
	//for performing the actual extraction.
	class ExtractedMesh : public ExtractionHelper::IEntityContainer
	{
	public:	
		ExtractedMesh(const MeshSettings& meshSettings);

		//Performs the extraction of a part of a point cloud.
		//  modifiedVertices - the set of modified points from the hierarchy.
		//  deleteAndExpand  - specifies if the current extraction result should be cleaned in the surrounding of
		//                     the modified points before calculating the extraction.
		//  removeVertices   - list of old mesh vertices that need to be removed.
		void extract(PreparedVertexSet<THierarchy, THierarchy::VertexIndex, true, false>& modifiedVertices, bool deleteAndExpand = false, const std::vector<MeshVertexType>& removeVertices = std::vector<MeshVertexType>());

		//Resets the extraction result and produces a clean and empty mesh.
		virtual void reset();

		//Saves the coarse mesh to a PLY file.
		void saveCoarseToPLY(const std::string& path);

		//Saves the wireframe of the coarse mesh in MeshLab format. The edges are tessellated
		//according to the face resolution.
		void saveWireframeToPLY(const std::string& path);

		//Saves the tessellated mesh to a PLY file. If triangulate is set to true,
		//all quads are triangulated in the output.
		void saveFineToPLY(const std::string& path, bool triangulate = false);

		//Visits the fine mesh with the given visitor.
		void extractFineMesh(osr::MeshVisitor& visitor, bool triangulate);

		//Saves the state of the mesh to file.
		void saveToFile(FILE* f) const;
		//Restores the state of the mesh from file.
		void loadFromFile(FILE* f);

		void getInterpolatedPositionNormal(const ExtractionHelper::Vertex& v, int localIndex, Vector3f& out_position, Vector3f& out_normal) const;
		void getInterpolatedPositionNormal(const ExtractionHelper::Edge& e, int localIndex, Vector3f& out_position, Vector3f& out_normal) const;
		void getInterpolatedPositionNormal(const ExtractionHelper::Triangle& t, int localIndex, Vector3f& out_position, Vector3f& out_normal) const;
		void getInterpolatedPositionNormal(const ExtractionHelper::Quad& q, int localIndex, Vector3f& out_position, Vector3f& out_normal) const;

	protected:

		//resolution for face attributes
		const int R = 10;
		const int texelsPerEdge = R - 1;
		const int triInnerRings = (R % 2 == 0 ? (R - 2) / 2 : (R - 1) / 2);
		const int texelsPerTri = (R % 2 == 0 ? (3 * R - 6) * R / 4 + 1 : 3 * (R - 1) * (R - 1) / 4 );
		const int texelsPerQuad = (R - 1) * (R - 1);	

		nse::data::PersistentIndexContainer<ExtractionHelper::Triangle> triangles;
		nse::data::PersistentIndexContainer<ExtractionHelper::Quad> quads;
		nse::data::PersistentIndexContainer<ExtractionHelper::Edge> edges;
		nse::data::PersistentIndexContainer<ExtractionHelper::Vertex> vertices;

		std::map<std::pair<uint32_t, uint32_t>, int32_t> vertexPairToEdge;
		std::map<uint32_t, std::vector<uint32_t>> vertexToDependentEdges; //non-incident edges of higher-order polygons

		const MeshSettings& meshSettings;

		//For a given edge ID (might be negative), calculates the index of the according start vertex.
		uint32_t startVertex(int32_t edgeId) const;

		//For a given edge ID (might be negative), calculates the index of the according edge.
		uint32_t edgeIndex(int32_t edgeId) const;

		//Extension point that gets called after extraction is done.
		virtual void post_extract() { }	

		//Extension point that gets called with the list of modified points
		virtual void modifiedData(std::vector<Vector3f>& points) { }

		//Extension point that gets called with adjacency data of the modified points.
		virtual void adjacencyData(std::vector<Vector3f>&& adj, std::vector<Vector3f>&& color) { }

		//Extension point that gets called with the collapsed adjacency graph.
		virtual void collapsedGraphData(std::vector<Vector3f>&& adj, std::vector<Vector3f>&& color) { }

		//The current generation of the mesh.
		uint8_t currentGeneration = -1;

	private:
		//Removes old geometry whose source points have changed or whose source points have been removed completely.
		void deleteModifiedGeometry(PreparedVertexSet<THierarchy, THierarchy::VertexIndex, true, false>& modifiedVertices, const std::vector<MeshVertexType>& removeVertices);

		//Extracts the collapsed adjacency graph for a given set of modified points.
		//  newVertices - output list of newly created mesh vertices
		//  checkForDuplicatedAboveIndex - points with indices above this value are checked for being duplicates.
		void extract_graph(PreparedVertexSet<THierarchy, THierarchy::VertexIndex, true, false>& modifiedVertices,
			std::vector<std::vector<TaggedLink>> &adj_new,
			std::vector<size_t>& newVertices, size_t checkForDuplicatesAboveIndex);

		//Extracts faces from the collapsed adjacency graph by greedy search.
		void extract_faces(std::vector<std::vector<TaggedLink> > &adj, std::vector<size_t>& newVertices, std::vector<size_t>& newEdges, std::vector<size_t>& newTris, std::vector<size_t>& newQuads);

		//Tries to find a face with the specified degree starting at the edge defined by cur/curIdx
		bool extract_face(uint32_t cur, uint32_t curIdx, size_t targetSize,
			std::vector<std::vector<TaggedLink>>& adj, std::vector<std::pair<uint32_t, uint32_t>> &result);
	
		//Fills the polygon defined by the given corner vertices with primitive faces.
		void fill_face(std::vector<std::pair<uint32_t, uint32_t>> &verts, std::vector<size_t>& newVertices, std::vector<size_t>& newEdges, std::vector<size_t>& newTris, std::vector<size_t>& newQuads, std::set<size_t>& nonmanifoldEdges);
	
		//Gets the edge ID between the two given vertices or creates a new one if it does not exist.
		int32_t get_edge(uint32_t v1, uint32_t v2, std::vector<size_t>& newEdges);

		//Adds a triangle or a quad to the mesh.
		void add_primitive_face(const std::vector<uint32_t>& indices, bool checkForDuplicate, std::vector<size_t>& newEdges, std::vector<size_t>& newTris, std::vector<size_t>& newQuads, std::set<size_t>& nonmanifoldEdges);

		//Searches for a duplicate vertex at the provided position. The vertex with id ignore is not considered a duplicate.
		bool findDuplicateVertex(const Vector3f& p, size_t& outVertexIndex, size_t ignore);

		//Searches for a duplicate face.
		template <size_t N, typename DataSource>
		bool findDuplicateFace(const std::array<int32_t, N>& edges, DataSource& data, size_t& out_index);

		//Calculates the detail map for newly created geometry.
		void mapAttributes(PreparedVertexSet<THierarchy, THierarchy::VertexIndex, true, false>& modifiedVertices, const std::vector<size_t>& newVertices, const std::vector<size_t>& newEdges, const std::vector<size_t>& newTris, const std::vector<size_t>& newQuads);

		//Adds the uniform graph Laplacian for newly created geometry to the least squares system.
		template <typename Builder>
		void prepareLaplacian(const std::vector<size_t>& newVertices, const std::vector<size_t>& newEdges, const std::vector<size_t>& newTris, const std::vector<size_t>& newQuads, float weight, Builder& systemBuilder);

		//Projects a point on a quad in the direction of the interpolated normal.
		void projectPointOnQuad(const ExtractionHelper::Quad& quad, const Vector3f& p, Vector2f& uv, Vector3f& interpolP, Vector3f& interpolN);
		//Projects a point on a triangle in the direction of the interpolated normal.
		void projectPointOnTriangle(const ExtractionHelper::Triangle& quad, const Vector3f& p, Vector2f& uv, Vector3f& interpolP, Vector3f& interpolN);

		//Maps a Ring/Edge/Vertex tuple (triangle texel parameterization) to the triangle-local texel index.
		int revToIndex(const Vector3i& rev) const;
		//Maps a triangle-local texel index to its Ring/Edge/Vertex tuple
		Vector3i indexToREV(int index) const;
		//Maps barycentric coordinates of a texel to the according Ring/Edge/Vertex tuple (triangle texel parameterization).
		Eigen::Vector3f barycentricToREV(const Vector2f& barycentric) const;	
		//Maps a Ring/Edge/Vertex tuple to the according barycentric coordinates
		Eigen::Vector2f revToBarycentric(const Vector3i& rev) const;

		void getEntityTexel(const ExtractionHelper::Vertex& v, const ExtractionHelper::Entity*& entity, int& localIndex);
		void getEntityTexel(const ExtractionHelper::Edge& e, int discreteU, const ExtractionHelper::Entity*& entity, int& localIndex);
		void getEntityTexelBarycentric(const ExtractionHelper::Triangle& tri, const Vector2i& uv, const ExtractionHelper::Entity*& entity, int& localIndex);
		void getEntityTexel(const ExtractionHelper::Triangle& tri, const Vector3i& rev, const ExtractionHelper::Entity*& entity, int& localIndex);
		void getEntityTexel(const ExtractionHelper::Quad& quad, const Vector2i& coord, const ExtractionHelper::Entity*& entity, int& localIndex);

		void getInterpolationInfo(const ExtractionHelper::Triangle& tri, const Vector2f& uv, ExtractionHelper::FaceInterpolationInfo* interpolationInfo);
		void getInterpolationInfo(const ExtractionHelper::Quad& quad, const Vector2f& uv, ExtractionHelper::FaceInterpolationInfo* interpolationInfo);

		//Removes faces in order to produce manifoldness.
		template <typename FaceType, typename IncidentFacesCallback>
		void removeIncidentArtefacts(uint32_t eid, nse::data::PersistentIndexContainer<FaceType>& faces, std::set<uint32_t>& removedEdges, std::set<uint32_t>& removedFaces, const IncidentFacesCallback& getIncidentFaces);

		void calculateCollapsedGraphVisualization(const std::vector<size_t>& newVertices, const std::vector<bool>& partOfCore, const std::vector<bool>& vertexReachableFromCore, std::vector<std::vector<TaggedLink>>& adj);

		bool remove_spurious_vertices = true;
		bool remove_unnecessary_edges = true;
		bool remove_unnecessary_triangles = true;
		bool snap_vertices = true;
		bool fill_holes = true;	

		const int maxFaceDegree = 8;

		//Checks if the stored information of the mesh are consistent.
		template <typename FaceType, typename IncidentFacesCallback>
		void checkIncidenceConsistency(nse::data::PersistentIndexContainer<ExtractionHelper::Edge>& edges, nse::data::PersistentIndexContainer<FaceType>& faces, const IncidentFacesCallback& getIncidentFaces);
	};

	//Checks if the graph is symmetric.
	extern void checkSymmetry(std::vector<std::vector<TaggedLink>>& adj);	

	template <typename Builder>
	void ExtractedMesh::prepareLaplacian(const std::vector<size_t>& newVertices, const std::vector<size_t>& newEdges, const std::vector<size_t>& newTris, const std::vector<size_t>& newQuads, float weight, Builder& systemBuilder)
	{
		nse::util::TimedBlock b("Preparing Laplacian ..");

		// The number of rows of the linear system that are cached before adding them to the system.
		// Should be close to a multiple of the number of texels per quad and texels per triangle
		const int prepareRows = 9720;

		std::vector<LaplacianEntry> preparedRows(prepareRows);

		//Add a row for each new vertex
		auto generateRowForVertex = [this](size_t vIdx, LaplacianEntry& row)
		{
			row.reset();
			ExtractionHelper::Vertex& v = vertices[vIdx];

			row.center.entity = &v;
			row.center.localIndex = 0;

			for (auto eid : v.incidentEdges)
			{
				auto& e = edges[eid];
				int localTexelIndex = (vIdx == e.v[0] ? 0 : texelsPerEdge - 1);
				row.addNeighbor(&e, localTexelIndex);
			}
		};

		int processedVertices = 0;
		while (processedVertices < newVertices.size())
		{
			int lower = processedVertices;
			int upperExcl = std::min<int>(newVertices.size(), lower + prepareRows);
	#pragma omp parallel for
			for (int i = lower; i < upperExcl; ++i)
			{
				auto& v = vertices[newVertices[i]];
				if (v.incidentEdges.size() == 0 || v.generation != currentGeneration)
					continue;
				generateRowForVertex(newVertices[i], preparedRows[i - lower]);
			}

			for (int i = lower; i < upperExcl; ++i)
				if (vertices[newVertices[i]].incidentEdges.size() > 0 && vertices[newVertices[i]].generation == currentGeneration)
					systemBuilder.addLaplacianEntry(preparedRows[i - lower], weight, currentGeneration, *this);
			processedVertices = upperExcl;
		}

		//Add rows for each new edge
		auto generateRowsForEdge = [this](size_t eid, LaplacianEntry* rowPtr)
		{
			auto& e = edges[eid];

			struct IterationInfo
			{
				const ExtractionHelper::Entity* entity;
				int offset;
				int stride;
				int modulo;

				IterationInfo() {}
				IterationInfo(ExtractionHelper::Entity* entity, int offset, int stride, int modulo)
					: entity(entity), offset(offset), stride(stride), modulo(modulo)
				{ }
			};

			//this vector specifies which texels of the incident faces are neighbors of the current edge's texels
			std::vector<IterationInfo> incidentFaceNeighborTexelsStartStride(e.incidentQuads.size() + e.incidentTriangles.size());
			for (int i = 0; i < e.incidentQuads.size(); ++i)
			{
				auto& quad = quads[e.incidentQuads[i]];

				if (quad.edges[0] == eid)
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, 0, 1, texelsPerQuad);
				else if (edgeIndex(quad.edges[0]) == eid) //inverse
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, R - 2, -1, texelsPerQuad);
				else if (quad.edges[1] == eid)
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, R - 2, R - 1, texelsPerQuad);
				else if (edgeIndex(quad.edges[1]) == eid) //inverse
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, (R - 1) * (R - 1) - 1, -(R - 1), texelsPerQuad);
				else if (quad.edges[2] == eid)
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, (R - 1) * (R - 1) - 1, -1, texelsPerQuad);
				else if (edgeIndex(quad.edges[2]) == eid) //inverse
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, (R - 1) * (R - 2), 1, texelsPerQuad);
				else if (quad.edges[3] == eid)
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, (R - 1) * (R - 2), -(R - 1), texelsPerQuad);
				else if (edgeIndex(quad.edges[3]) == eid) //inverse
					incidentFaceNeighborTexelsStartStride[i] = IterationInfo(&quad, 0, R - 1, texelsPerQuad);
			}
			int texelsOuterRing = 3 * (R - 2);
			for (int i = 0; i < e.incidentTriangles.size(); ++i)
			{
				auto& tri = triangles[e.incidentTriangles[i]];

				if (tri.edges[0] == eid)
					incidentFaceNeighborTexelsStartStride[i + e.incidentQuads.size()] = IterationInfo(&tri, 0, 1, texelsOuterRing);
				else if (tri.edges[1] == eid)
					incidentFaceNeighborTexelsStartStride[i + e.incidentQuads.size()] = IterationInfo(&tri, R - 2, 1, texelsOuterRing);
				else if (tri.edges[2] == eid)
					incidentFaceNeighborTexelsStartStride[i + e.incidentQuads.size()] = IterationInfo(&tri, 2 * R - 4, 1, texelsOuterRing);
				else if (edgeIndex(tri.edges[0]) == eid) //inverse
					incidentFaceNeighborTexelsStartStride[i + e.incidentQuads.size()] = IterationInfo(&tri, R - 2, -1, texelsOuterRing);
				else if (edgeIndex(tri.edges[1]) == eid) //inverse
					incidentFaceNeighborTexelsStartStride[i + e.incidentQuads.size()] = IterationInfo(&tri, 2 * R - 4, -1, texelsOuterRing);
				else if (edgeIndex(tri.edges[2]) == eid) //inverse
					incidentFaceNeighborTexelsStartStride[i + e.incidentQuads.size()] = IterationInfo(&tri, 3 * R - 6, -1, texelsOuterRing);
			}

			for (int i = 0; i < texelsPerEdge; ++i)
			{
				auto& row = *(rowPtr + i);
				row.reset();
				row.center.entity = &e;
				row.center.localIndex = i;
				if (i == 0)
				{
					auto& v = vertices[e.v[0]];
					row.addNeighbor(&v, 0);
				}
				else
					row.addNeighbor(&e, i - 1);

				if (i == texelsPerEdge - 1)
				{
					auto& v = vertices[e.v[1]];
					row.addNeighbor(&v, 0);
				}
				else
					row.addNeighbor(&e, i + 1);

				for (auto& n : incidentFaceNeighborTexelsStartStride)
					row.addNeighbor(n.entity, (n.offset + (i * n.stride)) % n.modulo);
			}
		};
		int processedEdges = 0;
		while (processedEdges < newEdges.size())
		{
			int lower = processedEdges;
			int upperExcl = std::min<int>(newEdges.size(), lower + prepareRows / texelsPerEdge);
	#pragma omp parallel for
			for (int i = lower; i < upperExcl; ++i)
			{
				generateRowsForEdge(newEdges[i], &preparedRows[(i - lower) * texelsPerEdge]);
			}

			int rows = (upperExcl - lower) * texelsPerEdge;
			for (int i = 0; i < rows; ++i)
				systemBuilder.addLaplacianEntry(preparedRows[i], weight, currentGeneration, *this);
			processedEdges = upperExcl;
		}

		//Add rows for each new triangle
		auto generateRowsForTriangle = [this](size_t tid, LaplacianEntry* rowPtr)
		{
			const ExtractionHelper::Entity* entity;
			int localIndex;

			auto& tri = triangles[tid];
			for (int ring = 1; ring <= triInnerRings; ++ring)
			{
				auto ringBase = 3 * (ring - 1) * (R - ring); //all texels from larger rings;
				int verticesOnRingEdge = R - 2 * ring;
				int verticesOnRing = 3 * verticesOnRingEdge;
				for (int edge = 0; edge < 3; ++edge)
				{
					//Entries for first vertex on edge
					int idx = revToIndex(Vector3i(ring, edge, 0));
					auto ringLocalIdx = idx - ringBase;

					rowPtr->reset();
					rowPtr->center.entity = &tri;
					rowPtr->center.localIndex = idx; //current texel
					rowPtr->addNeighbor(&tri, ringBase + (ringLocalIdx + 1) % verticesOnRing); //next vertex on ring
					rowPtr->addNeighbor(&tri, ringBase + (ringLocalIdx - 1 + verticesOnRing) % verticesOnRing); //previous vertex on ring

					getEntityTexel(tri, Vector3i(ring - 1, edge, 1), entity, localIndex); //outer ring, same edge
					rowPtr->addNeighbor(entity, localIndex);
					getEntityTexel(tri, Vector3i(ring - 1, (edge + 2) % 3, verticesOnRingEdge + 1), entity, localIndex); //outer ring, previous edge
					rowPtr->addNeighbor(entity, localIndex);
					rowPtr++;

					for (int v = 1; v < verticesOnRingEdge; ++v)
					{
						auto& row = *rowPtr++;
						row.reset();

						idx = revToIndex(Vector3i(ring, edge, v));
						ringLocalIdx = idx - ringBase;
						row.center.entity = &tri;
						row.center.localIndex = idx; //current texel
						row.addNeighbor(&tri, ringBase + (ringLocalIdx + 1) % verticesOnRing); //next vertex on ring
						row.addNeighbor(&tri, ringBase + (ringLocalIdx - 1 + verticesOnRing) % verticesOnRing); //previous vertex on ring

																												//find the neighbor on the inner ring
						auto innerNeighborEdge = edge;
						auto innerNeighborV = v - 1;
						if (innerNeighborV >= R - 2 * (ring + 1)) //if we are at the end of an edge
						{
							innerNeighborEdge++; //proceed to the next edge
							if (innerNeighborEdge >= 3)
								innerNeighborEdge = 0;
							innerNeighborV = 0;
						}
						getEntityTexel(tri, Vector3i(ring + 1, innerNeighborEdge, innerNeighborV), entity, localIndex); //vertex on inner ring
						row.addNeighbor(entity, localIndex);
						getEntityTexel(tri, Vector3i(ring - 1, edge, v + 1), entity, localIndex); //vertex on outer ring
						row.addNeighbor(entity, localIndex);
					}
				}
			}
			if (R % 2 == 0)
			{
				//center vertex only exists for even resolution
				rowPtr->reset();

				getEntityTexel(tri, Vector3i(triInnerRings + 1, 0, 0), entity, localIndex);
				rowPtr->center.entity = entity;
				rowPtr->center.localIndex = localIndex;
				getEntityTexel(tri, Vector3i(triInnerRings, 0, 1), entity, localIndex);
				rowPtr->addNeighbor(entity, localIndex);
				getEntityTexel(tri, Vector3i(triInnerRings, 1, 1), entity, localIndex);
				rowPtr->addNeighbor(entity, localIndex);
				getEntityTexel(tri, Vector3i(triInnerRings, 2, 1), entity, localIndex);
				rowPtr->addNeighbor(entity, localIndex);
			}
		};
		int processedTriangles = 0;
		while (processedTriangles < newTris.size())
		{
			int lower = processedTriangles;
			int upperExcl = std::min<int>(newTris.size(), lower + prepareRows / texelsPerTri);
	#pragma omp parallel for
			for (int i = lower; i < upperExcl; ++i)
			{
				generateRowsForTriangle(newTris[i], &preparedRows[(i - lower) * texelsPerTri]);
			}

			int rows = (upperExcl - lower) * texelsPerTri;
			for (int i = 0; i < rows; ++i)
				systemBuilder.addLaplacianEntry(preparedRows[i], weight, currentGeneration, *this);
			processedTriangles = upperExcl;
		}

		//Add rows for each new quad
		auto generateRowsForQuad = [this](size_t qid, LaplacianEntry* rowPtr)
		{
			auto& quad = quads[qid];

			const ExtractionHelper::Entity* entity;
			int localIndex;

			for (int j = 0; j < R - 1; ++j)
				for (int i = 0; i < R - 1; ++i)
				{
					auto& row = *rowPtr++;
					row.reset();
					int localIdx = i + (R - 1) * j;

					getEntityTexel(quad, Vector2i(i + 1, j + 1), entity, localIndex);
					row.center.entity = entity;
					row.center.localIndex = localIndex;

					getEntityTexel(quad, Vector2i(i + 0, j + 1), entity, localIndex);
					row.addNeighbor(entity, localIndex);

					getEntityTexel(quad, Vector2i(i + 2, j + 1), entity, localIndex);
					row.addNeighbor(entity, localIndex);

					getEntityTexel(quad, Vector2i(i + 1, j + 0), entity, localIndex);
					row.addNeighbor(entity, localIndex);

					getEntityTexel(quad, Vector2i(i + 1, j + 2), entity, localIndex);
					row.addNeighbor(entity, localIndex);
				}
		};

		int processedQuads = 0;
		while (processedQuads < newQuads.size())
		{
			int lower = processedQuads;
			int upperExcl = std::min<int>(newQuads.size(), lower + prepareRows / texelsPerQuad);
	#pragma omp parallel for
			for (int i = lower; i < upperExcl; ++i)
			{
				generateRowsForQuad(newQuads[i], &preparedRows[(i - lower) * texelsPerQuad]);
			}

			int rows = (upperExcl - lower) * texelsPerQuad;
			for (int i = 0; i < rows; ++i)
				systemBuilder.addLaplacianEntry(preparedRows[i], weight, currentGeneration, *this);
			processedQuads = upperExcl;
		}
	}

	template <typename FaceType, typename IncidentFacesCallback>
	void ExtractedMesh::checkIncidenceConsistency(nse::data::PersistentIndexContainer<ExtractionHelper::Edge>& edges, nse::data::PersistentIndexContainer<FaceType>& faces, const IncidentFacesCallback& getIncidentFaces)
	{
		for (auto edgeIt = edges.begin(); edgeIt != edges.end(); ++edgeIt)
		{
			auto& edge = *edgeIt;
			for (auto faceIt : getIncidentFaces(edge))
			{
				auto& face = faces[faceIt];
				bool found = false;
				for (int i = 0; i < FaceType::FaceDegree; ++i)
					if (edgeIndex(face.edges[i]) == edgeIt.index())
						found = true;
				if (!found)
					std::cout << "Edge " << edgeIt.index() << " is incident to face " << faceIt << " but not the other way around." << std::endl;
			}
		}
	}
}