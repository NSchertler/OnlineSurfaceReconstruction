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
		template <typename Hierarchy, typename Index>
		void extract(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices, bool deleteAndExpand = false, const std::vector<MeshVertexType>& removeVertices = std::vector<MeshVertexType>());

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

		void prepareUnityMesh(bool triangulate);

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
		template <typename Hierarchy, typename Index>
		void deleteModifiedGeometry(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices, const std::vector<MeshVertexType>& removeVertices);

		//Extracts the collapsed adjacency graph for a given set of modified points.
		//  newVertices - output list of newly created mesh vertices
		//  checkForDuplicatedAboveIndex - points with indices above this value are checked for being duplicates.
		template <typename Hierarchy, typename Index>
		void extract_graph(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices,
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
		template <typename Hierarchy, typename Index>
		void mapAttributes(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices, const std::vector<size_t>& newVertices, const std::vector<size_t>& newEdges, const std::vector<size_t>& newTris, const std::vector<size_t>& newQuads);

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
		void prepareUnityMeshName(const std::string& path);
	};

	//Checks if the graph is symmetric.
	extern void checkSymmetry(std::vector<std::vector<TaggedLink>>& adj);

	template <typename Hierarchy, typename Index>
	void ExtractedMesh::extract(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices, bool deleteAndExpand, const std::vector<MeshVertexType>& removeVertices)
	{
		nse::util::TimedBlock b("Extracting mesh ..", true);

		++currentGeneration;

	#ifdef DEBUG_VISUALIZATION
		saveCoarseToPLY("BeforeDeletion.ply");

		{
			nse::util::TimedBlock b("Generating modified visualization ..");
			std::vector<Vector3f> modifiedPoints(modifiedVertices.includedVertices);
	#pragma omp parallel for
			for (int i = 0; i < modifiedVertices.includedVertices; ++i)
				modifiedPoints[i] = modifiedVertices.template attribute<Position>(VertexSetIndex(i));
			modifiedData(modifiedPoints);
		}
	#endif

		size_t duplicatesAboveIndex = modifiedVertices.includedVertices;
		if (deleteAndExpand)
		{		
	#ifdef DEBUG_VISUALIZATION
			std::vector<Vector3f> adjacencyVis;
			std::vector<Vector3f> adjacencyVisColor;
		
			auto oldVertexCount = modifiedVertices.includedVertices;
			auto oldVertexCountWithNeighbors = modifiedVertices.vertices.size();

			{
				nse::util::TimedBlock b("Preparing adjacency visualization ..");
				for (int i = 0; i < modifiedVertices.includedVertices; ++i)
				{
					VertexSetIndex current(i);
					modifiedVertices.forEachNeighbor(current, [&](const VertexSetIndex& n)
					{
						adjacencyVis.push_back(modifiedVertices.template attribute<Position>(current));
						adjacencyVis.push_back(modifiedVertices.template attribute<Position>(n));
						adjacencyVisColor.push_back(Vector3f::Ones());
						adjacencyVisColor.push_back(n.index < oldVertexCount ? Vector3f(1, 1, 1) : Vector3f(1, 0, 0));
					});
				}
			}
	#endif		
			//Delete geometry that references points in the modified set.
			deleteModifiedGeometry(modifiedVertices, removeVertices);
		
			//Grow the set of modified points
			modifiedVertices.expandBy(3 * modifiedVertices.hierarchy->meshSettings().scale());

	#ifdef DEBUG_VISUALIZATION
			{
				nse::util::TimedBlock b("Calculating adjacency visualization ..");
				for (int i = 0; i < modifiedVertices.includedVertices; ++i)
				{
					VertexSetIndex current(i);

					Vector3f firstColor;
					if (i < oldVertexCount)
						firstColor = Vector3f::Ones();
					else if (i < oldVertexCountWithNeighbors)
						firstColor = Vector3f::UnitX();
					else
						firstColor = Vector3f::UnitY();

					modifiedVertices.forEachNeighbor(current, [&](const VertexSetIndex& n)
					{
						if (i < oldVertexCount && n.index < oldVertexCount)
							return;

						Vector3f secondColor;
						if (n.index < oldVertexCount)
							secondColor = Vector3f::Ones();
						else if (n.index < oldVertexCountWithNeighbors)
							secondColor = Vector3f::UnitX();
						else
							secondColor = Vector3f::UnitY();

						adjacencyVis.push_back(modifiedVertices.template attribute<Position>(current));
						adjacencyVis.push_back(modifiedVertices.template attribute<Position>(n));
						adjacencyVisColor.push_back(firstColor);
						adjacencyVisColor.push_back(secondColor);
					});
				}

				adjacencyData(std::move(adjacencyVis), std::move(adjacencyVisColor));
			}
	#endif
		}

	#ifdef DEBUG_VISUALIZATION
		saveCoarseToPLY("AfterDeletion.ply");
	#endif

		std::vector<std::vector<TaggedLink>> adj_extracted;
		std::vector<size_t> newVertices, newEdges, newTris, newQuads;

		extract_graph(modifiedVertices, adj_extracted, newVertices, duplicatesAboveIndex);

		extract_faces(adj_extracted, newVertices, newEdges, newTris, newQuads);

		mapAttributes(modifiedVertices, newVertices, newEdges, newTris, newQuads);

		//Remove all vertices that have no incident edges.
		//Start by setting the correspondence of point that reference these vertex to INVALID.
		for (int i = 0; i < modifiedVertices.includedVertices; ++i)
		{
			auto& corr = modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i));
			if (corr != INVALID && vertices[corr].incidentEdges.size() == 0)
				corr = INVALID;
		}
		for (auto vid : newVertices)
			if (vertices[vid].incidentEdges.size() == 0)
				vertices.erase(vid);

		b.earlyExit();

		post_extract();

	#ifdef DEBUG_VISUALIZATION
		saveCoarseToPLY("extracted.ply");
		saveFineToPLY("extractedFine.ply");
	#endif
	}

	template <typename Hierarchy, typename Index>
	void ExtractedMesh::deleteModifiedGeometry(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices, const std::vector<MeshVertexType>& removeVertices)
	{
		nse::util::TimedBlock b("Deleting geometry that changed ..");
		std::set<uint32_t> deletedVertices, deletedEdges, deletedDependentEdges;
		//fill the set of vertices that need to be deleted
		for (auto& v : removeVertices)
		{
			if (vertices[v.vertexIndex].generation == v.generation)
				deletedVertices.insert(v.vertexIndex);
		}
		for (int i = 0; i < modifiedVertices.includedVertices; ++i)
		{
			//delete the mesh vertices that are referenced by modified points
			auto meshVertex = modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i));
			auto meshVertexGeneration = modifiedVertices.template attribute<MeshVertexGeneration>(VertexSetIndex(i));
			if (meshVertex != INVALID && vertices[meshVertex].generation == meshVertexGeneration)
			{
				deletedVertices.insert(meshVertex);
				auto dependentIt = vertexToDependentEdges.find(meshVertex);
				if (dependentIt != vertexToDependentEdges.end())
				{
					deletedDependentEdges.insert(dependentIt->second.begin(), dependentIt->second.end());
					vertexToDependentEdges.erase(dependentIt);
				}
			}
		}

		//actually delete the relevant vertices
		for (uint32_t v : deletedVertices)
			vertices.erase(v);

		//now find the edges that are incident to the deleted vertices
		auto edgeIt = edges.begin();
		while (edgeIt != edges.end())
		{
			bool v1Deleted = deletedVertices.find(edgeIt->v[0]) != deletedVertices.end();
			bool v2Deleted = deletedVertices.find(edgeIt->v[1]) != deletedVertices.end();
			if (v1Deleted || v2Deleted || deletedDependentEdges.find(edgeIt.index()) != deletedDependentEdges.end())
			{
				vertexPairToEdge.erase(std::make_pair(edgeIt->v[0], edgeIt->v[1]));
				vertexPairToEdge.erase(std::make_pair(edgeIt->v[1], edgeIt->v[0]));
				deletedEdges.insert(edgeIt.index());
				auto eid = edgeIt.index();
				if (!v1Deleted)
				{
					auto& c = vertices[edgeIt->v[0]].incidentEdges;
					c.erase(std::remove(c.begin(), c.end(), eid), c.end());
				}
				if (!v2Deleted)
				{
					auto& c = vertices[edgeIt->v[1]].incidentEdges;
					c.erase(std::remove(c.begin(), c.end(), eid), c.end());
				}
				edgeIt = edges.erase(edgeIt);
			}
			else
				++edgeIt;
		}

		for (auto it = vertexToDependentEdges.begin(); it != vertexToDependentEdges.end(); )
		{
			for (auto e : deletedEdges)
			{
				it->second.erase(std::remove(it->second.begin(), it->second.end(), e), it->second.end());
			}
			if (it->second.size() == 0)
				it = vertexToDependentEdges.erase(it);
			else
				++it;
		}

		//Finally, find the triangles and quads that are incident to a deleted edge.
		auto triIt = triangles.begin();
		while (triIt != triangles.end())
		{
			bool edgeErased[3] = { deletedEdges.find(edgeIndex(triIt->edges[0])) != deletedEdges.end(),
				deletedEdges.find(edgeIndex(triIt->edges[1])) != deletedEdges.end(),
				deletedEdges.find(edgeIndex(triIt->edges[2])) != deletedEdges.end() };
			if (edgeErased[0] || edgeErased[1] || edgeErased[2])
			{
				for (int i = 0; i < 3; ++i)
				{
					if (edgeErased[i])
						continue;
					auto& edge = edges[edgeIndex(triIt->edges[i])];
					auto tid = triIt.index();
					edge.incidentTriangles.erase(std::remove_if(edge.incidentTriangles.begin(), edge.incidentTriangles.end(), [tid](uint32_t face) { return tid == face; }));
				}
				triIt = triangles.erase(triIt);
			}
			else
				++triIt;
		}

		auto quadIt = quads.begin();
		while (quadIt != quads.end())
		{
			bool edgeErased[4] = { deletedEdges.find(edgeIndex(quadIt->edges[0])) != deletedEdges.end(),
				deletedEdges.find(edgeIndex(quadIt->edges[1])) != deletedEdges.end(),
				deletedEdges.find(edgeIndex(quadIt->edges[2])) != deletedEdges.end(),
				deletedEdges.find(edgeIndex(quadIt->edges[3])) != deletedEdges.end() };
			if (edgeErased[0] || edgeErased[1] || edgeErased[2] || edgeErased[3])
			{
				for (int i = 0; i < 4; ++i)
				{
					if (edgeErased[i])
						continue;
					auto& edge = edges[edgeIndex(quadIt->edges[i])];
					auto qid = quadIt.index();
					edge.incidentQuads.erase(std::remove_if(edge.incidentQuads.begin(), edge.incidentQuads.end(), [qid](uint32_t face) { return qid == face; }));
				}
				quadIt = quads.erase(quadIt);
			}
			else
				++quadIt;
		}
	}

	template <typename Hierarchy, typename Index>
	void ExtractedMesh::extract_graph(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices, std::vector<std::vector<TaggedLink>>& adj_new, std::vector<size_t>& newVertices, size_t checkForDuplicatesAboveIndex)
	{
		const bool deterministic = false;

		Float inv_scale = 1 / meshSettings.scale();

		//Specifies if a mesh vertex is reachable from the unexpanded point set, avoiding (but including) matched duplicates
		std::vector<bool> vertexReachableFromCore;
		//front for BFS
		std::queue<uint32_t> reachableFromCoreBFS;
		uint32_t potentialDuplicateVerticesAboveIndex = INVALID;
		//specifies for each (collapsed) vertex if it might be a duplicate
		std::vector<bool> potentialDuplicate(modifiedVertices.includedVertices, false);
		//specifies for each (collapsed) vertex if it is part of the unexpanded point set
		std::vector<bool> partOfCore(modifiedVertices.includedVertices, true);

	#pragma omp parallel for
		for (int i = checkForDuplicatesAboveIndex; i < modifiedVertices.includedVertices; ++i)
		{
			potentialDuplicate[i] = true; 
			partOfCore[i] = false;
		}
		typedef std::array<uint32_t, 2> PlainEdge;

		{
			uint32_t size32 = (uint32_t)modifiedVertices.includedVertices;
			if (modifiedVertices.includedVertices != size32)
				throw std::runtime_error("The utilized union-find data structure cannot handle " + std::to_string(modifiedVertices.includedVertices) + " points.");

			DisjointSets dset(size32);
			adj_new.clear();
			adj_new.resize(modifiedVertices.includedVertices);		
			typedef std::pair<PlainEdge, float> WeightedEdge;

			tbb::concurrent_vector<WeightedEdge> collapse_edge_vec;
			collapse_edge_vec.reserve(modifiedVertices.includedVertices * 2.5f);

			auto classify_edge = [&](const PlainEdge& e, const Vector3f& pos, const Vector3f& posField, const Vector3f& dirField, const Vector3f& normal)
			{
				//Checks the relative distance of two vertices in position field space and marks the according edge as "to be collapsed" if they are close to each other

				Vector3f pn = modifiedVertices.template attribute<Position>(VertexSetIndex(e[1]));
				Vector3f on = modifiedVertices.template attribute<PosField>(VertexSetIndex(e[1]));
				Vector3f qn = modifiedVertices.template attribute<DirField>(VertexSetIndex(e[1]));
				Vector3f nn = modifiedVertices.template attribute<Normal>(VertexSetIndex(e[1]));

				Vector3f dirCompat1, dirCompat2;
				meshSettings.rosy->findCompatible(dirField, normal, qn, nn, dirCompat1, dirCompat2);

				Float error = 0;
				std::pair<Vector2i, Vector2i> shift = meshSettings.posy->findCompatibleIndex(pos, normal, dirCompat1, posField, pn, nn, dirCompat2, on, meshSettings.scale(), inv_scale, &error);

				Vector2i absDiff = (shift.first - shift.second).cwiseAbs();

				if (absDiff.maxCoeff() > 1 || (absDiff == Vector2i(1, 1) && meshSettings.posy->posy() == 4))
					return; /* Ignore longer-distance links and diagonal lines for quads */
				bool collapse = absDiff.sum() == 0;

				if (collapse)
					collapse_edge_vec.push_back(std::make_pair(e, error));
				else
				{
					uint32_t lockId = e[1];
					while (!dset.try_lock(lockId))
						;
					adj_new[e[0]].push_back(lockId);
					adj_new[lockId].push_back(e[0]);
					dset.unlock(lockId);
				}
			};

			tbb::concurrent_vector<PlainEdge> unprocessedEdges;
			auto classify_edges = [&](const tbb::blocked_range<uint32_t> &range)
			{
				//Classifies every edge of the input adjacency graph of point as "to be collapsed" or not.
				for (uint32_t i = range.begin(); i != range.end(); ++i)
				{
					modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i)) = INVALID;
					modifiedVertices.template attribute<MeshVertexGeneration>(VertexSetIndex(i)) = currentGeneration;
					while (!dset.try_lock(i))
						;

					auto current = VertexSetIndex(i);
					Vector3f p = modifiedVertices.template attribute<Position>(current);
					Vector3f o = modifiedVertices.template attribute<PosField>(current);
					Vector3f q = modifiedVertices.template attribute<DirField>(current);
					Vector3f n = modifiedVertices.template attribute<Normal>(current);

					modifiedVertices.forEachNeighbor(current, [&](const VertexSetIndex& neighbor)
					{
						size_t index = neighbor.index;
						uint32_t j = (uint32_t)index;
						if (j != index)
							return; //make sure that the index is representable as a 32 bit number
					
						if (j < i)
						{
							//this edge must only be processed if it is an asymmetric one.
							bool isAsymmetric = true;
							modifiedVertices.forEachNeighbor(VertexSetIndex(neighbor), [&](const VertexSetIndex& nn)
							{
								if (nn.index == i)
									isAsymmetric = false;
							});

							//process later to avoid deadlock
							if (isAsymmetric)
								unprocessedEdges.push_back(PlainEdge({ j, i }));

							return;
						}

						classify_edge(PlainEdge({ i, j }), p, o, q, n);
					});

					dset.unlock(i);
				}
			};

			std::atomic<uint32_t> nConflicts(0), nItem(0);
			std::vector<uint16_t> nCollapses(modifiedVertices.includedVertices, 0);
			auto collapse_edges = [&](const tbb::blocked_range<uint32_t> &range)
			{
				std::set<uint32_t> temp;

				for (uint32_t i = range.begin(); i != range.end(); ++i)
				{
					const WeightedEdge &we = collapse_edge_vec[nItem++];
					PlainEdge edge = we.first;

					/* Lock both sets and determine the current representative ID */
					bool ignore_edge = false;
					do 
					{
						if (edge[0] > edge[1])
							std::swap(edge[0], edge[1]);
						if (!dset.try_lock(edge[0]))
							continue;
						if (!dset.try_lock(edge[1])) 
						{
							dset.unlock(edge[0]);
							if (edge[1] == edge[0])
							{
								ignore_edge = true;
								break;
							}
							continue;
						}
						break;
					} while (true);

					if (ignore_edge)
						continue;

					bool contained = false;
					for (auto neighbor : adj_new[edge[0]]) 
					{
						if (dset.find(neighbor.id) == edge[1])
						{
							contained = true;
							break;
						}
					}

					if (contained)
					{
						dset.unlock(edge[0]);
						dset.unlock(edge[1]);
						nConflicts++;
						continue;
					}

					//at this point, the two vertices can be merged
				
					//gather all neighbors of both vertices
					temp.clear();
					for (auto neighbor : adj_new[edge[0]])
						temp.insert(dset.find(neighbor.id));
					for (auto neighbor : adj_new[edge[1]])
						temp.insert(dset.find(neighbor.id));

					//the target index will be either of the two source vertices
					//move all neighbors to the target vertex (the other vertex will have an empty adjacency)
					uint32_t target_idx = dset.unite_index_locked(edge[0], edge[1]);
					adj_new[edge[0]].clear();
					adj_new[edge[1]].clear();
					adj_new[target_idx].reserve(temp.size());
					for (auto j : temp)
						adj_new[target_idx].push_back(j);
					nCollapses[target_idx] = nCollapses[edge[0]] + nCollapses[edge[1]] + 1;
					adj_new[edge[0]].shrink_to_fit();
					adj_new[edge[1]].shrink_to_fit();
				
					potentialDuplicate[target_idx] = potentialDuplicate[edge[0]] || potentialDuplicate[edge[1]];
					partOfCore[target_idx] = partOfCore[edge[0]] || partOfCore[edge[1]];

					dset.unite_unlock(edge[0], edge[1]);
				}
			};

			{
				nse::util::TimedBlock classify("Step 1: Classifying .. ");
				tbb::blocked_range<uint32_t> range1(0u, (uint32_t)modifiedVertices.includedVertices);
				if (!deterministic)
					tbb::parallel_for(range1, classify_edges);
				else
					classify_edges(range1);	

	#pragma omp parallel for
				for (int i = 0; i < unprocessedEdges.size(); ++i)
				{
					auto& e = unprocessedEdges[i];

					while (!dset.try_lock(e[0]))
						;
					auto current = VertexSetIndex(e[0]);
					Vector3f p = modifiedVertices.template attribute<Position>(current);
					Vector3f o = modifiedVertices.template attribute<PosField>(current);
					Vector3f q = modifiedVertices.template attribute<DirField>(current);
					Vector3f n = modifiedVertices.template attribute<Normal>(current);

					classify_edge(e, p, o, q, n);

					dset.unlock(e[0]);
				}
			}
			//checkSymmetry(adj_new);

			{
				nse::util::TimedBlock coll("Step 2: Collapsing " + std::to_string(collapse_edge_vec.size()) + " edges ..");

				struct WeightedEdgeComparator
				{
					bool operator()(const WeightedEdge& e1, const WeightedEdge& e2) const { return e1.second < e2.second; }
				};

				//if (deterministic)
				//    pss::parallel_stable_sort(collapse_edge_vec.begin(), collapse_edge_vec.end(), WeightedEdgeComparator());
				//else
				tbb::parallel_sort(collapse_edge_vec.begin(), collapse_edge_vec.end(), WeightedEdgeComparator());

				tbb::blocked_range<uint32_t> range2(0u, (uint32_t)collapse_edge_vec.size(), GRAIN_SIZE);
				if (!deterministic)
					tbb::parallel_for(range2, collapse_edges);
				else
					collapse_edges(range2);
				if (nConflicts > 0)
					std::cout << "Ignored " << nConflicts << " conflicts." << std::endl;
			}

			//checkSymmetry(adj_new);

			Float avg_collapses = 0;
			{
				nse::util::TimedBlock assign("Step 3: Assigning vertices ..");
				//at this step, the collapsed points (those that survived) are moved to the front of the vertex set
				for (uint32_t i = 0; i < adj_new.size(); ++i) 
				{
					if (adj_new[i].empty()) //if it does not represent a mesh vertex, ignore it
					{
						modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i)) = INVALID;
						continue;
					}

					if (i != newVertices.size()) 
					{
						//move the relevant information to the next available slot
						adj_new[newVertices.size()].swap(adj_new[i]);
						std::swap(nCollapses[newVertices.size()], nCollapses[i]);
					}
					avg_collapses += nCollapses[newVertices.size()];
					auto vid = vertices.insert();
					vertices[vid].generation = currentGeneration;
					newVertices.push_back(vid);
					if (potentialDuplicate[i])
					{
						if (potentialDuplicateVerticesAboveIndex == INVALID)
							potentialDuplicateVerticesAboveIndex = newVertices.size() - 1;
						vertices[vid].generation = currentGeneration - 1; //it might be a duplicate; this value is refined when assigning positions				
					}

					if(partOfCore[i])
					{					
						vertexReachableFromCore.push_back(true); //vertex is in the core
						reachableFromCoreBFS.push(newVertices.size() - 1);					
					}
					else
					{
						vertexReachableFromCore.push_back(false);					
					}
					partOfCore[newVertices.size() - 1] = partOfCore[i]; //map per-point attribute to per-vertex attribute
					modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i)) = newVertices.size() - 1; //temporary value; this will be modified
					auto& v = vertices[newVertices.back()];
					v.normal.setZero();
					v.position.setZero();
				}
				//remove all adjacency information of points that do not represent a mesh vertex
				adj_new.resize(newVertices.size());
				adj_new.shrink_to_fit();
				avg_collapses /= newVertices.size();

				//map point indices to mesh vertex indices
				tbb::parallel_for(
					tbb::blocked_range<uint32_t>(0u, newVertices.size(), GRAIN_SIZE),
					[&](const tbb::blocked_range<uint32_t> &range)
				{
					std::set<uint32_t> temp;
					for (uint32_t i = range.begin(); i != range.end(); ++i)
					{
						temp.clear();
						for (auto k : adj_new[i])
							temp.insert(modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(dset.find(k.id))));
						adj_new[i].clear();
						for (auto j : temp)
							adj_new[i].push_back(TaggedLink(j));
					}
				});
			}

			//checkSymmetry(adj_new);
			if (remove_spurious_vertices)
			{
				nse::util::TimedBlock remov("Step 3a: Removing spurious vertices ..");
				std::cout.flush();
				uint32_t removed = 0;
				uint16_t spurious_threshold = std::min<uint16_t>(20u, avg_collapses / 10);
				for (uint32_t i = 0; i<adj_new.size(); ++i) 
				{
					if (nCollapses[i] > spurious_threshold)
						continue;

					for (auto neighbor : adj_new[i]) 
					{
						auto &a = adj_new[neighbor.id];
						a.erase(std::remove_if(a.begin(), a.end(), [&](const TaggedLink &v) { return v.id == i; }), a.end());
					}

					adj_new[i].clear();
					++removed;
				}
				if (removed > 0)
					std::cout << "removed " << removed << " vertices." << std::endl;
			}

			{
				nse::util::TimedBlock b("Step 4: Assigning positions to vertices ..");

				{
					Eigen::VectorXf cluster_weight(newVertices.size());
					cluster_weight.setZero();

					tbb::blocked_range<uint32_t> range(0u, modifiedVertices.includedVertices, GRAIN_SIZE);
				
					auto map = [&](const tbb::blocked_range<uint32_t> &range)
					{
						for (uint32_t i = range.begin(); i < range.end(); ++i)
						{
							auto j = modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(dset.find(i)));
							if (j == INVALID )
								continue;

							auto current = VertexSetIndex(i);
							modifiedVertices.template attribute<MeshVertex>(current) = j;

							Float weight = std::exp(-(modifiedVertices.template attribute<PosField>(current) - modifiedVertices.template attribute<Position>(current)).squaredNorm() * inv_scale * inv_scale * 9);

							auto& v = vertices[newVertices[j]];
							for (uint32_t k = 0; k < 3; ++k) 
							{
								nse::data::atomicAdd(&v.position.coeffRef(k), modifiedVertices.template attribute<PosField>(current)(k)*weight);
								nse::data::atomicAdd(&v.normal.coeffRef(k), modifiedVertices.template attribute<Normal>(current)(k)*weight);
							}
							nse::data::atomicAdd(&cluster_weight[j], weight);
						}
					};

					if (!deterministic)
						tbb::parallel_for(range, map);
					else
						map(range);

	#pragma omp parallel for
					for (int i = 0; i < newVertices.size(); ++i)
					{
						if (cluster_weight[i] == 0) 
						{
							std::cout << "Warning: vertex " << i << " did not receive any contributions!" << std::endl;
							continue;
						}
						auto& v = vertices[newVertices[i]];
						v.position /= cluster_weight[i];
						v.normal.normalize();					
					}
				}
			}
		}

		{
			nse::util::TimedBlock b("Step 4a: Updating mesh vertex indices ..");
			//update to the real vertex index
	#pragma omp parallel for
			for (int i = 0; i < modifiedVertices.includedVertices; ++i)
			{
				uint32_t& j = modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i));
				if (j == INVALID)
					continue;
				j = newVertices[j];
			}
		}

		if (remove_unnecessary_edges)
		{
			nse::util::TimedBlock b("Step 5: Snapping and removing unnecessary edges ..");
			bool changed;
			uint32_t nRemoved = 0, nSnapped = 0;
			do
			{
				changed = false;
				std::cout << ".";
				std::cout.flush();

				bool changed_inner;
				do
				{
					changed_inner = false;
					Float thresh = 0.3f * meshSettings.scale();

					std::vector<std::tuple<Float, uint32_t, uint32_t, uint32_t>> candidates;
					for (uint32_t i_id = 0; i_id < adj_new.size(); ++i_id)
					{
						auto const &adj_i = adj_new[i_id];
						const Vector3f p_i = vertices[newVertices[i_id]].position;
						for (uint32_t j = 0; j<adj_i.size(); ++j)
						{
							uint32_t j_id = adj_i[j].id;
							const Vector3f p_j = vertices[newVertices[j_id]].position;
							auto const &adj_j = adj_new[j_id];

							for (uint32_t k = 0; k<adj_j.size(); ++k)
							{
								uint32_t k_id = adj_j[k].id;
								if (k_id == i_id)
									continue;
								const Vector3f p_k = vertices[newVertices[k_id]].position;
								Float a = (p_j - p_k).norm(), b = (p_i - p_j).norm(), c = (p_i - p_k).norm();
								if (a > std::max(b, c))
								{
									//Heron's formula
									Float s = 0.5f * (a + b + c);
									Float height = 2 * std::sqrt(s*(s - a)*(s - b)*(s - c)) / a;
									if (height < thresh)
										candidates.push_back(std::make_tuple(height, i_id, j_id, k_id));
								}
							}
						}
					}

					std::sort(candidates.begin(), candidates.end(), [&](
						const decltype(candidates)::value_type &v0,
						const decltype(candidates)::value_type &v1)
					{ return std::get<0>(v0) < std::get<0>(v1); });

					for (auto t : candidates)
					{
						uint32_t i = std::get<1>(t), j = std::get<2>(t), k = std::get<3>(t);
						bool edge1 = std::find_if(adj_new[i].begin(), adj_new[i].end(),
							[j](const TaggedLink &l) { return l.id == j; }) != adj_new[i].end();
						bool edge2 = std::find_if(adj_new[j].begin(), adj_new[j].end(),
							[k](const TaggedLink &l) { return l.id == k; }) != adj_new[j].end();
						bool edge3 = std::find_if(adj_new[k].begin(), adj_new[k].end(),
							[i](const TaggedLink &l) { return l.id == i; }) != adj_new[k].end();

						if (!edge1 || !edge2)
							continue;

						ExtractionHelper::Vertex 
							&v_i = vertices[newVertices[i]], 
							&v_j = vertices[newVertices[j]], 
							&v_k = vertices[newVertices[k]];
						Float a = (v_j.position - v_k.position).norm(), 
							b = (v_i.position - v_j.position).norm(), 
							c = (v_i.position - v_k.position).norm();
						Float s = 0.5f * (a + b + c);
						Float height = 2 * std::sqrt(s*(s - a)*(s - b)*(s - c)) / a;
						if (height != std::get<0>(t))
							continue;
						if ((v_i.position - v_j.position).norm() < thresh || (v_i.position - v_k.position).norm() < thresh)
						{
							const ExtractionHelper::Vertex& merge_v = ((v_i.position - v_j.position).norm() < thresh ? v_j : v_k);
							size_t merge_id = ((v_i.position - v_j.position).norm() < thresh ? j : k);
							v_i.position = (v_i.position + merge_v.position) * 0.5f;
							v_i.normal = (v_i.normal + merge_v.normal).normalized();
							std::set<uint32_t> adj_updated;
							for (auto const &n : adj_new[merge_id])
							{
								if (n.id == i)
									continue;
								adj_updated.insert(n.id);
								for (auto &n2 : adj_new[n.id])
								{
									if (n2.id == merge_id)
										n2.id = i;
								}
							}
							for (auto &n : adj_new[i])
								adj_updated.insert(n.id);
							adj_updated.erase(i);
							adj_updated.erase(merge_id);
							adj_new[merge_id].clear();
							adj_new[i].clear();
							for (uint32_t l : adj_updated)
								adj_new[i].push_back(l);
						}
						else
						{
							Vector3f n_k = v_k.normal, n_j = v_j.normal;

							v_i.position = (v_j.position + v_k.position) * 0.5f;
							v_i.normal = (v_j.normal + v_k.normal).normalized();

							adj_new[j].erase(std::remove_if(adj_new[j].begin(), adj_new[j].end(),
								[k](const TaggedLink &l) { return l.id == k; }), adj_new[j].end());
							adj_new[k].erase(std::remove_if(adj_new[k].begin(), adj_new[k].end(),
								[j](const TaggedLink &l) { return l.id == j; }), adj_new[k].end());

							if (!edge3)
							{
								adj_new[i].push_back(k);
								adj_new[k].push_back(i);
							}
						}

						changed = true;
						changed_inner = true;
						++nSnapped;
					}
				} while (changed_inner);

				if (meshSettings.posy->posy() == 4)
				{
					std::vector<std::pair<Float, PlainEdge>> candidates;
					for (uint32_t i = 0; i < adj_new.size(); ++i)
					{
						auto const &adj_i = adj_new[i];
						const ExtractionHelper::Vertex& v_i = vertices[newVertices[i]];
						for (uint32_t j = 0; j < adj_i.size(); ++j)
						{
							uint32_t j_id = adj_i[j].id;
							const ExtractionHelper::Vertex& v_j = vertices[newVertices[j_id]];

							uint32_t nTris = 0;
							Float length = 0.0f;
							for (uint32_t k = 0; k < adj_i.size(); ++k)
							{
								uint32_t k_id = adj_i[k].id;
								if (k_id == j_id)
									continue;
								const ExtractionHelper::Vertex& v_k = vertices[newVertices[k_id]];
								if (std::find_if(adj_new[j_id].begin(), adj_new[j_id].end(),
									[k_id](const TaggedLink &l) { return l.id == k_id; }) == adj_new[j_id].end())
									continue;
								nTris++;
								length += (v_k.position - v_i.position).norm() + (v_k.position - v_j.position).norm();
							}

							if (nTris == 2)
							{
								Float exp_diag = length / 4 * std::sqrt(2.f);
								Float diag = (v_i.position - v_j.position).norm();
								Float score = std::abs((diag - exp_diag) / std::min(diag, exp_diag));
								candidates.push_back(std::make_pair(std::abs(score), PlainEdge({ i, j_id })));
							}
						}
					}
					std::sort(candidates.begin(), candidates.end(), [&](
						const std::pair<Float, PlainEdge> &v0,
						const std::pair<Float, PlainEdge> &v1) { return v0.first < v1.first; });

					for (auto c : candidates)
					{
						uint32_t i_id = c.second[0], j_id = c.second[1];
						auto const &adj_i = adj_new[i_id];
						uint32_t nTris = 0;
						for (uint32_t k = 0; k < adj_i.size(); ++k)
						{
							uint32_t k_id = adj_i[k].id;
							if (std::find_if(adj_new[j_id].begin(), adj_new[j_id].end(),
								[k_id](const TaggedLink &l) { return l.id == k_id; }) == adj_new[j_id].end())
								continue;
							nTris++;
						}
						if (nTris == 2)
						{
							adj_new[i_id].erase(std::remove_if(adj_new[i_id].begin(), adj_new[i_id].end(),
								[j_id](const TaggedLink &l) { return l.id == j_id; }), adj_new[i_id].end());
							adj_new[j_id].erase(std::remove_if(adj_new[j_id].begin(), adj_new[j_id].end(),
								[i_id](const TaggedLink &l) { return l.id == i_id; }), adj_new[j_id].end());
							changed = true;
							++nRemoved;
						}
					}
				}
			} while (changed);
			if (nSnapped > 0)
				std::cout << "Snapped " << nSnapped << " vertices." << std::endl;
			if (nRemoved > 0)
				std::cout << "Removed " << nRemoved << " vertices." << std::endl;
		}

		{
			nse::util::TimedBlock b("Removing degenerate vertices ..");
			//with exactly one adjacent vertex

			//vertices that need to be re-checked because one of its neighbors has been removed
			std::vector<uint32_t> recheck;

			auto checkVertex = [&](int i)
			{
				if (adj_new[i].size() == 1)
				{
					auto adjVert = adj_new[i][0].id;
					adj_new[adjVert].erase(std::remove_if(adj_new[adjVert].begin(), adj_new[adjVert].end(),
						[i](const TaggedLink &l) { return l.id == i; }), adj_new[adjVert].end());
					adj_new[i].clear();
					if (adjVert < i)
						recheck.push_back(adjVert);
				}
			};
		
			for (int i = 0; i < newVertices.size(); ++i)
			{
				checkVertex(i);
			}

			while (!recheck.empty())
			{
				std::vector<uint32_t> oldrecheck;
				oldrecheck.swap(recheck);
				for (auto i : oldrecheck)
					checkVertex(i);
			}
		}

		{
			nse::util::TimedBlock b("Step 6: Orienting edges ..");

			tbb::parallel_for(
				tbb::blocked_range<uint32_t>(0u, (uint32_t)newVertices.size(), GRAIN_SIZE),
				[&](const tbb::blocked_range<uint32_t> &range)
			{
				for (uint32_t i = range.begin(); i != range.end(); ++i)
				{
					ExtractionHelper::Vertex& v = vertices[newVertices[i]];
					Vector3f s, t, p = v.position;
					coordinate_system(v.normal, s, t);

					std::sort(adj_new[i].begin(), adj_new[i].end(),
						[&](const TaggedLink &j0, const TaggedLink &j1)
					{
						Vector3f v0 = vertices[newVertices[j0.id]].position - p, v1 = vertices[newVertices[j1.id]].position - p;
						return std::atan2(t.dot(v0), s.dot(v0)) > std::atan2(t.dot(v1), s.dot(v1));
					});
				}
			});
		}

		if (potentialDuplicateVerticesAboveIndex < newVertices.size())
		{
			{
				nse::util::TimedBlock b("Finding duplicate vertices ..");
				std::map<uint32_t, uint32_t> duplicateMeshVertexToNewVertexIndex;
				std::map<uint32_t, uint32_t> oldVertexIdToNewVertexId;
				for (uint32_t i = potentialDuplicateVerticesAboveIndex; i < newVertices.size(); ++i)
				{
					if (adj_new[i].size() == 0)
						continue;

					auto& v = vertices[newVertices[i]];
					if (v.generation != currentGeneration)
					{
						//check if it is really a duplicate
						size_t dupIndex;
						if (findDuplicateVertex(v.position, dupIndex, newVertices[i]))
						{						
							oldVertexIdToNewVertexId[newVertices[i]] = dupIndex;
							auto dupUsage = duplicateMeshVertexToNewVertexIndex.find(dupIndex);
							if (dupUsage == duplicateMeshVertexToNewVertexIndex.end())
							{
								vertices.erase(newVertices[i]);
								newVertices[i] = dupIndex;
								duplicateMeshVertexToNewVertexIndex[dupIndex] = i;
							}
							else
							{
								//The target vertex has already been used for another vertex. 
								//Merge the adjacency lists.
								std::set<uint32_t> newAdj;
								for (auto& adj : adj_new[dupUsage->second])
									if (adj.id != i)
										newAdj.insert(adj.id);
								for (auto& adj : adj_new[i])
								{
									if (adj.id != dupUsage->second)
									{
										newAdj.insert(adj.id);
										//update the adjacencies of the neighbors
										for (auto& adjNeighbor : adj_new[adj.id])
											if (adjNeighbor.id == i)
												adjNeighbor.id = dupUsage->second;
									}
								}
								adj_new[i].clear();
								adj_new[dupUsage->second].clear();
								for (auto id : newAdj)
									adj_new[dupUsage->second].emplace_back(id);
							}
						}
						else
							v.generation = currentGeneration;
					}
				}

				//finally, update the vertex correspondences for changed duplicates
				for (int i = checkForDuplicatesAboveIndex; i < modifiedVertices.includedVertices; ++i)
				{
					auto& corr = modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i));
					if (corr == INVALID)
						continue;
					auto it = oldVertexIdToNewVertexId.find(corr);
					if (it != oldVertexIdToNewVertexId.end())
					{
						corr = it->second;
						modifiedVertices.template attribute<MeshVertexGeneration>(VertexSetIndex(i)) = vertices[corr].generation;
					}
				}
			}
			{
				nse::util::TimedBlock b("Removing unmatched duplicates ..");
				//perform BFS from the unexpanded point set and stop at matched duplicates
				while (!reachableFromCoreBFS.empty())
				{
					uint32_t i = reachableFromCoreBFS.front();
					reachableFromCoreBFS.pop();
				
					if (vertices[newVertices[i]].generation != currentGeneration && !partOfCore[i])
					{
						//duplicate vertex

						for (int j = 0; j < adj_new[i].size(); ++j)
						{
							auto current = i;
							auto prev = adj_new[i][j].id;

							int length = 1;

							//trace the according face and mark all duplicates on it as reachable
							while (length < 8)
							{
								if (vertices[newVertices[current]].generation != currentGeneration)
								{
									vertexReachableFromCore[current] = true;
								}

								uint32_t prevIdxInAdjacency = INVALID;
								for (int k = 0; k < adj_new[current].size(); ++k)
									if (adj_new[current][k].id == prev)
									{
										prevIdxInAdjacency = k;
										break;
									}

								if (prevIdxInAdjacency == INVALID || adj_new[current].size() == 1)
									break;

								//move on
								++length;
								current = adj_new[current][(prevIdxInAdjacency + 1) % adj_new[current].size()].id;
								prev = current;
								if (current == i)
									break;
							}
						}
					}
					else
					{
						//no duplicate vertex
						//(or a duplicate that is part of the core - this may happen if a point referenced vertex v (which is deleted) but after optimization references another vertex that already exists)

						for (auto& n : adj_new[i])
						{
							if (!vertexReachableFromCore[n.id] || vertices[newVertices[n.id]].generation != currentGeneration)
								//if (vertices[newVertices[n.id]].generation != currentGeneration || (vertices[newVertices[n.id]].generation == currentGeneration && vertices[newVertices[i]].generation == currentGeneration))
							{
								if(!vertexReachableFromCore[n.id])
									reachableFromCoreBFS.push(n.id);
								vertexReachableFromCore[n.id] = true;
							}
						}
					}
				}


	#ifdef DEBUG_VISUALIZATION
				calculateCollapsedGraphVisualization(newVertices, partOfCore, vertexReachableFromCore, adj_new);
	#endif

				//remove unreachable vertices by clearing their adjacency
				for (uint32_t i = potentialDuplicateVerticesAboveIndex; i < newVertices.size(); ++i)
				{
					if (!vertexReachableFromCore[i])
					{
						for (auto& n : adj_new[i])
						{
							if (vertexReachableFromCore[n.id])
								adj_new[n.id].erase(std::remove_if(adj_new[n.id].begin(), adj_new[n.id].end(), [i](auto& entry) { return i == entry.id; }), adj_new[n.id].end());
						}
						adj_new[i].clear();
					}
				}
			}
		}
		else
		{
	#ifdef DEBUG_VISUALIZATION
			calculateCollapsedGraphVisualization(newVertices, partOfCore, vertexReachableFromCore, adj_new);
	#endif
		}
	}

	extern Vector4f repairSolution(const Vector4f& sol);

	template <typename Hierarchy, typename Index>
	void ExtractedMesh::mapAttributes(PreparedVertexSet<Hierarchy, Index, true, false>& modifiedVertices, const std::vector<size_t>& newVertices, const std::vector<size_t>& newEdges, const std::vector<size_t>& newTris, const std::vector<size_t>& newQuads)
	{
		assert(R >= 2); //there must be at least one texel on an edge

		nse::util::TimedBlock b("Mapping attributes onto surface ..", true);

		//set up indices in texel vector
		uint32_t nextIndex = 0;	
	
		for (int i = 0; i < newVertices.size(); ++i)
		{
			auto& v = vertices[newVertices[i]];
			if (v.generation == currentGeneration)
				if (v.incidentEdges.size() > 0)
					v.indexInTexelVector = nextIndex++;
				else
					v.generation = currentGeneration - 1;
		}
	
		for (auto e : newEdges)
		{
			edges[e].indexInTexelVector = nextIndex;
			nextIndex += texelsPerEdge;
		}	
	
		for (auto t : newTris)
		{
			triangles[t].indexInTexelVector = nextIndex;
			nextIndex += texelsPerTri;
		}	

		for (auto q : newQuads)
		{
			quads[q].indexInTexelVector = nextIndex;
			nextIndex += texelsPerQuad;
		}
		int totalTexels = nextIndex;	
		if (totalTexels == 0)
			return;

		//count the neighbors in the two-ring for each element
		VectorXi entriesPerRow(totalTexels);
	#pragma omp parallel for
		for (int i = 0; i < newVertices.size(); ++i)
		{
			auto& v = vertices[newVertices[i]];		
			if (v.generation == currentGeneration)
			{			
				std::set<size_t> incidentTris, incidentQuads;
				for (auto eid : v.incidentEdges)
				{
					auto& e = edges[eid];
					for (auto f : e.incidentTriangles)
						incidentTris.insert(f);
					for (auto f : e.incidentQuads)
						incidentQuads.insert(f);
				}
				entriesPerRow[v.indexInTexelVector] = 1 + v.incidentEdges.size() * 2 + incidentTris.size() + incidentQuads.size();
			}
		}

	#pragma omp parallel for
		for (int i = 0; i < newEdges.size(); ++i)
		{
			auto& e = edges[newEdges[i]];
		
			entriesPerRow[e.indexInTexelVector] = 4 + (e.incidentQuads.size() + e.incidentTriangles.size()) * 3 + (vertices[e.v[0]].incidentEdges.size() - 1);
			for (int j = 1; j < texelsPerEdge - 1; ++j)
				entriesPerRow[e.indexInTexelVector + j] = 5 + (e.incidentQuads.size() + e.incidentTriangles.size()) * 4;
			entriesPerRow[e.indexInTexelVector + texelsPerEdge - 1] = 4 + (e.incidentQuads.size() + e.incidentTriangles.size()) * 3 + (vertices[e.v[1]].incidentEdges.size() - 1);
		}

	#pragma omp parallel for
		for (int i = 0; i < newTris.size(); ++i)
		{
			auto& tri = triangles[newTris[i]];
			for (int edge = 0; edge < 3; ++edge)
			{			
				int prevEdge = (edge + 2) % 3;
				auto& e = edges[edgeIndex(tri.edges[edge])];
				auto& prevE = edges[edgeIndex(tri.edges[prevEdge])];
				entriesPerRow[tri.indexInTexelVector + edge * (texelsPerEdge - 1)] = 9;
				entriesPerRow[tri.indexInTexelVector + edge * (texelsPerEdge - 1)] += e.incidentQuads.size() + e.incidentTriangles.size();
				entriesPerRow[tri.indexInTexelVector + edge * (texelsPerEdge - 1)] += prevE.incidentQuads.size() + prevE.incidentTriangles.size();
				for (int j = 1; j < texelsPerEdge; ++j)
					entriesPerRow[tri.indexInTexelVector + edge * (texelsPerEdge - 1) + j] = 11 + e.incidentQuads.size() + e.incidentTriangles.size();
			}
			for (int j = 3 * texelsPerEdge - 3; j < texelsPerTri; ++j)
				entriesPerRow[tri.indexInTexelVector + j] = 13;
		}
	#pragma omp parallel for
		for(int i = 0; i < newQuads.size(); ++i)
		{
			auto& quad = quads[newQuads[i]];		
			auto& bottomE = edges[edgeIndex(quad.edges[0])];
			auto& rightE = edges[edgeIndex(quad.edges[1])];
			auto& topE = edges[edgeIndex(quad.edges[2])];
			auto& leftE = edges[edgeIndex(quad.edges[3])];

			int rowIdx = 0;
			entriesPerRow[quad.indexInTexelVector + 0] = 9 + bottomE.incidentQuads.size() + bottomE.incidentTriangles.size() + leftE.incidentQuads.size() + leftE.incidentTriangles.size();
			for (int x = 1; x < texelsPerEdge - 1; ++x)
			{
				entriesPerRow[quad.indexInTexelVector + x] = 11 + bottomE.incidentQuads.size() + bottomE.incidentTriangles.size();
			}
			entriesPerRow[quad.indexInTexelVector + texelsPerEdge - 1] = 9 + bottomE.incidentQuads.size() + bottomE.incidentTriangles.size() + rightE.incidentQuads.size() + rightE.incidentTriangles.size();

			for (int y = 1; y < texelsPerEdge - 1; ++y)
			{
				rowIdx = y * texelsPerEdge;
				entriesPerRow[quad.indexInTexelVector + rowIdx] = 11 + edges[edgeIndex(quad.edges[3])].incidentQuads.size() + edges[edgeIndex(quad.edges[3])].incidentTriangles.size();
				for (int x = 1; x < texelsPerEdge - 1; ++x)
				{
					entriesPerRow[quad.indexInTexelVector + rowIdx + x] = 13;
				}
				entriesPerRow[quad.indexInTexelVector + rowIdx + texelsPerEdge - 1] = 11 + edges[edgeIndex(quad.edges[1])].incidentQuads.size() + edges[edgeIndex(quad.edges[1])].incidentTriangles.size();
			}

			rowIdx = (texelsPerEdge - 1) * texelsPerEdge;
			entriesPerRow[quad.indexInTexelVector + rowIdx + 0] = 9 + topE.incidentQuads.size() + topE.incidentTriangles.size() + leftE.incidentQuads.size() + leftE.incidentTriangles.size();
			for (int x = 1; x < texelsPerEdge - 1; ++x)
			{
				entriesPerRow[quad.indexInTexelVector + rowIdx + x] = 11 + topE.incidentQuads.size() + topE.incidentTriangles.size();
			}
			entriesPerRow[quad.indexInTexelVector + rowIdx + texelsPerEdge - 1] = 9 + topE.incidentQuads.size() + topE.incidentTriangles.size() + rightE.incidentQuads.size() + rightE.incidentTriangles.size();
		}

		//Set up the least squares system that we solve to find the attributes for each texel
	#ifdef GEOMETRIC_LAPLACIAN
		GeometricLeastSquaresSystemBuilder 
	#else
		HeightFieldLeastSquaresSystemBuilder
	#endif
			systemBuilder(totalTexels);
		systemBuilder.reserve(entriesPerRow);
		
		{
			nse::util::TimedBlock b("Constructing Least-Squares System ..");		

			prepareLaplacian(newVertices, newEdges, newTris, newQuads, meshSettings.smoothness, systemBuilder);		
		
			{
				nse::util::TimedBlock b("Adding interpolation constraints ..");

				//Add interpolation constraints for every modified point
	#pragma omp parallel for
				for (int i = 0; i < modifiedVertices.includedVertices; ++i)
				{
					uint32_t correspondingMeshVertex = modifiedVertices.template attribute<MeshVertex>(VertexSetIndex(i));
					if (correspondingMeshVertex == INVALID)
						continue; //the point did not produce any geometry

					auto& p = modifiedVertices.template attribute<Position>(VertexSetIndex(i));
					auto& v = vertices[correspondingMeshVertex];

					//Find the incident faces
					std::set<uint32_t> incidentQuads, incidentTris;
					for (auto eid : v.incidentEdges)
					{
						auto& e = edges[eid];
						for (auto tid : e.incidentTriangles)
							incidentTris.insert(tid);
						for (auto qid : e.incidentQuads)
							incidentQuads.insert(qid);
					}

					float bestH = meshSettings.scale(); //h is the projection height over the face. Ignore faces where this height is larger than the target edge length
					int bestFace = 0; //negative for triangles, positive for quads, zero specifies no face
					Vector2f bestUV;

					Vector2f uv;
					Vector3f ip, in; //interpolated position and normal

					for (auto tid : incidentTris)
					{
						auto& tri = triangles[tid];
						projectPointOnTriangle(tri, p, uv, ip, in);
						if (uv.x() < 0 || uv.y() < 0 || (1 - uv.x() - uv.y()) < 0)
							continue; //projection falls outside of triangle

						float h = (p - ip).dot(in) / in.squaredNorm();
						if ((ip + h * in - p).squaredNorm() > meshSettings.scale() * meshSettings.scale() * 0.001f)
							continue; //the projection failed
						if (std::abs(h) < std::abs(bestH))
						{
							bestH = h;
							bestFace = -tid - 1;
							bestUV = uv;
						}
					}

					for (auto qid : incidentQuads)
					{
						auto& quad = quads[qid];
						projectPointOnQuad(quad, p, uv, ip, in);
						if (uv.x() < 0 || uv.x() > 1 || uv.y() < 0 || uv.y() > 1)
							continue; //projection falls outside of quad

						float h = (p - ip).dot(in) / in.squaredNorm();
						if ((ip + h * in - p).squaredNorm() > meshSettings.scale() * meshSettings.scale() * 0.001f)
							continue; //the projection failed
						if (std::abs(h) < std::abs(bestH))
						{
							bestH = h;
							bestFace = qid + 1;
							bestUV = uv;
						}
					}					

					if (bestFace == 0) //no valid projection
						continue;

					auto c = modifiedVertices.template attribute<Color>(VertexSetIndex(i)).template cast<float>();
					Vector4f value(c.x(), c.y(), c.z(), bestH);

					ExtractionHelper::FaceInterpolationInfo interpol[4]; //find the interpolation corners

					if (bestFace < 0)
					{
						//triangle
						int tid = -bestFace - 1;
						auto& tri = triangles[tid];
						
						if (tri.generation != currentGeneration)
							continue;

						getInterpolationInfo(tri, bestUV, interpol);
					}

					if (bestFace > 0)
					{
						//quad
						int qid = bestFace - 1;
						auto& quad = quads[qid];

						if (quad.generation != currentGeneration)
							continue;

						getInterpolationInfo(quad, bestUV, interpol);
					}

					systemBuilder.addInterpolationConstraint(interpol, 4, value, 1 - meshSettings.smoothness, currentGeneration);

				} //omp parallel for each point
			} // nse::util::TimedBlock		

			{
				nse::util::TimedBlock b{ "Repairing initial guess .." };
			
				systemBuilder.closeGapsInGuess();
			}		  

			std::cout << "Number of texels: " << totalTexels << std::endl;
		}	

		Eigen::Matrix<float, -1, 4> solution;
		{
			nse::util::TimedBlock b("Solving ..");
		
			solution = systemBuilder.solve();
		}

		{
			nse::util::TimedBlock("Extracting solution ..");

			for (int i = 0; i < newVertices.size(); ++i)
			{
				if (vertices[newVertices[i]].generation != currentGeneration)
					continue;
				auto& v = vertices[newVertices[i]];
				v.colorDisplacement = repairSolution(solution.row(v.indexInTexelVector));
			}

			for (auto eid : newEdges)
			{
				auto& e = edges[eid];
				e.colorDisplacement.resize(texelsPerEdge);
				for (int i = 0; i < texelsPerEdge; ++i)
				{				
					e.colorDisplacement[i] = repairSolution(solution.row(e.indexInTexelVector + i));
				}
			}

			for (auto tid : newTris)
			{
				auto& tri = triangles[tid];
				tri.colorDisplacement.resize(texelsPerTri);
				for (int i = 0; i < texelsPerTri; ++i)
				{
					tri.colorDisplacement[i] = repairSolution(solution.row(tri.indexInTexelVector + i));
				}
			}

			for (auto qid : newQuads)
			{
				auto& quad = quads[qid];
				quad.colorDisplacement.resize(texelsPerQuad);
				for (int i = 0; i < texelsPerQuad; ++i)
				{
					quad.colorDisplacement[i] = repairSolution(solution.row(quad.indexInTexelVector + i));
				}
			}
		}	
	}

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