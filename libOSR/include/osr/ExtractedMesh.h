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

#include "osr/MeshSettings.h"
#include "osr/PreparedVertexSet.h"
#include "osr/HierarchyDef.h"
#include <nsessentials/data/PersistentIndexContainer.h>
#include <nsessentials/data/Serialization.h>
#include "osr/ExtractionUtils.h"
#include "osr/MeshVisitor.h"

#include <set>
#include <array>
#include <map>

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
#ifdef GEOMETRIC_LAPLACIAN
	class GeometricLeastSquaresSystemBuilder;
#else
	class HeightFieldLeastSquaresSystemBuilder;
#endif

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
	class OSR_EXPORT ExtractedMesh : public ExtractionHelper::IEntityContainer
	{
	public:	
		ExtractedMesh(const MeshSettings& meshSettings);

#ifdef GEOMETRIC_LAPLACIAN
		typedef GeometricLeastSquaresSystemBuilder MappingBuilder;
#else
		typedef HeightFieldLeastSquaresSystemBuilder MappingBuilder;
#endif

		//Performs the extraction of a part of a point cloud.
		//  modifiedVertices - the set of modified points from the hierarchy.
		//  deleteAndExpand  - specifies if the current extraction result should be cleaned in the surrounding of
		//                     the modified points before calculating the extraction.
		//  removeVertices   - list of old mesh vertices that need to be removed.
		void extract(PreparedVertexSet<THierarchy::VertexIndex, true, false>& modifiedVertices, bool deleteAndExpand = false, const std::vector<MeshVertexType>& removeVertices = std::vector<MeshVertexType>());

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
		void deleteModifiedGeometry(PreparedVertexSet<THierarchy::VertexIndex, true, false>& modifiedVertices, const std::vector<MeshVertexType>& removeVertices);

		//Extracts the collapsed adjacency graph for a given set of modified points.
		//  newVertices - output list of newly created mesh vertices
		//  checkForDuplicatedAboveIndex - points with indices above this value are checked for being duplicates.
		void extract_graph(PreparedVertexSet<THierarchy::VertexIndex, true, false>& modifiedVertices,
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
		void mapAttributes(PreparedVertexSet<THierarchy::VertexIndex, true, false>& modifiedVertices, const std::vector<size_t>& newVertices, const std::vector<size_t>& newEdges, const std::vector<size_t>& newTris, const std::vector<size_t>& newQuads);

		//Adds the uniform graph Laplacian for newly created geometry to the least squares system.
		void prepareLaplacian(const std::vector<size_t>& newVertices, const std::vector<size_t>& newEdges, const std::vector<size_t>& newTris, const std::vector<size_t>& newQuads, float weight, MappingBuilder& systemBuilder);

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