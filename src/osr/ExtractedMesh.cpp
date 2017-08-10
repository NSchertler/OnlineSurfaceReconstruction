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

#include "osr/ExtractedMesh.h"

#include <tbb/tbb.h>
#include <tbb/concurrent_vector.h>
#include <unordered_set>
#include <tuple>
#include <fstream>

#include <nsessentials/util/IndentationLog.h>

using namespace osr;
using namespace ExtractionHelper;

ExtractedMesh::ExtractedMesh(const MeshSettings& meshSettings)
	: meshSettings(meshSettings)
{ }


void ExtractedMesh::reset()
{
	triangles.clear();
	quads.clear();
	edges.clear();
	vertexPairToEdge.clear();
	vertices.clear();

	vertexToDependentEdges.clear();
}

template <typename FaceType, typename IncidentFacesCallback>
void ExtractedMesh::removeIncidentArtefacts(uint32_t eid, nse::data::PersistentIndexContainer<FaceType>& faces, std::set<uint32_t>& removedEdges, std::set<uint32_t>& removedFaces, const IncidentFacesCallback& getIncidentFaces)
{
	for (auto faceIt = getIncidentFaces(edges[eid]).begin(); faceIt != getIncidentFaces(edges[eid]).end(); )
	{
		auto faceId = *faceIt;
		auto& face = faces[faceId];
		bool isArtefact = true;

		//the face is an artefact if all edges are either non-manifold or boundaries
		for (int i = 0; i < FaceType::FaceDegree; ++i)
		{
			auto faceEid = edgeIndex(face.edges[i]);
			auto& e = edges[faceEid];

			if (e.incidentQuads.size() + e.incidentTriangles.size() == 2)
			{
				isArtefact = false;
				break;
			}
		}
		if (isArtefact)
		{
			//the face is an artefact, remove it
			for (int i = 0; i < FaceType::FaceDegree; ++i)
			{
				auto faceEid = edgeIndex(face.edges[i]);
				auto& e = edges[faceEid];

				if (faceEid == eid)
					faceIt = getIncidentFaces(e).erase(faceIt);
				else
					getIncidentFaces(e).erase(std::remove(getIncidentFaces(e).begin(), getIncidentFaces(e).end(), faceId), getIncidentFaces(e).end());

				if (e.incidentQuads.size() + e.incidentTriangles.size() == 1)
				{
					//remove boundary edges
					for (int i = 0; i < 2; ++i)
					{
						auto& incidence = vertices[e.v[i]].incidentEdges;
						incidence.erase(std::remove(incidence.begin(), incidence.end(), faceEid), incidence.end());
					}

					vertexPairToEdge.erase(std::make_pair(e.v[0], e.v[1]));
					vertexPairToEdge.erase(std::make_pair(e.v[1], e.v[0]));

					removedEdges.insert(faceEid);
					edges.erase(faceEid);
				}
			}

			removedFaces.insert(faceId);
			faces.erase(faceId);
		}
		else
			++faceIt;
	}
}

void ExtractedMesh::extract_faces(std::vector<std::vector<TaggedLink>>& adj, std::vector<size_t>& newVertices, std::vector<size_t>& newEdges, std::vector<size_t>& newTris, std::vector<size_t>& newQuads)
{
	VectorXu stats(10);
	stats.setZero();	

	std::set<size_t> nonmanifoldEdges;

	std::vector<std::pair<uint32_t, uint32_t>> result;
	uint32_t nFaces = 0, nHoles = 0;
	{		
		nse::util::TimedBlock b("Step 7: Extracting faces ..");				
		for (uint32_t _deg = 3; _deg <= maxFaceDegree; _deg++)
		{
			uint32_t deg = _deg;
			if (meshSettings.posy->posy() == 4 && (deg == 3 || deg == 4))
				deg = 7 - deg; //start with quads for 4-posy

			for (uint32_t i = 0; i < adj.size(); ++i)
			{
				for (uint32_t j = 0; j < adj[i].size(); ++j) 
				{
					if (!extract_face(i, j, deg, adj, result))
						continue;
					stats[result.size()]++;
					fill_face(result, newVertices, newEdges, newTris, newQuads, nonmanifoldEdges);
					nFaces++;
				}
			}
		}
	}

	if (fill_holes) 
	{
		nse::util::TimedBlock b("Step 8: Filling holes ..");
		for (uint32_t i = 0; i < adj.size(); ++i) 
		{
			for (uint32_t j = 0; j<adj[i].size(); ++j) 
			{
				if (!adj[i][j].used()) 
				{
					uint32_t j_id = adj[i][j].id;
					bool found = false;
					for (uint32_t k = 0; k<adj[j_id].size(); ++k)
					{
						if (adj[j_id][k].id == i) 
						{
							found = true;
							if (adj[j_id][k].used()) 
							{
								adj[i][j].flag |= 2;
								adj[j_id][k].flag |= 2;
							}
							break;
						}
					}
					if (!found)
						std::cout << "Internal error" << std::endl;
				}
			}
		}

		uint32_t linksLeft = 0;
		for (uint32_t i = 0; i < adj.size(); ++i)
		{
			adj[i].erase(std::remove_if(adj[i].begin(), adj[i].end(),
				[](const TaggedLink &l) { return (l.flag & 2) == 0; }), adj[i].end());
			linksLeft += adj[i].size();
		}

		for (uint32_t i = 0; i < adj.size(); ++i)
		{
			for (uint32_t j = 0; j<adj[i].size(); ++j)
			{
				if (!extract_face(i, j, 0, adj, result))
					continue;
				if (result.size() > maxFaceDegree) 
				{
					std::cout << "Not trying to fill a hole of degree " << result.size() << std::endl;
					continue;
				}
				if (result.size() >= (size_t)stats.size())
				{
					uint32_t oldSize = stats.size();
					stats.conservativeResize(result.size() + 1);
					stats.tail(stats.size() - oldSize).setZero();
				}
				stats[result.size()]++;
				fill_face(result, newVertices, newEdges, newTris, newQuads, nonmanifoldEdges);
				nHoles++;
			}
		}
		if (nHoles > 0)
			std::cout << nHoles << " holes." << std::endl;
	}

	{
		bool first = true;
		std::cout << "Intermediate mesh statistics: ";
		for (int i = 0; i<stats.size(); ++i) {
			if (stats[i] == 0)
				continue;
			if (!first)
				std::cout << ", ";
			std::cout << "degree " << i << ": " << stats[i] << (stats[i] == 1 ? " face" : " faces");
			first = false;
		}
		std::cout << std::endl;
	}	

	//remove simple non-manifold artefacts

	std::set<uint32_t> removedEdges, removedTris, removedQuads;

	for (auto eid : nonmanifoldEdges)
	{
		//look for faces that have only boundary or non-manifold edges and remove them
		removeIncidentArtefacts(eid, triangles, removedEdges, removedTris, [this](Edge& e) -> std::vector<uint32_t>& { return e.incidentTriangles; });
		removeIncidentArtefacts(eid, quads, removedEdges, removedQuads, [this](Edge& e) -> std::vector<uint32_t>& { return e.incidentQuads; });
	}

	if (removedEdges.size() > 0)
		newEdges.erase(std::remove_if(newEdges.begin(), newEdges.end(), [&](uint32_t edge) {return removedEdges.find(edge) != removedEdges.end(); }), newEdges.end());
	if (removedTris.size() > 0)
		newTris.erase(std::remove_if(newTris.begin(), newTris.end(), [&](uint32_t tri) {return removedTris.find(tri) != removedTris.end(); }), newTris.end());
	if (removedQuads.size() > 0)
		newQuads.erase(std::remove_if(newQuads.begin(), newQuads.end(), [&](uint32_t quad) {return removedQuads.find(quad) != removedQuads.end(); }), newQuads.end());

	std::cout << "Removed non-manifold artefacts: " << removedEdges.size() << " edges, " << removedTris.size() << " triangles, " << removedQuads.size() << " quads." << std::endl;

#if REMOVE_NONMANIFOLD
	std::cout << "Step 11: Removing nonmanifold elements.. ";
	remove_nonmanifold(F, mV_extracted, Nf);
	std::cout << "done. (took " << timeString(nse::util::Timer.reset()) << ")" << std::endl;
#endif
}

bool ExtractedMesh::extract_face(uint32_t cur, uint32_t curIdx, size_t targetSize,
	std::vector<std::vector<TaggedLink>>& adj, std::vector<std::pair<uint32_t, uint32_t>> &result)
{
	uint32_t initial = cur;
	bool success = false;
	result.clear();
	std::set<uint32_t> vertices;
	while(true)
	{
		if (adj[cur][curIdx].used() ||
			(targetSize > 0 && result.size() + 1 > targetSize))
		{
			break;
		}

		if (result.size() + 1 > maxFaceDegree)
		{
			result.clear();
			return false;
		}

		result.push_back(std::make_pair(cur, curIdx));
		vertices.insert(cur);

		uint32_t next = adj[cur][curIdx].id,
			next_rank = adj[next].size(),
			idx = INVALID;

		for (uint32_t j = 0; j<next_rank; ++j) 
		{
			if (adj[next][j].id == cur)
			{
				idx = j;
				break;
			}
		}

		if (idx == INVALID || next_rank == 1)
			break;

		cur = next;
		curIdx = (idx + 1) % next_rank;
		if (cur == initial) 
		{
			success = targetSize == 0 || result.size() == targetSize;
			break;
		}
	}

	if (vertices.size() != result.size()) //at least one vertex is used twice
		success = false;


	if (success) 
	{
		for (auto kv : result)
			adj[kv.first][kv.second].markUsed();
	}
	else
	{
		result.clear();
	}
	return success;
};

int32_t ExtractedMesh::get_edge(uint32_t v1, uint32_t v2, std::vector<size_t>& newEdges)
{
	auto p = std::make_pair(v1, v2);
	auto e1It = vertexPairToEdge.find(p);
	if (e1It == vertexPairToEdge.end())
	{
		auto eid = edges.insert();
		edges[eid].v = { v1, v2 };
		edges[eid].generation = currentGeneration;
		vertices[v1].incidentEdges.push_back(eid);
		vertices[v2].incidentEdges.push_back(eid);		
		newEdges.push_back(eid);

		e1It = vertexPairToEdge.insert(std::make_pair(p, eid)).first;
		vertexPairToEdge[std::make_pair(v2, v1)] = -eid - 1;
	}
	return e1It->second;
}

float cross(const Vector2f& a, const Vector2f& b)
{
	return a.x() * b.y() - a.y() * b.x();
}

void invBilinear(const Vector2f& p, const Vector2f& p1, const Vector2f& p2, const Vector2f& p3, Vector2f& resultUV)
{
	//  p3 ---- p2
	//   ^      |
	//   v      |
	//   |      |
	//   0 -u-> p1


	float c01 = cross(p, p1);
	float c12 = cross(p1, p2);
	float c13 = cross(p1, p3);
	float c13Mc12 = c13 - c12;
	if (1 - std::abs(p1.dot(p3 - p2) / (p1.norm() * (p3 - p2).norm())) < 0.001f)
	{
		//(0 -> p1) and (p3 -> p2) are parallel		
		resultUV.y() = -c01 / c13;

		Vector2f numU = p - resultUV.y() * p3;
		Vector2f denomU = p1 + resultUV.y() * (p2 - p3 - p1);
		if (std::abs(denomU.x()) > std::abs(denomU.y()))
			resultUV.x() = numU.x() / denomU.x();
		else
			resultUV.x() = numU.y() / denomU.y();
	}
	else
	{
		//(0 -> p1) and (p3 -> p2) are not parallel, hence c13Mc12 is not 0
		float c02 = cross(p, p2);
		float c03 = cross(p, p3);

		float inner = c02 - c01 - c03 - c13;
		float radicand = inner * inner - 4 * c03 * c13Mc12;
		float ad = c01 + c03 + c13 - c02;
		float denom = 2 * c13Mc12;

		if (radicand < 0)
			radicand = 0;

		float rt = sqrt(radicand);
		resultUV.x() = (rt + ad) / denom;
		Vector2f alternative;
		alternative.x() = (-rt + ad) / denom;
		
		Vector2f numV = p - resultUV.x() * p1;
		Vector2f denomV = p3 + resultUV.x() * (p2 - p3 - p1);
		if (std::abs(denomV.x()) > abs(denomV.y()))
			resultUV.y() = numV.x() / denomV.x();
		else
			resultUV.y() = numV.y() / denomV.y();

		numV = p - alternative.x() * p1;
		denomV = p3 + alternative.x() * (p2 - p3 - p1);
		if (std::abs(denomV.x()) > abs(denomV.y()))
			alternative.y() = numV.x() / denomV.x();
		else
			alternative.y() = numV.y() / denomV.y();

		if ((alternative - Vector2f::Constant(0.5f)).squaredNorm() < (resultUV - Vector2f::Constant(0.5f)).squaredNorm())
			resultUV = alternative;
	}
}

float frac(float n)
{
	return n - floor(n);
}

void ExtractedMesh::add_primitive_face(const std::vector<uint32_t>& indices, bool checkForDuplicate, std::vector<size_t>& newEdges, std::vector<size_t>& newTris, std::vector<size_t>& newQuads, std::set<size_t>& nonmanifoldEdges)
{
	if (indices.size() == 3)
	{
		std::array<int32_t, 3> edges = { get_edge(indices[0], indices[1], newEdges), get_edge(indices[1], indices[2], newEdges), get_edge(indices[2], indices[0], newEdges) };

		//check if all edges share a triangle (i.e. we are about to add the backside of an existing triangle)
		std::vector<uint32_t> commonTriangles;
		for (auto tid : this->edges[edgeIndex(edges[0])].incidentTriangles)
			commonTriangles.push_back(tid);
		std::sort(commonTriangles.begin(), commonTriangles.end());
		for (int i = 1; i < 3; ++i)
		{
			std::vector<uint32_t> edgeTriangles;
			for (auto tid : this->edges[edgeIndex(edges[i])].incidentTriangles)
				edgeTriangles.push_back(tid);
			std::sort(edgeTriangles.begin(), edgeTriangles.end());
			std::vector<uint32_t> intersection;
			std::set_intersection(edgeTriangles.begin(), edgeTriangles.end(), commonTriangles.begin(), commonTriangles.end(), std::back_inserter(intersection));
			commonTriangles.swap(intersection);
		}
		if (commonTriangles.size() == 1)
			return;

		Vector3f faceNormal = (vertices[indices[1]].position - vertices[indices[0]].position)
			.cross(vertices[indices[2]].position - vertices[indices[0]].position).normalized();		

		size_t dupIndex;
		if (checkForDuplicate && findDuplicateFace(edges, triangles, dupIndex))
			; //do nothing, face already exists
		else
		{
			auto tid = triangles.insert();
			newTris.push_back(tid);
			triangles[tid].edges = edges;
			triangles[tid].generation = currentGeneration;
				
			for (int i = 0; i < 3; ++i)
			{
				auto& edge = this->edges[edgeIndex(edges[i])];
				edge.incidentTriangles.push_back(tid);
				if (edge.incidentQuads.size() + edge.incidentTriangles.size() > 2)
					nonmanifoldEdges.insert(edgeIndex(edges[i]));
			}
		}
	}
	else if (indices.size() == 4)
	{				
		//check if the quad is planar enough
		auto& p0 = vertices[indices[0]].position;
		auto& p1 = vertices[indices[1]].position;
		auto& p2 = vertices[indices[2]].position;
		auto& p3 = vertices[indices[3]].position;

		float diag1 = (p2 - p0).norm();
		float diag2 = (p3 - p1).norm();
		float nonplanarity = std::abs((p1 - p0).cross(p2 - p0).dot(p3 - p0) / pow(0.5f * (diag1 + diag2), 3));
		if (nonplanarity > 0.1f)
		{
			//tessellate with two triangles
			if (diag1 < diag2)
			{
				add_primitive_face({ indices[0], indices[1], indices[2] }, checkForDuplicate, newEdges, newTris, newQuads, nonmanifoldEdges);
				add_primitive_face({ indices[0], indices[2], indices[3] }, checkForDuplicate, newEdges, newTris, newQuads, nonmanifoldEdges);
			}
			else
			{
				add_primitive_face({ indices[0], indices[1], indices[3] }, checkForDuplicate, newEdges, newTris, newQuads, nonmanifoldEdges);
				add_primitive_face({ indices[1], indices[2], indices[3] }, checkForDuplicate, newEdges, newTris, newQuads, nonmanifoldEdges);
			}
			return;
		}

		std::array<int32_t, 4> edges = { get_edge(indices[0], indices[1], newEdges), get_edge(indices[1], indices[2], newEdges), get_edge(indices[2], indices[3], newEdges), get_edge(indices[3], indices[0], newEdges) };

		Vector3f faceNormal = (
			p0.cross(p1) +
			p1.cross(p2) +
			p2.cross(p3) +
			p3.cross(p0)).normalized();

		size_t dupIndex;
		if (checkForDuplicate && findDuplicateFace(edges, quads, dupIndex))
			; //do nothing, face already exists
		else
		{
			auto qid = quads.insert();
			newQuads.push_back(qid);
			quads[qid].edges = edges;
			quads[qid].generation = currentGeneration;
			for (int i = 0; i < 4; ++i)
			{
				auto& edge = this->edges[edgeIndex(edges[i])];
				edge.incidentQuads.push_back(qid);
				if (edge.incidentQuads.size() + edge.incidentTriangles.size() > 2)
					nonmanifoldEdges.insert(edgeIndex(edges[i]));
			}			
		}
	}
	else
	{
		assert(false); //primitive face must be a triangle or a quad
	}
}

uint32_t ExtractedMesh::startVertex(int32_t edgeId) const
{
	if (edgeId >= 0)
		return edges[edgeId].v[0];
	else
		return edges[-edgeId - 1].v[1];
}

uint32_t ExtractedMesh::edgeIndex(int32_t edgeId) const
{
	if (edgeId >= 0)
		return edgeId;
	else
		return -edgeId - 1;
}


void ExtractedMesh::fill_face(std::vector<std::pair<uint32_t, uint32_t>> &verts, std::vector<size_t>& newVertices, std::vector<size_t>& newEdges, std::vector<size_t>& newTris, std::vector<size_t>& newQuads, std::set<size_t>& nonmanifoldEdges)
{
	const Float optimalAngle = (meshSettings.posy->posy() - 2) * M_PI / meshSettings.posy->posy();

	int newEdgesBefore = newEdges.size();

	while (verts.size() > 2) 
	{
		if (verts.size() <= (size_t)meshSettings.posy->posy()) 
		{
			//regular polygon or smaller
			bool checkForDuplicate = true;
			std::vector<uint32_t> indices(verts.size());
			for (int i = 0; i < verts.size(); ++i)
			{
				indices[i] = newVertices[verts[i].first];
				if (vertices[newVertices[verts[i].first]].generation == currentGeneration)
					checkForDuplicate = false;
			}
			add_primitive_face(indices, checkForDuplicate, newEdges, newTris, newQuads, nonmanifoldEdges);
			break;
		}
		else if (verts.size() - (meshSettings.posy->posy() - 2) >= 4 || meshSettings.posy->posy() == 3)
		{
			//face is not regular and contains enough vertices such that cutting off a regular polygon leaves at least a quad
			
			//cycle through the face and find consecutive vertices that best resemble a regular polygon (in terms of interior angles)
			Float best_score = std::numeric_limits<Float>::infinity();
			uint32_t best_idx = INVALID;

			for (uint32_t i = 0; i<verts.size(); ++i) 
			{
				Float score = 0.f;
				for (int k = 0; k < meshSettings.posy->posy(); ++k) 
				{
					Vector3f v0 = vertices[newVertices[verts[(i + k) % verts.size()].first]].position;
					Vector3f v1 = vertices[newVertices[verts[(i + k + 1) % verts.size()].first]].position;
					Vector3f v2 = vertices[newVertices[verts[(i + k + 2) % verts.size()].first]].position;
					Vector3f d0 = (v0 - v1).normalized();
					Vector3f d1 = (v2 - v1).normalized();
					Float angle = std::acos(d0.dot(d1));
					score += std::abs(angle - optimalAngle);
				}

				if (score < best_score) 
				{
					best_score = score;
					best_idx = i;
				}
			}

			//we have found the position to cut off the regular polygon	
			std::vector<uint32_t> indices(meshSettings.posy->posy());
			bool checkForDuplicate = true;
			for (int i = 0; i < meshSettings.posy->posy(); ++i) 
			{
				uint32_t &j = verts[(best_idx + i) % verts.size()].first;
				indices[i] = newVertices[j];
				if (vertices[newVertices[j]].generation == currentGeneration)
					checkForDuplicate = false;
				if (i != 0 && (int)i != meshSettings.posy->posy() - 1) //keep the boundary vertices
					j = INVALID;
			}
			add_primitive_face(indices, checkForDuplicate, newEdges, newTris, newQuads, nonmanifoldEdges);
			verts.erase(std::remove_if(verts.begin(), verts.end(),
				[](const std::pair<uint32_t, uint32_t> &v) { return v.first == INVALID; }), verts.end());
		}
		else 
		{
			assert(meshSettings.posy->posy() == 4);
			//number of remaining vertices must be 5

			//most likely a t-junction, triangulate
	
			//find the t-junction
			float tJunctionCosAngle = 1;
			int tJunction = -1;
			for (uint32_t k = 0; k<verts.size(); ++k) 
			{
				Vector3f& a = vertices[newVertices[verts[(k + verts.size() - 1) % verts.size()].first]].position;
				Vector3f& b = vertices[newVertices[verts[k].first]].position;
				Vector3f& c = vertices[newVertices[verts[(k + 1) % verts.size()].first]].position;

				auto ab = a - b;
				auto cb = c - b;
				float angle = ab.dot(cb) / (ab.norm() * cb.norm());

				if (angle < tJunctionCosAngle)
				{
					tJunction = k;
					tJunctionCosAngle = angle;
				}
			}
			
			std::vector<uint32_t> indices(3);
			indices[0] = newVertices[verts[tJunction].first];
			for (int i = 0; i < 3; ++i)
			{
				auto idx1 = verts[(tJunction + 1 + i) % verts.size()].first;
				auto idx2 = verts[(tJunction + 2 + i) % verts.size()].first;
				indices[1] = newVertices[idx1];
				indices[2] = newVertices[idx2];
				bool checkForDuplicate = 
					vertices[indices[0]].generation != currentGeneration
					&& vertices[indices[1]].generation != currentGeneration
					&& vertices[indices[2]].generation != currentGeneration;
				add_primitive_face(indices, checkForDuplicate, newEdges, newTris, newQuads, nonmanifoldEdges);
			}

			break;
		}
	}

	//add dependencies from vertices to inner edges
	if (verts.size() > (size_t)meshSettings.posy->posy())
	{
		for (int i = newEdgesBefore; i < newEdges.size(); ++i)
		{
			auto& e = edges[newEdges[i]];
			bool isInnerEdge = true;
			//check if it is an inner edge
			for (int j = 0; j < verts.size(); ++j)
			{
				auto v = newVertices[verts[j].first];
				if (v == e.v[0] || v == e.v[1])
				{
					auto next = verts[(j + 1) % verts.size()].first;
					if (newVertices[next] == e.v[0] || newVertices[next] == e.v[1])
						isInnerEdge = false;
					break;
				}
			}
			if (!isInnerEdge)
				continue;

			for (int j = 0; j < verts.size(); ++j)
			{
				auto v = newVertices[verts[j].first];
				if (v == e.v[0])
					continue;
				if (v == e.v[1])
					continue;
				vertexToDependentEdges[v].push_back(newEdges[i]);
			}
		}
	}
}

void ExtractedMesh::saveCoarseToPLY(const std::string& path)
{
	nse::util::TimedBlock b("Exporting coarse mesh to PLY");
	std::ofstream ply(path, std::ios::binary);

	ply << "ply" << std::endl;
	ply << "format binary_little_endian 1.0" << std::endl;
	ply << "element vertex " << vertices.sizeWithGaps() << std::endl;
	ply << "property float32 x" << std::endl;
	ply << "property float32 y" << std::endl;
	ply << "property float32 z" << std::endl;
	ply << "property float32 nx" << std::endl;
	ply << "property float32 ny" << std::endl;
	ply << "property float32 nz" << std::endl;
	ply << "element face " << (quads.sizeNotDeleted() + triangles.sizeNotDeleted()) << std::endl;
	ply << "property list uint8 int32 vertex_index" << std::endl;
	ply << "end_header" << std::endl;	

	for (int i = 0; i < vertices.sizeWithGaps(); ++i)
	{
		auto& v = vertices[i];
		float data[6] = { v.position.x(), v.position.y(), v.position.z(), v.normal.x(), v.normal.y(), v.normal.z() };
		ply.write(reinterpret_cast<const char*>(data), 6 * sizeof(float));
	}
	for (auto& q : quads)
	{
		uint8_t count = 4;
		int32_t data[4] = { (int32_t)startVertex(q.edges[0]) , (int32_t)startVertex(q.edges[1]) , (int32_t)startVertex(q.edges[2]) , (int32_t)startVertex(q.edges[3]) };
		ply.write(reinterpret_cast<const char*>(&count), 1);
		ply.write(reinterpret_cast<const char*>(data), 4 * 4);
	}
	for (auto& tri : triangles)
	{
		uint8_t count = 3;
		int32_t data[3] = { (int32_t)startVertex(tri.edges[0]) , (int32_t)startVertex(tri.edges[1]) , (int32_t)startVertex(tri.edges[2]) };
		ply.write(reinterpret_cast<const char*>(&count), 1);
		ply.write(reinterpret_cast<const char*>(data), 3 * 4);	
	}
	ply.close();	
}

void ExtractedMesh::saveWireframeToPLY(const std::string& path)
{
	std::vector<Vector3f> positions;

	struct Face
	{
		int32_t count, v1, v2, v3;
	};

	std::vector<Face> faces;

	//this format is meant to be read by MeshLab. We are tricking it by creating duplicate vertices and degenerate faces

	for (auto& e : edges)
	{
		int baseIndex = positions.size();
		auto& v0 = vertices[e.v[0]].position;
		auto& v1 = vertices[e.v[1]].position;
		auto& n0 = vertices[e.v[0]].normal;
		auto& n1 = vertices[e.v[1]].normal;
		for (int i = 0; i <= R; ++i)
		{
			float t = (float)i / R;
			Vector4f cd;
			if (i == 0)
				cd = vertices[e.v[0]].colorDisplacement;
			else if (i == R)
				cd = vertices[e.v[1]].colorDisplacement;
			else
				cd = e.colorDisplacement[i - 1];
			Vector3f p = (1 - t) * v0 + t * v1 + cd.w() * ((1 - t) * n0 + t * n1);
			positions.push_back(p);
			positions.push_back(p);

			if (i < R)
			{
				Face face = { 3, baseIndex + 2 * i, baseIndex + 2 * i + 2, baseIndex + 2 * i + 3 };
				faces.push_back(face);
			}
		}
	}

	nse::util::TimedBlock b("Exporting coarse mesh to PLY");
	std::ofstream ply(path, std::ios::binary);

	ply << "ply" << std::endl;
	ply << "format binary_little_endian 1.0" << std::endl;
	ply << "element vertex " << positions.size() << std::endl;
	ply << "property float32 x" << std::endl;
	ply << "property float32 y" << std::endl;
	ply << "property float32 z" << std::endl;	
	ply << "element face " << faces.size() << std::endl;
	ply << "property list int32 int32 vertex_index" << std::endl;
	ply << "end_header" << std::endl;

	ply.write(reinterpret_cast<const char*>(positions.data()), positions.size() * sizeof(Vector3f));
	ply.write(reinterpret_cast<const char*>(faces.data()), faces.size() * sizeof(Face));

	ply.close();
}

Eigen::Vector3f colorDisplacementToRGBColor(const Vector4f& color)
{
	Vector3us Lab(color.x(), color.y(), color.z());
	auto rgb = LabToRGB(Lab);

	Eigen::Vector3f r;
	for (int i = 0; i < 3; ++i)
		r(i) = std::min(1.0f, std::max(0.0f, (float)rgb(i) / 65535.0f));
	return r;
}

void ExtractedMesh::extractFineMesh(osr::MeshVisitor& visitor, bool triangulate)
{
	//set up indices in texel vector
	uint32_t nextIndex = 0;
	int faces = 0;
	for (auto& v : vertices)
		v.indexInTexelVector = nextIndex++;
	for (auto& e : edges)
	{
		e.indexInTexelVector = nextIndex;
		nextIndex += texelsPerEdge;
	}
	for (auto& t : triangles)
	{
		t.indexInTexelVector = nextIndex;
		nextIndex += (R - 1) * (R - 2) / 2;
		faces += R * R;
	}
	for (auto& q : quads)
	{
		q.indexInTexelVector = nextIndex;
		nextIndex += texelsPerQuad;
		if (triangulate)
			faces += 2 * R * R;
		else
			faces += R * R;
	}
	int totalTexels = nextIndex;

	visitor.begin(totalTexels, faces);

	//vertex data
	for (auto& v : vertices)
	{
		Vector3f p = v.position + v.colorDisplacement.w() * v.normal;
		visitor.addVertex(p, colorDisplacementToRGBColor(v.colorDisplacement));
	}
	for (auto& e : edges)
	{
		auto& v0 = vertices[e.v[0]];
		auto& v1 = vertices[e.v[1]];
		for (int i = 1; i < R; ++i)
		{
			float t = (float)i / R;
			auto& cd = e.colorDisplacement[i - 1];
			Vector3f n = (1 - t) * v0.normal + t * v1.normal;
			Vector3f p = (1 - t) * v0.position + t * v1.position + cd.w() * n;
			visitor.addVertex(p, colorDisplacementToRGBColor(cd));
		}
	}
	for (auto& tri : triangles)
	{
		auto& v0 = vertices[startVertex(tri.edges[0])];
		auto& v1 = vertices[startVertex(tri.edges[1])];
		auto& v2 = vertices[startVertex(tri.edges[2])];
		for (int v = 1; v < R - 1; ++v)
			for (int u = 1; u + v < R; ++u)
			{
				FaceInterpolationInfo interpolInfo[4];
				getInterpolationInfo(tri, Vector2f((float)u / R, (float)v / R), interpolInfo);

				Vector4f cd;
				cd.setZero();
				for (int i = 0; i < 4; ++i)
					cd += interpolInfo[i].weight * interpolInfo[i].entity->texel(interpolInfo[i].localTexelIndex);

				Vector3f n = barycentric(v0.normal, v1.normal, v2.normal, Vector2f((float)u / R, (float)v / R));
				Vector3f p = barycentric(v0.position, v1.position, v2.position, Vector2f((float)u / R, (float)v / R)) + cd.w() * n;
				visitor.addVertex(p, colorDisplacementToRGBColor(cd));
			}
	}

	for (auto& q : quads)
	{
		auto& v0 = vertices[startVertex(q.edges[0])];
		auto& v1 = vertices[startVertex(q.edges[1])];
		auto& v2 = vertices[startVertex(q.edges[2])];
		auto& v3 = vertices[startVertex(q.edges[3])];
		for (int v = 1; v < R; ++v)
			for (int u = 1; u < R; ++u)
			{
				auto& cd = q.colorDisplacement[(u - 1) + (R - 1) * (v - 1)];
				Vector3f n = bilinear(v0.normal, v1.normal, v2.normal, v3.normal, Vector2f((float)u / R, (float)v / R));
				Vector3f p = bilinear(v0.position, v1.position, v2.position, v3.position, Vector2f((float)u / R, (float)v / R)) + cd.w() * n;
				visitor.addVertex(p, colorDisplacementToRGBColor(cd));
			}
	}

	//face data

	for (auto& tri : triangles)
	{
		uint8_t count = 3;
		uint32_t data[3];
		for (int v = 0; v < R; ++v)
			for (int u = 0; u + v < R; ++u)
			{
				const Entity* e;
				int i;

				getEntityTexelBarycentric(tri, Vector2i(u, v), e, i);
				data[0] = e->indexInTexelVector + i;

				getEntityTexelBarycentric(tri, Vector2i(u + 1, v), e, i);
				data[1] = e->indexInTexelVector + i;

				getEntityTexelBarycentric(tri, Vector2i(u, v + 1), e, i);
				data[2] = e->indexInTexelVector + i;

				visitor.addFace(count, data);				

				if (u < R - v - 1)
				{
					data[0] = data[2];

					getEntityTexelBarycentric(tri, Vector2i(u + 1, v + 1), e, i);
					data[2] = e->indexInTexelVector + i;

					visitor.addFace(count, data);					
				}
			}
	}

	for (auto& q : quads)
	{
		uint8_t count = triangulate ? 3 : 4;
		uint32_t data[4];

		const Entity* e[4];
		int i[4];

		for (int u = 0; u < R; ++u)
			for (int v = 0; v < R; ++v)
			{
				getEntityTexel(q, Vector2i(u, v), e[0], i[0]);
				getEntityTexel(q, Vector2i(u + 1, v), e[1], i[1]);
				getEntityTexel(q, Vector2i(u + 1, v + 1), e[2], i[2]);
				getEntityTexel(q, Vector2i(u, v + 1), e[3], i[3]);

				if (triangulate)
				{
					data[0] = e[0]->indexInTexelVector + i[0];
					data[1] = e[1]->indexInTexelVector + i[1];
					data[2] = e[2]->indexInTexelVector + i[2];

					visitor.addFace(count, data);					

					data[0] = e[0]->indexInTexelVector + i[0];
					data[1] = e[2]->indexInTexelVector + i[2];
					data[2] = e[3]->indexInTexelVector + i[3];

					visitor.addFace(count, data);
				}
				else
				{

					data[0] = e[0]->indexInTexelVector + i[0];
					data[1] = e[1]->indexInTexelVector + i[1];
					data[2] = e[2]->indexInTexelVector + i[2];
					data[3] = e[3]->indexInTexelVector + i[3];

					visitor.addFace(count, data);
				}
			}
	}
	visitor.end();
}

void ExtractedMesh::saveFineToPLY(const std::string& path, bool triangulate)
{
	nse::util::TimedBlock b("Exporting fine mesh to PLY");
	
	WritePLYMeshVisitor visitor(path);
	extractFineMesh(visitor, triangulate);
}

void ExtractedMesh::saveToFile(FILE * f) const
{
	triangles.saveToFile(f);
	quads.saveToFile(f);
	edges.saveToFile(f);
	vertices.saveToFile(f);

	size_t n = vertexPairToEdge.size();
	fwrite(&n, sizeof(size_t), 1, f);
	for (auto& entry : vertexPairToEdge)
		fwrite(&entry, sizeof(std::pair<std::pair<uint32_t, uint32_t>, int32_t>), 1, f);

	n = vertexToDependentEdges.size();
	fwrite(&n, sizeof(size_t), 1, f);
	for (auto& entry : vertexToDependentEdges)
	{
		fwrite(&entry.first, sizeof(uint32_t), 1, f);
		n = entry.second.size();
		fwrite(&n, sizeof(size_t), 1, f);
		fwrite(entry.second.data(), sizeof(uint32_t), n, f);
	}

	fwrite(&currentGeneration, sizeof(uint8_t), 1, f);
}

void ExtractedMesh::loadFromFile(FILE * f)
{
	triangles.loadFromFile(f);
	quads.loadFromFile(f);
	edges.loadFromFile(f);
	vertices.loadFromFile(f);

	size_t n;
	fread(&n, sizeof(size_t), 1, f);
	for (int i = 0; i < n; ++i)
	{
		std::pair<std::pair<uint32_t, uint32_t>, int32_t> entry;
		fread(&entry, sizeof(std::pair<std::pair<uint32_t, uint32_t>, int32_t>), 1, f);
		vertexPairToEdge[entry.first] = entry.second;
	}	

	fread(&n, sizeof(size_t), 1, f);
	for(int i = 0; i < n; ++i)
	{
		uint32_t v;
		fread(&v, sizeof(uint32_t), 1, f);
		size_t dependent;
		fread(&dependent, sizeof(size_t), 1, f);
		auto& entry = vertexToDependentEdges[v];		
		entry.resize(dependent);
		fread(entry.data(), sizeof(uint32_t), dependent, f);
	}

	fread(&currentGeneration, sizeof(uint8_t), 1, f);

	post_extract();
}

bool ExtractedMesh::findDuplicateVertex(const Vector3f& p, size_t& outVertexIndex, size_t ignore)
{
	float radiusSq = meshSettings.scale() * meshSettings.scale() * 0.25f * 0.25f;
	for (auto it = vertices.begin(); it != vertices.end(); ++it)
	{
		if (it.index() != ignore && it->incidentEdges.size() > 0 && (it->position - p).squaredNorm() <= radiusSq)
		{
			outVertexIndex = it.index();
			return true;
		}
	}
	return false;
}

template <size_t N, typename DataSource>
bool ExtractedMesh::findDuplicateFace(const std::array<int32_t, N>& edges, DataSource& data, size_t& out_index)
{
	int minEdge = 0;
	for(int i = 1; i < N; ++i)
		if (edges[i] < edges[minEdge])
			minEdge = i;
	
	for (auto it = data.begin(); it != data.end(); ++it)
	{
		int minFEdge = 0;
		for (int i = 1; i < N; ++i)
			if (it->edges[i] < it->edges[minFEdge])
				minFEdge = i;

		bool isDup = true;
		for (int i = 0; i < N; ++i)
		{
			if (edges[(minEdge + i) % N] != it->edges[(minFEdge + i) % N])
			{
				isDup = false;
				break;
			}
		}
		if (isDup)
		{
			out_index = it.index();
			return true;
		}
	}
	return false;
}

void ExtractedMesh::projectPointOnQuad(const Quad& quad, const Vector3f& p, Vector2f& uv, Vector3f& interpolP, Vector3f& interpolN)
{
	auto& v1 = vertices[startVertex(quad.edges[0])];
	auto& v2 = vertices[startVertex(quad.edges[1])];
	auto& v3 = vertices[startVertex(quad.edges[2])];
	auto& v4 = vertices[startVertex(quad.edges[3])];


	Eigen::Matrix<float, 3, 2> jacobian;
	
	uv = Vector2f::Constant(0.5f);

	//Newton iteration for projection on quad

	//objective function:
	// F(u, v) = (p - interpolP) x interpolN = 0
	Vector3f F;

	for (int i = 0; i < 4; ++i)
	{
		interpolP = bilinear(v1.position, v2.position, v3.position, v4.position, uv);
		interpolN = bilinear(v1.normal, v2.normal, v3.normal, v4.normal, uv);
			
		Vector3f dPdu = (1 - uv.y()) * v2.position + uv.y() * v3.position - ((1 - uv.y()) * v1.position + uv.y() * v4.position);
		Vector3f dPdv = (1 - uv.x()) * v4.position + uv.x() * v3.position - ((1 - uv.x()) * v1.position + uv.x() * v2.position);
		Vector3f dNdu = (1 - uv.y()) * v2.normal + uv.y() * v3.normal - ((1 - uv.y()) * v1.normal + uv.y() * v4.normal);
		Vector3f dNdv = (1 - uv.x()) * v4.normal + uv.x() * v3.normal - ((1 - uv.x()) * v1.normal + uv.x() * v2.normal);

		F = (p - interpolP).cross(interpolN);
		Vector3f dFdu = (-dPdu).cross(interpolN) + (p - interpolP).cross(dNdu);
		Vector3f dFdv = (-dPdv).cross(interpolN) + (p - interpolP).cross(dNdv);

		jacobian.col(0) = dFdu;
		jacobian.col(1) = dFdv;

		//std::cout << uv.transpose() << " => " << F.transpose() << std::endl;

		Vector2f rhs = -jacobian.transpose() * F;
		auto lhs = jacobian.transpose() * jacobian;
		float norm = 1.0f / (lhs(0, 0) * lhs(1, 1) - lhs(0, 1) * lhs(1, 0));

		uv += Vector2f(lhs(1, 1) * rhs.x() - lhs(0, 1) * rhs.y(), -lhs(1, 0) * rhs.x() + lhs(0, 0) * rhs.y()) * norm;
	}		

	interpolP = bilinear(v1.position, v2.position, v3.position, v4.position, uv);
	interpolN = bilinear(v1.normal, v2.normal, v3.normal, v4.normal, uv);
}

void ExtractedMesh::projectPointOnTriangle(const Triangle& tri, const Vector3f& p, Vector2f& uv, Vector3f& interpolP, Vector3f& interpolN)
{
	auto& v1 = vertices[startVertex(tri.edges[0])];
	auto& v2 = vertices[startVertex(tri.edges[1])];
	auto& v3 = vertices[startVertex(tri.edges[2])];

	Eigen::Matrix<float, 3, 2> jacobian;

	uv = Vector2f::Constant(0.333f);

	//Newton iteration for projection on triangle

	//objective function:
	// F(u, v) = (p - interpolP) x interpolN = 0
	Vector3f F;

	Vector3f dPdu = v1.position - v3.position;
	Vector3f dPdv = v2.position - v3.position;
	Vector3f dNdu = v1.normal - v3.normal;
	Vector3f dNdv = v2.normal - v3.normal;

	for (int i = 0; i < 4; ++i)
	{
		interpolP = barycentric(v1.position, v2.position, v3.position, uv);
		interpolN = barycentric(v1.normal, v2.normal, v3.normal, uv);

		F = (p - interpolP).cross(interpolN);
		Vector3f dFdu = (-dPdu).cross(interpolN) + (p - interpolP).cross(dNdu);
		Vector3f dFdv = (-dPdv).cross(interpolN) + (p - interpolP).cross(dNdv);

		jacobian.col(0) = dFdu;
		jacobian.col(1) = dFdv;

		//std::cout << uv.transpose() << " => " << F.transpose() << std::endl;

		Vector2f rhs = -jacobian.transpose() * F;
		auto lhs = jacobian.transpose() * jacobian;
		float norm = 1.0f / (lhs(0, 0) * lhs(1, 1) - lhs(0, 1) * lhs(1, 0));

		uv += Vector2f(lhs(1, 1) * rhs.x() - lhs(0, 1) * rhs.y(), -lhs(1, 0) * rhs.x() + lhs(0, 0) * rhs.y()) * norm;
	}

	interpolP = barycentric(v1.position, v2.position, v3.position, uv);
	interpolN = barycentric(v1.normal, v2.normal, v3.normal, uv);
}

void ExtractedMesh::getEntityTexel(const Vertex& v, const Entity*& entity, int& localIndex)
{
	entity = &v;
	localIndex = 0;
}

void ExtractedMesh::getEntityTexel(const Edge& e, int discreteU, const Entity*& entity, int& localIndex)
{
	if (discreteU < 0 || discreteU > R)
		std::cout << "Invalid edge U " << discreteU << std::endl;
	
	if (discreteU == 0)
		getEntityTexel(vertices[e.v[0]], entity, localIndex);
	else if (discreteU == R)
		getEntityTexel(vertices[e.v[1]], entity, localIndex);
	else
	{
		entity = &e;
		localIndex = discreteU - 1;
	}
}

void ExtractedMesh::getEntityTexel(const Triangle& tri, const Vector3i& rev, const Entity*& entity, int& localIndex)
{
	if (rev.x() == 0)
		getEntityTexel(edges[edgeIndex(tri.edges[rev.y()])], (tri.edges[rev.y()] < 0 ? R - rev.z() : rev.z()), entity, localIndex);
	else
	{
		localIndex = revToIndex(rev);
		entity = &tri;
	}
}

void ExtractedMesh::getEntityTexelBarycentric(const Triangle& tri, const Vector2i& uv, const Entity*& entity, int& localIndex)
{
	if(uv.x() == 0)
		getEntityTexel(edges[edgeIndex(tri.edges[1])], (tri.edges[1] < 0 ? uv.y() : R - uv.y()), entity, localIndex);
	else if(uv.y() == 0)
		getEntityTexel(edges[edgeIndex(tri.edges[2])], (tri.edges[2] < 0 ? R - uv.x() : uv.x()), entity, localIndex);
	else if(R - uv.x() - uv.y() == 0)
		getEntityTexel(edges[edgeIndex(tri.edges[0])], (tri.edges[0] < 0 ? R - uv.y() : uv.y()), entity, localIndex);
	else
	{
		entity = &tri;
		localIndex = (uv.y() - 1) * (2 * R - uv.y() - 2) / 2 + uv.x() - 1;
	}
}

void ExtractedMesh::getEntityTexel(const Quad& quad, const Vector2i& coord, const Entity*& entity, int& localIndex)
{
	if (coord.y() == 0)
		getEntityTexel(edges[edgeIndex(quad.edges[0])], (quad.edges[0] < 0 ? R - coord.x() : coord.x()), entity, localIndex);
	else if (coord.x() == R)
		getEntityTexel(edges[edgeIndex(quad.edges[1])], (quad.edges[1] < 0 ? R - coord.y() : coord.y()), entity, localIndex);
	else if (coord.y() == R)
		getEntityTexel(edges[edgeIndex(quad.edges[2])], (quad.edges[2] >= 0 ? R - coord.x() : coord.x()), entity, localIndex);
	else if (coord.x() == 0)
		getEntityTexel(edges[edgeIndex(quad.edges[3])], (quad.edges[3] >= 0 ? R - coord.y() : coord.y()), entity, localIndex);
	else
	{
		entity = &quad;
		localIndex = coord.x() - 1 + (coord.y() - 1) * (R - 1);
	}
}

void ExtractedMesh::getInterpolationInfo(const Triangle& tri, const Vector2f& uv, ExtractionHelper::FaceInterpolationInfo* interpolationInfo)
{
	auto rev = barycentricToREV(uv);

	Vector3i lowerREV((int)std::floor(rev.x()), (int)rev.y(), (int)std::floor(rev.z()));
	Vector2f localUV(rev.x() - lowerREV.x(), rev.z() - lowerREV.z());
	
	Vector3i targetREV = lowerREV;

	getEntityTexel(tri, targetREV, interpolationInfo[0].entity, interpolationInfo[0].localTexelIndex);
	interpolationInfo[0].weight = (1 - localUV.x()) * (1 - localUV.y());

	targetREV = lowerREV + Vector3i(0, 0, 1);
	targetREV.z() = std::min(targetREV.z(), R - 2 * targetREV.x());
	if (targetREV.z() == R - 2 * targetREV.x())
	{
		targetREV.y() = (targetREV.y() + 1) % 3;
		targetREV.z() = 0;
	}
	getEntityTexel(tri, targetREV, interpolationInfo[1].entity, interpolationInfo[1].localTexelIndex);
	interpolationInfo[1].weight = localUV.x() * (1 - localUV.y());

	targetREV = lowerREV + Vector3i(1, 0, -1);
	targetREV.z() = std::max(targetREV.z(), 0);
	if (targetREV.z() == R - 2 * targetREV.x())
	{
		targetREV.y() = (targetREV.y() + 1) % 3;
		targetREV.z() = 0;
	}
	getEntityTexel(tri, targetREV, interpolationInfo[2].entity, interpolationInfo[2].localTexelIndex);
	interpolationInfo[2].weight = (1 - localUV.x()) * localUV.y();

	targetREV = lowerREV + Vector3i(1, 0, 0);
	targetREV.z() = std::min(targetREV.z(), R - 2 * targetREV.x());
	if (targetREV.z() == R - 2 * targetREV.x())
	{
		targetREV.y() = (targetREV.y() + 1) % 3;
		targetREV.z() = 0;
	}
	getEntityTexel(tri, targetREV, interpolationInfo[3].entity, interpolationInfo[3].localTexelIndex);
	interpolationInfo[3].weight = localUV.x() * localUV.y();
}

void ExtractedMesh::getInterpolationInfo(const Quad& quad, const Vector2f& uv, ExtractionHelper::FaceInterpolationInfo* interpolationInfo)
{
	Vector2i discreteLowerUV((int)std::floor(uv.x() * R), (int)std::floor(uv.y() * R));
	Vector2f localUV = uv * R - discreteLowerUV.cast<float>();

	getEntityTexel(quad, discreteLowerUV, interpolationInfo[0].entity, interpolationInfo[0].localTexelIndex);
	interpolationInfo[0].weight = (1 - localUV.x()) * (1 - localUV.y());

	getEntityTexel(quad, discreteLowerUV + Vector2i(1, 0), interpolationInfo[1].entity, interpolationInfo[1].localTexelIndex);
	interpolationInfo[1].weight = localUV.x() * (1 - localUV.y());

	getEntityTexel(quad, discreteLowerUV + Vector2i(0, 1), interpolationInfo[2].entity, interpolationInfo[2].localTexelIndex);
	interpolationInfo[2].weight = (1 - localUV.x()) * localUV.y();

	getEntityTexel(quad, discreteLowerUV + Vector2i(1, 1), interpolationInfo[3].entity, interpolationInfo[3].localTexelIndex);
	interpolationInfo[3].weight = localUV.x() * localUV.y();
}

int ExtractedMesh::revToIndex(const Vector3i& rev) const
{
	return 3 * (rev.x() - 1) * (R - rev.x()) //all texels from larger rings
		+ rev.y() * (R - 2 * rev.x()) //all texels from previous edges of the same ring
		+ rev.z();
}

Vector3i ExtractedMesh::indexToREV(int index) const
{
	Vector3i rev;
	rev.x() = (int)((3 + 3 * R - sqrt(3 * (3 - 4 * index - 6 * R + 3 * R * R))) / 6);
	index -= 3 * (rev.x() - 1) * (R - rev.x());
	if (R - 2 * rev.x() == 0)
		rev.y() = 0;
	else
		rev.y() = index / (R - 2 * rev.x());
	index -= rev.y() * (R - 2 * rev.x());
	rev.z() = index;
	return rev;
}

Eigen::Vector3f ExtractedMesh::barycentricToREV(const Vector2f& barycentric) const
{
	Vector3f rev; //ring, edge, vector

	float minBary;
	float bz = 1 - barycentric.x() - barycentric.y();
	float projBary;
	if (barycentric.x() < barycentric.y() && barycentric.x() < bz)
	{
		minBary = barycentric.x();
		rev.y() = 1;
		projBary = barycentric.y();
	}
	else if (barycentric.y() < bz)
	{
		minBary = barycentric.y();
		rev.y() = 2;
		projBary = bz;
	}
	else
	{
		minBary = bz;
		rev.y() = 0;
		projBary = barycentric.x();
	}
	rev.x() = 3.0f / 2.0f * R * minBary;
	unsigned int baseRing = std::floor(rev.x());
	unsigned int verticesOnRingEdge = R - 2 * baseRing;

	float denom = 3 * minBary - 1;
	rev.z() = (denom == 0
		? 0
		: (projBary + 2 * minBary - 1) * (R - 2 * baseRing) / denom
		);
	if (rev.z() >= verticesOnRingEdge)
	{
		rev.z() -= verticesOnRingEdge;
		rev.y() = rev.y() + 1;
		if (rev.y() >= 3)
			rev.y() -= 3;
	}

	return rev;
}

Eigen::Vector2f ExtractedMesh::revToBarycentric(const Vector3i& rev) const
{
	Vector3f base = Vector3f::Zero(), nextBase = Vector3f::Zero(), edgeDir = Vector3f::Zero();
	base(rev.y()) = 1;
	nextBase((rev.y() + 1) % 3) = 1;
	edgeDir((rev.y() + 1) % 3) = 1; edgeDir -= base;	
	Vector3f center(1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f);
	base += 2.0f * rev.x() / R * (center - base);
	nextBase += 2.0f * rev.x() / R * (center - nextBase);

	int texelsOnRingEdge = R - 2 * rev.x();
	Vector3f bary;
	if (texelsOnRingEdge > 0)
	{
		float t = (float)rev.z() / texelsOnRingEdge;
		bary = (1 - t) * base + t * nextBase;
	}
	else
		bary = base;

	return Vector2f(bary.x(), bary.y());
}

void ExtractedMesh::getInterpolatedPositionNormal(const ExtractionHelper::Vertex& v, int localIndex, Vector3f& out_position, Vector3f& out_normal) const
{
	out_position = v.position;
	out_normal = v.normal;
}

void ExtractedMesh::getInterpolatedPositionNormal(const ExtractionHelper::Edge& e, int localIndex, Vector3f& out_position, Vector3f& out_normal) const
{
	float t = (float)(1 + localIndex) / R;
	auto& v1 = vertices[e.v[0]];
	auto& v2 = vertices[e.v[1]];
	out_position = (1 - t) * v1.position + t * v2.position;
	out_normal = (1 - t) * v1.normal + t * v2.normal;
}

void ExtractedMesh::getInterpolatedPositionNormal(const ExtractionHelper::Triangle& t, int localIndex, Vector3f& out_position, Vector3f& out_normal) const
{
	Vertex v[] = { vertices[startVertex(t.edges[0])],
		vertices[startVertex(t.edges[1])],
		vertices[startVertex(t.edges[2])] };

	auto rev = indexToREV(localIndex);
	auto bary = revToBarycentric(rev);
	out_position = barycentric(v[0].position, v[1].position, v[2].position, bary);
	out_normal = barycentric(v[0].normal, v[1].normal, v[2].normal, bary);
}

void ExtractedMesh::getInterpolatedPositionNormal(const ExtractionHelper::Quad& q, int localIndex, Vector3f& out_position, Vector3f& out_normal) const
{
	Vertex v[] = { vertices[startVertex(q.edges[0])],
					vertices[startVertex(q.edges[1])],
					vertices[startVertex(q.edges[2])],
					vertices[startVertex(q.edges[3])] };

	Vector2f uv((float)(localIndex % (R - 1) + 1) / R, (float)(localIndex / (R - 1) + 1) / R);
	out_position = bilinear(v[0].position, v[1].position, v[2].position, v[3].position, uv);
	out_normal = bilinear(v[0].normal, v[1].normal, v[2].normal, v[3].normal, uv);
}

Vector4f osr::repairSolution(const Vector4f& sol)
{
	Vector4f r = sol;

	for (int i = 0; i < 3; ++i)
		r(i) = (unsigned short)std::min(65535.0f, std::max(0.0f, sol(i)));

	return r;
}

void ExtractedMesh::calculateCollapsedGraphVisualization(const std::vector<size_t>& newVertices, const std::vector<bool>& partOfCore, const std::vector<bool>& vertexReachableFromCore, std::vector<std::vector<TaggedLink>>& adj)
{
	nse::util::TimedBlock b("Calculating visualization for extraction graph ..");
	std::vector<Vector3f> visPos, visCol;

	auto colorFun = [&](uint32_t i)
	{
		Vector3f c(0, 0, 0);
		if (partOfCore[i])
			c.x() = 1;
		if (vertexReachableFromCore[i])
			c.y() = 1;
		if (vertices[newVertices[i]].generation != currentGeneration)
			c.z() = 1;
		return c;
	};

	for (int i = 0; i < newVertices.size(); ++i)
	{
		auto& p1 = vertices[newVertices[i]].position;
		auto c1 = colorFun(i);
		for (int j = 0; j < adj[i].size(); ++j)
		{
			auto v2 = adj[i][j].id;
			auto& p2 = vertices[newVertices[v2]].position;
			auto c2 = colorFun(v2);

			visPos.push_back(p1);
			visPos.push_back(p2);
			visCol.push_back(c1);
			visCol.push_back(c2);
		}
	}
	collapsedGraphData(std::move(visPos), std::move(visCol));
}

void osr::checkSymmetry(std::vector<std::vector<TaggedLink>>& adj)
{
	for (uint32_t i = 0; i < adj.size(); ++i)
	{
		for (uint32_t j = 0; j < adj[i].size(); ++j)
		{
			uint32_t j_id = adj[i][j].id;
			bool found = false;
			for (uint32_t k = 0; k<adj[j_id].size(); ++k)
			{
				if (adj[j_id][k].id == i)
				{
					found = true;
					break;
				}
			}
			if (!found)
				std::cout << "Adjacency is not symmetric! (" << i << ", " << j_id << ")" << std::endl;
		}
	}
}

template<>
void nse::data::saveToFile(const ExtractionHelper::Triangle & object, FILE * f)
{
	saveToFile(object.generation, f);
	saveToFile(object.edges, f);
	saveToFile(object.colorDisplacement, f);
}

template<>
void nse::data::loadFromFile(ExtractionHelper::Triangle & object, FILE * f)
{
	loadFromFile(object.generation, f);
	loadFromFile(object.edges, f);
	loadFromFile(object.colorDisplacement, f);
}

template<>
void nse::data::saveToFile(const ExtractionHelper::Quad & object, FILE * f)
{
	saveToFile(object.generation, f);
	saveToFile(object.edges, f);
	saveToFile(object.colorDisplacement, f);
}

template<>
void nse::data::loadFromFile(ExtractionHelper::Quad & object, FILE * f)
{
	loadFromFile(object.generation, f);
	loadFromFile(object.edges, f);
	loadFromFile(object.colorDisplacement, f);
}

template<>
void nse::data::saveToFile(const ExtractionHelper::Edge & object, FILE * f)
{
	saveToFile(object.generation, f);
	saveToFile(object.v, f);
	saveToFile(object.colorDisplacement, f);
	saveToFile(object.incidentTriangles, f);
	saveToFile(object.incidentQuads, f);
}

template<>
void nse::data::loadFromFile(ExtractionHelper::Edge & object, FILE * f)
{
	loadFromFile(object.generation, f);
	loadFromFile(object.v, f);
	loadFromFile(object.colorDisplacement, f);
	loadFromFile(object.incidentTriangles, f);
	loadFromFile(object.incidentQuads, f);
}

template<>
void nse::data::saveToFile(const ExtractionHelper::Vertex & object, FILE * f)
{
	saveToFile(object.generation, f);
	saveToFile(object.position, f);
	saveToFile(object.normal, f);
	saveToFile(object.colorDisplacement, f);
	saveToFile(object.incidentEdges, f);
}

template<>
void nse::data::loadFromFile(ExtractionHelper::Vertex & object, FILE * f)
{
	loadFromFile(object.generation, f);
	loadFromFile(object.position, f);
	loadFromFile(object.normal, f);
	loadFromFile(object.colorDisplacement, f);
	loadFromFile(object.incidentEdges, f);
}
