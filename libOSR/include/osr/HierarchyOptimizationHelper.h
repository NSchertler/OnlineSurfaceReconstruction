/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include <vector>
#include <iterator>
#include <algorithm>
#include <unordered_map>
#include <mutex>

#include "osr/common.h"
#include "osr/Attributes.h"
#include <nsessentials/util/IndentationLog.h>

#include "osr/PreparedVertexSet.h"

// This file contains structures and functions to prepare a set of vertices for optimization. Depending
// on the Hierarchy and the Index type, several options for the preparation can be chosen by specializing
// IndexOptimizationTraits<>. The default options store neighbors explicitly in the resulting set and use
// a graph coloring-based parallelization strategy.

namespace osr
{
	template <typename Index, typename Hierarchy>
	struct IndexOptimizationTraits
	{
		//Specifies if the phase of a vertex can be calculated efficiently from its index.
		//In this case, the hierarchy must provide a size_t phase(Index) function.
		static const bool HasImplicitPhases = false;

		//Specifies if neighbors should be stored explicitly in the prepared vertex set.
		//If not, the hierarchy is queried again every time the neighbors are needed.
		static const bool StoreNeighbors = true;
	};

	const int MAX_COLORS = 32;
	struct ColorData
	{
		uint8_t nColors;
		uint32_t nNodes[MAX_COLORS];
		uint32_t nNeighbors[MAX_COLORS];
		ColorData() : nColors(0) { }
	};

	//neighbors per vertex must be sorted by index
	extern ColorData generateGraphColoring(int vertexCount, const std::vector<std::vector<size_t>>& neighbors, std::vector<uint8_t>& color);

	// Provides functionality to prepare a set of vertices for optimization (e.g. calculation of neighbors and phase separation).
	// This is the base template. It has no implementations. Implementations will be provided for the partial specializations below.
	template <typename Iterator, typename Hierarchy,
		bool HasImplicitPhases = IndexOptimizationTraits<typename Iterator::value_type, Hierarchy>::HasImplicitPhases,
		bool StoreNeighbors = IndexOptimizationTraits<typename Iterator::value_type, Hierarchy>::StoreNeighbors,
		bool UseOriginalIndexForNeighbors = true>
		struct HierarchyOptimizationHelper
	{
		static PreparedVertexSet<Hierarchy, typename Iterator::value_type, StoreNeighbors> prepare(Iterator vertices_begin, Iterator vertices_end, Hierarchy& hierarchy);
	};

	//No implicit phases
	//Store neighbors explicitly
	template <typename Iterator, typename Hierarchy>
	struct HierarchyOptimizationHelper<Iterator, Hierarchy, false, true, true>
	{
		typedef typename Iterator::value_type Index;

		static PreparedVertexSet<Hierarchy, Index, true> prepare(Iterator vertices_begin, Iterator vertices_end, Hierarchy& hierarchy)
		{
			std::vector<Index> vertices;
			std::unordered_map<Index, size_t> indexToVectorPosition;
			for (auto it = vertices_begin; it != vertices_end; ++it)
			{
				vertices.push_back(*it);
				indexToVectorPosition[*it] = vertices.size() - 1;
			}
			size_t verticesIncluded = vertices.size();
			if (verticesIncluded == 0)
			{
				PreparedVertexSet<Hierarchy, Index, true> result(&hierarchy);
				result.includedVertices = 0;
				return result;
			}
			std::vector<std::vector<size_t>> neighbors(verticesIncluded); //reference to vertices
			std::vector<std::vector<Index>> additionalNeighbors(verticesIncluded); //neighbors that are not in vertices

			{
				nse::util::TimedBlock neighborBlock("Calculating neighbors ..");
#pragma omp parallel for
				for (int i = 0; i < verticesIncluded; ++i)
				{
					hierarchy.forEachNeighbor(vertices[i], [&](const auto& n)
					{
						auto it = indexToVectorPosition.find(n);
						if (it == indexToVectorPosition.end())
							additionalNeighbors[i].push_back(n);
						else
							neighbors[i].push_back(it->second);
					});
					std::sort(neighbors[i].begin(), neighbors[i].end()); //avoid deadlock in graph coloring
				}
			}

			std::vector<uint8_t> color;
			auto colorData = generateGraphColoring(verticesIncluded, neighbors, color);

			//Write data to vertex set
			PreparedVertexSet<Hierarchy, Index> vertexSet(&hierarchy);

			struct PhaseInfo
			{
				PhaseInfo() : vertexOffset(0), neighborsOffset(0) { }
				size_t vertexOffset;
				size_t neighborsOffset;
			};
			for (int i = 0; i < verticesIncluded; ++i)
				colorData.nNeighbors[color[i]] += additionalNeighbors[i].size();
			std::vector<PhaseInfo> phaseInfo(colorData.nColors);
			for (int i = 1; i < colorData.nColors; ++i)
			{
				phaseInfo[i].vertexOffset = phaseInfo[i - 1].vertexOffset + colorData.nNodes[i - 1];
				phaseInfo[i].neighborsOffset = phaseInfo[i - 1].neighborsOffset + colorData.nNeighbors[i - 1];
			}

			vertexSet.vertices.resize(verticesIncluded);
			vertexSet.neighbors.resize(phaseInfo[colorData.nColors - 1].neighborsOffset + colorData.nNeighbors[colorData.nColors - 1]);
			vertexSet.phaseStarts.resize(colorData.nColors);
#pragma omp parallel for
			for (int phase = 0; phase < colorData.nColors; ++phase)
			{
				size_t nextVertex = phaseInfo[phase].vertexOffset;
				size_t nextNeighbor = phaseInfo[phase].neighborsOffset;
				vertexSet.phaseStarts[phase] = nextVertex;
				for (int i = 0; i < verticesIncluded; ++i)
				{
					if (color[i] == phase)
					{
						vertexSet.vertices[nextVertex++] = { vertices[i], nextNeighbor };
						for (auto& n : neighbors[i])
							vertexSet.neighbors[nextNeighbor++] = vertices[n];
						for (auto& n : additionalNeighbors[i])
							vertexSet.neighbors[nextNeighbor++] = n;
					}
				}
			}

			vertexSet.includedVertices = verticesIncluded;
			return vertexSet;
		}
	};

	//No implicit phases
	//Store neighbors explicitly with intrinsic indices
	template <typename Iterator, typename Hierarchy>
	struct HierarchyOptimizationHelper<Iterator, Hierarchy, false, true, false>
	{
		typedef typename Iterator::value_type Index;

		static PreparedVertexSet<Hierarchy, Index, true, false> prepare(Iterator vertices_begin, Iterator vertices_end, Hierarchy& hierarchy)
		{
			PreparedVertexSet<Hierarchy, Index, true, false> vertexSet(&hierarchy);
			std::vector<Index> vertices;
			for (auto it = vertices_begin; it != vertices_end; ++it)
			{
				vertices.push_back(*it);
				vertexSet.indexToVectorPosition[*it] = vertices.size() - 1;
			}
			size_t verticesIncluded = vertices.size();
			if (verticesIncluded == 0)
			{
				PreparedVertexSet<Hierarchy, Index, true, false> result(&hierarchy);
				result.includedVertices = 0;
				return result;
			}
			std::vector<std::vector<size_t>> neighbors(verticesIncluded); //reference to vertices

			{
				nse::util::TimedBlock neighborBlock("Calculating neighbors ..");
				std::vector<std::vector<Index>> additionalNeighbors(verticesIncluded);
#pragma omp parallel for
				for (int i = 0; i < verticesIncluded; ++i)
				{
					hierarchy.forEachNeighbor(vertices[i], [&](const auto& n)
					{
						auto it = vertexSet.indexToVectorPosition.find(n);
						if (it == vertexSet.indexToVectorPosition.end())
							additionalNeighbors[i].push_back(n);
						else
							neighbors[i].push_back(it->second);
					});
					std::sort(neighbors[i].begin(), neighbors[i].end()); //avoid deadlock in graph coloring
				}

				for (int i = 0; i < verticesIncluded; ++i)
				{
					for (auto& n : additionalNeighbors[i])
					{
						auto it = vertexSet.indexToVectorPosition.find(n);
						if (it == vertexSet.indexToVectorPosition.end())
						{
							vertices.push_back(n);
							neighbors[i].push_back(vertices.size() - 1);
							vertexSet.indexToVectorPosition[n] = vertices.size() - 1;
						}
						else
							neighbors[i].push_back(it->second);
					}
				}
			}

			std::vector<uint8_t> color;
			auto colorData = generateGraphColoring(verticesIncluded, neighbors, color);

			//Write data to vertex set		

			struct PhaseInfo
			{
				PhaseInfo() : vertexOffset(0), neighborsOffset(0) { }
				size_t vertexOffset;
				size_t neighborsOffset;
			};

			std::vector<PhaseInfo> phaseInfo(colorData.nColors);
			for (int i = 1; i < colorData.nColors; ++i)
			{
				phaseInfo[i].vertexOffset = phaseInfo[i - 1].vertexOffset + colorData.nNodes[i - 1];
				phaseInfo[i].neighborsOffset = phaseInfo[i - 1].neighborsOffset + colorData.nNeighbors[i - 1];
			}

			vertexSet.vertices.resize(vertices.size());
			vertexSet.neighbors.resize(phaseInfo[colorData.nColors - 1].neighborsOffset + colorData.nNeighbors[colorData.nColors - 1]);
			vertexSet.phaseStarts.resize(colorData.nColors);
			std::vector<size_t> newIndices(verticesIncluded);
#pragma omp parallel for
			for (int phase = 0; phase < colorData.nColors; ++phase)
			{
				size_t nextVertex = phaseInfo[phase].vertexOffset;
				size_t nextNeighbor = phaseInfo[phase].neighborsOffset;
				vertexSet.phaseStarts[phase] = nextVertex;
				for (int i = 0; i < verticesIncluded; ++i)
				{
					if (color[i] == phase)
					{
						newIndices[i] = nextVertex;
						vertexSet.vertices[nextVertex++] = { vertices[i], nextNeighbor };
						//consider memcpy
						for (size_t n : neighbors[i])
							vertexSet.neighbors[nextNeighbor++] = n;
					}
				}
			}
#pragma omp parallel for
			for (int i = verticesIncluded; i < vertices.size(); ++i)
				vertexSet.vertices[i] = { vertices[i], vertexSet.neighbors.size() };

#pragma omp parallel for
			for (int i = 0; i < vertexSet.neighbors.size(); ++i)
			{
				if (vertexSet.neighbors[i] < verticesIncluded)
					vertexSet.neighbors[i] = newIndices[vertexSet.neighbors[i]];
			}
			for (auto& entry : vertexSet.indexToVectorPosition)
				if (entry.second < verticesIncluded)
					entry.second = newIndices[entry.second];

			vertexSet.includedVertices = verticesIncluded;
			return vertexSet;
		}
	};

	//With implicit phases
	//Store neighbors explicitly
	template <typename Iterator, typename Hierarchy>
	struct HierarchyOptimizationHelper<Iterator, Hierarchy, true, true, true>
	{
		typedef typename Iterator::value_type Index;

		static PreparedVertexSet<Hierarchy, Index, true> prepare(Iterator vertices_begin, Iterator vertices_end, Hierarchy& hierarchy)
		{
			std::vector<Index> vertices;
			for (auto it = vertices_begin; it != vertices_end; ++it)
			{
				vertices.push_back(*it);
			}
			size_t verticesIncluded = vertices.size();
			if (verticesIncluded == 0)
			{
				PreparedVertexSet<Hierarchy, Index, true> result(&hierarchy);
				result.includedVertices = 0;
				return result;
			}
			std::vector<std::vector<Index>> neighbors(verticesIncluded);

			std::vector<std::vector<size_t>> phases;
			std::vector<size_t> phaseNeighbors;
			{
				nse::util::TimedBlock neighborsBlock("Calculating neighbors ..");
				std::mutex mutex;
#pragma omp parallel for
				for (int i = 0; i < verticesIncluded; ++i)
				{
					hierarchy.forEachNeighbor(vertices[i], [&](const auto& n)
					{
						neighbors[i].push_back(n);
					});

					size_t phase = hierarchy.phase(vertices[i]);
					std::lock_guard<std::mutex> lock(mutex);
					if (phases.size() <= phase)
					{
						phases.resize(phase + 1);
						phaseNeighbors.resize(phase + 1);
					}
					phases[phase].push_back(i);
					phaseNeighbors[phase] += neighbors[i].size();
				}
			}

			//Write data to vertex set
			PreparedVertexSet<Hierarchy, Index> vertexSet(&hierarchy);

			struct PhaseInfo
			{
				PhaseInfo() : vertexOffset(0), neighborsOffset(0) { }
				size_t vertexOffset;
				size_t neighborsOffset;
			};
			std::vector<PhaseInfo> phaseInfo(phases.size());
			for (int i = 1; i < phases.size(); ++i)
			{
				phaseInfo[i].vertexOffset = phaseInfo[i - 1].vertexOffset + phases[i - 1].size();
				phaseInfo[i].neighborsOffset = phaseInfo[i - 1].neighborsOffset + phaseNeighbors[i - 1];
			}

			vertexSet.vertices.resize(vertices.size());
			vertexSet.neighbors.resize(phaseInfo[phases.size() - 1].neighborsOffset + phaseNeighbors[phases.size() - 1]);
			vertexSet.phaseStarts.resize(phases.size());
#pragma omp parallel for
			for (int phase = 0; phase < phases.size(); ++phase)
			{
				size_t nextVertex = phaseInfo[phase].vertexOffset;
				size_t nextNeighbor = phaseInfo[phase].neighborsOffset;
				vertexSet.phaseStarts[phase] = nextVertex;
				for (int i = 0; i < phases[phase].size(); ++i)
				{
					size_t v = phases[phase][i];
					vertexSet.vertices[nextVertex++] = { vertices[v], nextNeighbor };
					auto& myNeighbors = neighbors[v];
					memcpy(&vertexSet.neighbors[nextNeighbor], myNeighbors.data(), myNeighbors.size() * sizeof(Index));
					nextNeighbor += myNeighbors.size();
				}
			}

			vertexSet.includedVertices = verticesIncluded;
			return vertexSet;
		}
	};

	//With implicit phases
	//Do not store neighbors explicitly
	template <typename Iterator, typename Hierarchy>
	struct HierarchyOptimizationHelper<Iterator, Hierarchy, true, false, true>
	{
		typedef typename Iterator::value_type Index;

		static PreparedVertexSet<Hierarchy, Index, false> prepare(Iterator vertices_begin, Iterator vertices_end, Hierarchy& hierarchy)
		{
			std::vector<std::vector<Index>> phases;
			size_t verticesIncluded = 0;
			for (auto it = vertices_begin; it != vertices_end; ++it)
			{
				auto v = *it;
				size_t phase = hierarchy.phase(v);
				if (phases.size() <= phase)
					phases.resize(phase + 1);
				phases[phase].push_back(v);
				++verticesIncluded;
			}
			if (verticesIncluded == 0)
			{
				PreparedVertexSet<Hierarchy, Index, false> result(&hierarchy);
				result.includedVertices = 0;
				return result;
			}

			//Write data to vertex set
			PreparedVertexSet<Hierarchy, Index, false> vertexSet(&hierarchy);

			std::vector<size_t> vertexOffset(phases.size());
			for (int i = 1; i < phases.size(); ++i)
			{
				vertexOffset[i] = vertexOffset[i - 1] + phases[i - 1].size();
			}

			vertexSet.vertices.resize(verticesIncluded);
			vertexSet.phaseStarts.resize(phases.size());
#pragma omp parallel for
			for (int phase = 0; phase < phases.size(); ++phase)
			{
				vertexSet.phaseStarts[phase] = vertexOffset[phase];
				memcpy(&vertexSet.vertices[vertexOffset[phase]], phases[phase].data(), phases[phase].size() * sizeof(Index));
			}

			vertexSet.includedVertices = verticesIncluded;
			return vertexSet;
		}
	};

	//Convenience method to access the partial specializations above with template argument inference.
	template <typename Iterator, typename Hierarchy,
		bool HasImplicitPhases = IndexOptimizationTraits<typename Iterator::value_type, Hierarchy>::HasImplicitPhases,
		bool StoreNeighbors = IndexOptimizationTraits<typename Iterator::value_type, Hierarchy>::StoreNeighbors>
		PreparedVertexSet<Hierarchy, typename Iterator::value_type, StoreNeighbors> prepareForOptimization(Iterator vertices_begin, Iterator vertices_end, Hierarchy& hierarchy)
	{
		return HierarchyOptimizationHelper<Iterator, Hierarchy, HasImplicitPhases, StoreNeighbors>::prepare(vertices_begin, vertices_end, hierarchy);
	}

	//Convenience method to access the partial specializations above with template argument inference.
	template <typename Iterator, typename Hierarchy>
	PreparedVertexSet<Hierarchy, typename Iterator::value_type, true, false> prepareForOptimizationWithIntrinsicNeighborIndices(Iterator vertices_begin, Iterator vertices_end, Hierarchy& hierarchy)
	{
		return HierarchyOptimizationHelper<Iterator, Hierarchy, false, true, false>::prepare(vertices_begin, vertices_end, hierarchy);
	}
}