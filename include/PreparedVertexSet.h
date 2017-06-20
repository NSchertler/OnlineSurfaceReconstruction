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

#include <iostream>
#include <unordered_map>
#include <string>
#include "common.h"
#include "Attributes.h"
#include "IndentationLog.h"
#include "Optimizer.h"

// This file contains structures that represent a set of vertex that have been prepared for optimization
// (i.e. all relevant information is immediately available, e.g. parallelization strategy or attributes)
// Two variants are implemented - one that explicitly stores the neighbors, and another that passes the
// query on to the hierarchy. The inheritance hierarchy uses the Curiously Recurring Template Pattern.

namespace osr
{
	// This index type wraps size_t and is used to distinguish queries for the vertex set and for the original hierarchy.
	struct VertexSetIndex
	{
		VertexSetIndex(size_t i) : index(i) { }
		size_t index;
	};

	// Commonalities of both variants of the vertex set
	template <typename Hierarchy, typename Index, typename Derived, typename StoredVertexType>
	struct PreparedVertexSetBase
	{
		PreparedVertexSetBase()
			: hierarchy(nullptr) { }

		PreparedVertexSetBase(Hierarchy* h)
			: hierarchy(h) { }

		template <Attribute A> typename AttributeTraits<A>::Type& attribute(VertexSetIndex i) { return hierarchy->attribute<A>(vertices[i.index].vertex); }
		template <Attribute A> typename AttributeTraits<A>::Type& attribute(Index i) { return hierarchy->attribute<A>(i); }

		Hierarchy* hierarchy;

		class PhaseIterator : public std::iterator<std::random_access_iterator_tag, size_t>
		{
		public:
			PhaseIterator(size_t it)
				: it(VertexSetIndex(it))
			{ }

			const PhaseIterator& operator++() { ++it.index; return *this; }
			const PhaseIterator& operator+=(size_t n) { it.index += n; return *this; }
			PhaseIterator operator+(size_t n) const { return PhaseIterator(it.index + n); }

			const PhaseIterator& operator--() { --it.index; return *this; }
			const PhaseIterator& operator-=(size_t n) { it.index -= n; return *this; }
			PhaseIterator operator-(size_t n) const { return PhaseIterator(it.index - n); }

			size_t operator-(const PhaseIterator& other) const { return it.index - other.it.index; }

			bool operator<(const PhaseIterator& other) const { return it.index < other.it.index; }
			bool operator>(const PhaseIterator& other) const { return it.index > other.it.index; }
			bool operator<=(const PhaseIterator& other) const { return it.index <= other.it.index; }
			bool operator>=(const PhaseIterator& other) const { return it.index >= other.it.index; }
			bool operator==(const PhaseIterator& other) const { return it.index == other.it.index; }
			bool operator!=(PhaseIterator& other) const { return it.index != other.it.index; }
			VertexSetIndex& operator*() { return it; }
		private:
			VertexSetIndex it;
		};

		class PhasesIterator
		{
		public:
			PhasesIterator(std::vector<size_t>::iterator it, Derived& set)
				: it(it), set(set)
			{ }

			const PhasesIterator& operator++() { ++it; return *this; }
			bool operator!=(PhasesIterator& other) const { return it != other.it; }
			ForEachHelper<PhaseIterator> operator*()
			{
				auto nextIt = it; ++nextIt;
				auto endIt = (nextIt == set.phaseStarts.end() ? set.includedVertices : *nextIt);
				return ForEachHelper<PhaseIterator>(PhaseIterator(*it), PhaseIterator(endIt));
			}

		private:
			std::vector<size_t>::iterator it;
			Derived& set;
		};

		// All vertices in the current set. The specific subclass defines what exactly is stored for every vertex.
		std::vector<StoredVertexType> vertices;

		// For every parallelization phase, stores the index of the first vertex.
		std::vector<size_t> phaseStarts;

		// The number of vertices in the set. vertices may include more than these in case neighbors are explicitly stored.
		size_t includedVertices;

		// Causes the specified fields of the vertices in the set to be optimized.
		template <bool WithDirFieldConstraints, Attribute ... Attributes>
		void optimize(const Optimizer& o);

		void optimize(const Optimizer& o)
		{
			optimize<DirField, PosField>(o);
		}

		void optimizeNormals(const Optimizer& o)
		{
			ForEachHelper<PhasesIterator> phasesHelper(PhasesIterator(phaseStarts.begin(), *static_cast<Derived*>(this)), PhasesIterator(phaseStarts.end(), *static_cast<Derived*>(this)));

			o.optimizeNormals(*static_cast<Derived*>(this), phasesHelper);
		}
	};

	template <bool WithDirFieldConstraints, Attribute A, typename Hierarchy, typename Index, typename Derived, typename StoredVertexType>
	struct VertexSetOptimizeHelper
	{
		static void optimize(Derived& dataStore, const Optimizer& o, ForEachHelper<typename PreparedVertexSetBase<Hierarchy, Index, Derived, StoredVertexType>::PhasesIterator>& phases)
		{ }
	};

	template <bool WithDirFieldConstraints, typename Hierarchy, typename Index, typename Derived, typename StoredVertexType>
	struct VertexSetOptimizeHelper<WithDirFieldConstraints, DirField, Hierarchy, Index, Derived, StoredVertexType>
	{
		static void optimize(Derived& dataStore, const Optimizer& o, ForEachHelper<typename PreparedVertexSetBase<Hierarchy, Index, Derived, StoredVertexType>::PhasesIterator>& phases)
		{
			o.optimizeOrientations<WithDirFieldConstraints>(dataStore, phases);
		}
	};

	template <bool WithDirFieldConstraints, typename Hierarchy, typename Index, typename Derived, typename StoredVertexType>
	struct VertexSetOptimizeHelper<WithDirFieldConstraints, PosField, Hierarchy, Index, Derived, StoredVertexType>
	{
		static void optimize(Derived& dataStore, const Optimizer& o, ForEachHelper<typename PreparedVertexSetBase<Hierarchy, Index, Derived, StoredVertexType>::PhasesIterator>& phases)
		{
			o.optimizePositions(dataStore, phases);
		}
	};

	template <typename Hierarchy, typename Index, typename Derived, typename StoredVertexType>
	template <bool WithDirFieldConstraints, Attribute ... Attributes>
	void PreparedVertexSetBase<Hierarchy, Index, Derived, StoredVertexType>::optimize(const Optimizer& o)
	{
		ForEachHelper<PhasesIterator> phasesHelper(PhasesIterator(phaseStarts.begin(), *static_cast<Derived*>(this)), PhasesIterator(phaseStarts.end(), *static_cast<Derived*>(this)));

		TimedBlock b("Optimizing " + std::to_string(this->includedVertices) + " vertices ..");

		int dummy[] = { 0, (VertexSetOptimizeHelper<WithDirFieldConstraints, Attributes, Hierarchy, Index, Derived, StoredVertexType>::optimize(*static_cast<Derived*>(this), o, phasesHelper), 0)... };
		(void)dummy; //suppress compiler warnings for unused dummy
	}

	// Base template for the vertex set
	template <typename Hierarchy, typename Index, bool StoreNeighbors = true, bool UseOriginalIndexForNeighbors = true>
	struct PreparedVertexSet
	{ };


	template <typename Index>
	struct VertexWithNeighbors
	{
		Index vertex;

		// Index of the first neighbor in the neighbors vector
		size_t neighborOffset;
	};

	// Vertex set that stores neighbors explicitly.
	template <typename Hierarchy, typename Index>
	struct PreparedVertexSet<Hierarchy, Index, true, true> : public PreparedVertexSetBase<Hierarchy, Index, PreparedVertexSet<Hierarchy, Index, true, true>, VertexWithNeighbors<Index>>
	{
		PreparedVertexSet() { }

		PreparedVertexSet(Hierarchy* h)
			: PreparedVertexSetBase<Hierarchy, Index, PreparedVertexSet<Hierarchy, Index, true, true>, VertexWithNeighbors<Index>>(h) { }

		//iterate the stored neighbors
		template <typename Callback>
		void forEachNeighbor(const VertexSetIndex v, const Callback& callback)
		{
			auto start = this->vertices[v.index].neighborOffset;
			auto end = (v.index >= this->includedVertices - 1 ? neighbors.size() : this->vertices[v.index + 1].neighborOffset);
			for (size_t n = start; n != end; ++n)
				callback(neighbors[n]);
		}

		// Add a vector of neighbors to the set
		std::vector<Index> neighbors;
	};

	// Vertex set that stores neighbors explicitly and addresses them with an internal zero-based integer index.
	template <typename Hierarchy, typename Index>
	struct PreparedVertexSet<Hierarchy, Index, true, false> : public PreparedVertexSetBase<Hierarchy, Index, PreparedVertexSet<Hierarchy, Index, true, false>, VertexWithNeighbors<Index>>
	{
		PreparedVertexSet() { }

		PreparedVertexSet(Hierarchy* h)
			: PreparedVertexSetBase<Hierarchy, Index, PreparedVertexSet<Hierarchy, Index, true, false>, VertexWithNeighbors<Index>>(h) { }

		//iterate the stored neighbors
		template <typename Callback>
		void forEachNeighbor(const VertexSetIndex v, const Callback& callback)
		{
			auto start = this->vertices[v.index].neighborOffset;
			auto end = (v.index >= this->includedVertices - 1 ? neighbors.size() : this->vertices[v.index + 1].neighborOffset);
			for (size_t n = start; n != end; ++n)
				callback(VertexSetIndex(neighbors[n]));
		}

		void expandBy(float radius)
		{
			TimedBlock b("Expanding vertex set ..");

			//Add more vertices as needed

			std::vector<unsigned char> skipQuery(this->vertices.size() - this->includedVertices);
			int num_procs = 1;
#ifdef OPENMP
			num_procs = omp_get_num_procs();
#endif

			std::vector<std::vector<Index>> additionalVertices(num_procs);
			{
				TimedBlock b("Radius queries ..");
#ifdef OPENMP
				additionalVertices[omp_get_thread_num()].reserve(this->includedVertices / 10 / omp_get_num_procs());
#else
				additionalVertices[0].reserve(this->includedVertices / 10);
#endif
#pragma omp parallel for
				for (int i = this->includedVertices; i < this->vertices.size(); ++i)
				{
					if (skipQuery[i - this->includedVertices] != 0)
						continue;
					this->hierarchy->findNearestPointsRadius(this->hierarchy->template attribute<Position>(this->vertices[i].vertex), radius,
						[&](const auto& neighbor, float d)
					{
						auto it = indexToVectorPosition.find(neighbor); // <-- this search takes quite some time if the set is large; can this be improved?
						if (it == indexToVectorPosition.end())
#ifdef OPENMP
							additionalVertices[omp_get_thread_num()].push_back(neighbor);
#else
							additionalVertices[0].push_back(neighbor);
#endif
						else if (it->second >= this->includedVertices && d < 0.1f * radius) //do not query points that are very close to this query as they will produce nearly the same result
							skipQuery[it->second - this->includedVertices] = 1;

					});
				}
			}

			{
				TimedBlock b("Updating vertices ..");
				for (int i = 0; i < additionalVertices.size(); ++i)
					for (auto& n : additionalVertices[i])
					{
						if (indexToVectorPosition.find(n) == indexToVectorPosition.end())
						{
							this->vertices.push_back({ n, neighbors.size() });
							indexToVectorPosition[n] = this->vertices.size() - 1;
						}
					}
			}

			//Find neighbors for new vertices
			std::vector<std::vector<size_t>> localNeighbors(this->vertices.size() - this->includedVertices);
			{
				TimedBlock b("Neighbor queries for new vertices ..");
#pragma omp parallel for
				for (int i = this->includedVertices; i < this->vertices.size(); ++i)
				{
					this->hierarchy->forEachNeighbor(this->vertices[i].vertex, [&](const auto& n)
					{
						auto it = indexToVectorPosition.find(n);
						if (it != indexToVectorPosition.end())
							localNeighbors[i - this->includedVertices].push_back(it->second);
					});
				}
			}

			{
				TimedBlock b("Updating neighbors ..");
				for (int i = this->includedVertices; i < this->vertices.size(); ++i)
				{
					auto& myNeighbors = localNeighbors[i - this->includedVertices];
					this->vertices[i].neighborOffset = neighbors.size();
					neighbors.resize(neighbors.size() + myNeighbors.size());
					memcpy(neighbors.data() + this->vertices[i].neighborOffset, myNeighbors.data(), myNeighbors.size() * sizeof(size_t));
				}
			}

			this->includedVertices = this->vertices.size();
		}

		// Add a vector of neighbors to the set
		std::vector<size_t> neighbors;

		// Maps the original index to a local index
		std::unordered_map<Index, size_t> indexToVectorPosition;
	};

	template <typename Index>
	struct VertexWithoutNeighbors
	{
		Index vertex;
	};

	// Vertex set that does not store neighbors explicitly.
	template <typename Hierarchy, typename Index>
	struct PreparedVertexSet<Hierarchy, Index, false, true> : public PreparedVertexSetBase<Hierarchy, Index, PreparedVertexSet<Hierarchy, Index, false, true>, VertexWithoutNeighbors<Index>>
	{
		PreparedVertexSet() { }

		PreparedVertexSet(Hierarchy* h)
			: PreparedVertexSetBase<Hierarchy, Index, PreparedVertexSet<Hierarchy, Index, false, true>, VertexWithoutNeighbors<Index>>(h)
		{ }

		// pass the query on to the hierarchy
		template <typename Callback>
		void forEachNeighbor(const VertexSetIndex v, const Callback& callback)
		{
			this->hierarchy->forEachNeighbor(this->vertices[v.index].vertex, callback);
		}
	};
}