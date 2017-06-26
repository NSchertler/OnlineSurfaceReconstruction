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

#include "osr/Hierarchy.h"
#include "osr/Attributes.h"
#include "osr/AttributeConsistency.h"
#include "osr/INeighborQueryable.h"
#include "osr/Indexing.h"
#include "osr/Morton.h"

#include "osr/common.h"
#include "osr/Hierarchy.h"
#include "osr/Optimizer.h"
#include "osr/HierarchyCapabilities.h"
#include "osr/PersistentIndexContainer.h"
#include "osr/Serialization.h"

#include <algorithm>
#include <set>
#include <boost/align/aligned_allocator.hpp>

namespace osr
{
	//Octree-based hierarchy with Morton Index-based data storage
	namespace HierarchyMortonMultiPass
	{
		//number of shifted reresentations of the original data
		const int SHIFTS = 4;

		//number of virtual subdivisions for each dimension in a cell to find the Morton-order within the cell
		const int CELL_OVER_SUBDIV = 4;

		//specifies how far to look in each direction in the surrounding of a given vertex
		//in the shifted representations       <---- v ---->
		const int SHIFTED_NEIGHBOR_RANGE = 2;

		//the offset used for shifted sorting
		const unsigned int mortonShift = 5;
		const MortonCode64 offsetMorton = MortonCode64(5, 5, 5);

		//type of nodes that are stored in the hierarchy
		struct Node
		{
			Node();

			Node(const Vector3f& position, const Vector3f& normal);

			Node(const Vector3f& position, const Vector3f& normal, float area);

			Node(const Node& copy);

			Node& operator=(const Node& copy);

			Node& operator=(Node&& move);

			Vector3f position, normal;
			Vector3f dirField, posField;

			//sum of all constituent points' areas
			float area;

			//Nodes on the finest level have partially different attributes than nodes on the coarser levels.
			union
			{
				struct
				{
					Vector3us color;
					unsigned char meshVertexData[6]; //do not put the constituent variables here to avoid padding
				} forFinestLevel;

				struct
				{
					Vector3f directionConstraint;
				} forCoarserLevels;
			};


			template<Attribute A> typename AttributeTraits<A>::Type& attribute();
		};

		template<> typename AttributeTraits<Position>::Type& Node::attribute<Position>();
		template<> typename AttributeTraits<Normal>::Type& Node::attribute<Normal>();
		template<> typename AttributeTraits<DirField>::Type& Node::attribute<DirField>();
		template<> typename AttributeTraits<PosField>::Type& Node::attribute<PosField>();
		template<> typename AttributeTraits<Area>::Type& Node::attribute<Area>();
		template<> typename AttributeTraits<MeshVertex>::Type& Node::attribute<MeshVertex>();
		template<> typename AttributeTraits<MeshVertexGeneration>::Type& Node::attribute<MeshVertexGeneration>();
		template<> typename AttributeTraits<Color>::Type& Node::attribute<Color>();
		template<> typename AttributeTraits<DirFieldConstraint>::Type& Node::attribute<DirFieldConstraint>();

		//This class holds information on how the average values of a node should be updated as well as the old field values.
		class NodeState
		{
		public:
			NodeState()
				: avgPositionAdded(Vector3f::Zero()), avgNormalAdded(Vector3f::Zero()), weightAdded(0),
				avgPositionRemoved(Vector3f::Zero()), avgNormalRemoved(Vector3f::Zero()), weightRemoved(0),
				posField(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()),
				dirField(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN())
			{ }

			NodeState(MortonCode64 morton)
				: mortonIdx(morton),
				avgPositionAdded(Vector3f::Zero()), avgNormalAdded(Vector3f::Zero()), weightAdded(0),
				avgPositionRemoved(Vector3f::Zero()), avgNormalRemoved(Vector3f::Zero()), weightRemoved(0),
				posField(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()),
				dirField(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN())
			{ }

			MortonCode64 mortonIdx;
			bool operator<(const NodeState& rhs) const
			{
				return mortonIdx < rhs.mortonIdx;
			}

			template<Attribute A>
			typename AttributeTraits<A>::Type& avgAdded() { throw std::runtime_error("Field retrieval not implemented for type " + AttributeTraits<A>::name() + "."); }

			template<Attribute A>
			const typename AttributeTraits<A>::Type& avgAdded() const { throw std::runtime_error("Field retrieval not implemented for type " + AttributeTraits<A>::name() + "."); }

			template<Attribute A>
			typename AttributeTraits<A>::Type& avgRemoved() { throw std::runtime_error("Field retrieval not implemented for type " + AttributeTraits<A>::name() + "."); }

			template<Attribute A>
			const typename AttributeTraits<A>::Type& avgRemoved() const { throw std::runtime_error("Field retrieval not implemented for type " + AttributeTraits<A>::name() + "."); }

			void reset()
			{
				weightAdded = 0;
				avgPositionAdded.setZero();
				avgNormalAdded.setZero();
			}

			template <Attribute ... Attributes>
			void applyChangesToNode(Node& node) const;

			template <Attribute ... Attributes>
			void addData(float weight, const typename AttributeTraits<Attributes>::Type&... args);

			template <Attribute ... Attributes>
			void removeData(float weight, const typename AttributeTraits<Attributes>::Type&... args);

			template <Attribute ... Attributes>
			void addData(const NodeState& other);

			Vector3f avgPositionAdded;
			Vector3f avgNormalAdded;
			float weightAdded;
			Vector3f avgPositionRemoved;
			Vector3f avgNormalRemoved;
			float weightRemoved;

			Vector3f posField, dirField;
		private:

			template <Attribute A>
			void addChangesToNode(Node& node) const;

			template <Attribute A>
			void addChange(const typename AttributeTraits<A>::Type& attribute, float weight);

			template <Attribute A>
			void removeChange(const typename AttributeTraits<A>::Type& attribute, float weight);
		};

		template<> typename AttributeTraits<Position>::Type& NodeState::avgAdded<Position>();
		template<> typename AttributeTraits<Normal>::Type& NodeState::avgAdded<Normal>();
		template<> const typename AttributeTraits<Position>::Type& NodeState::avgAdded<Position>() const;
		template<> const typename AttributeTraits<Normal>::Type& NodeState::avgAdded<Normal>() const;
		template<> typename AttributeTraits<Position>::Type& NodeState::avgRemoved<Position>();
		template<> typename AttributeTraits<Normal>::Type& NodeState::avgRemoved<Normal>();
		template<> const typename AttributeTraits<Position>::Type& NodeState::avgRemoved<Position>() const;
		template<> const typename AttributeTraits<Normal>::Type& NodeState::avgRemoved<Normal>() const;

		template <Attribute ... Attributes>
		void NodeState::applyChangesToNode(Node& node) const
		{
			static_assert(sizeof...(Attributes) > 0, "Attribute list must not be empty.");
			node.area += weightAdded - weightRemoved;
			if (node.area > 0)
			{
				int dummy[] = { 0, (addChangesToNode<Attributes>(node), 0)... };
				(void)dummy; //suppress compiler warnings for unused dummy
			}
		}

		template <Attribute ... Attributes>
		void NodeState::addData(float weight, const typename AttributeTraits<Attributes>::Type&... args)
		{
			static_assert(sizeof...(Attributes) > 0, "Attribute list must not be empty.");
			weightAdded += weight;
			int dummy[] = { 0, (addChange<Attributes>(args, weight), 0)... };
			(void)dummy; //suppress compiler warnings for unused dummy
		}

		template <Attribute ... Attributes>
		void NodeState::removeData(float weight, const typename AttributeTraits<Attributes>::Type&... args)
		{
			static_assert(sizeof...(Attributes) > 0, "Attribute list must not be empty.");
			weightRemoved += weight;
			int dummy[] = { 0, (removeChange<Attributes>(args, weight), 0)... };
			(void)dummy; //suppress compiler warnings for unused dummy
		}

		template <Attribute ... Attributes>
		void NodeState::addData(const NodeState& other)
		{
			static_assert(sizeof...(Attributes) > 0, "Attribute list must not be empty.");
			if (other.weightAdded > 0)
				addData<Attributes...>(other.weightAdded, other.avgAdded<Attributes>()...);
			if (other.weightRemoved > 0)
				removeData<Attributes...>(other.weightRemoved, other.avgRemoved<Attributes>()...);
		}

		template <Attribute A>
		void NodeState::addChangesToNode(Node& node) const
		{
			auto prevAttribute = node.attribute<A>();
			node.attribute<A>() += (weightAdded * (avgAdded<A>() - node.attribute<A>()) - weightRemoved * (avgRemoved<A>() - node.attribute<A>())) / node.area;
			if (!AttributeConsistency<A>::makeConsistent(node))
				node.attribute<A>() = prevAttribute;
		}

		template <Attribute A>
		void NodeState::addChange(const typename AttributeTraits<A>::Type& attribute, float weight)
		{
			avgAdded<A>() += weight / weightAdded * (attribute - avgAdded<A>());
		}

		template <Attribute A>
		void NodeState::removeChange(const typename AttributeTraits<A>::Type& attribute, float weight)
		{
			avgRemoved<A>() += weight / weightRemoved * (attribute - avgRemoved<A>());
		}

		//Represents a reference to a node whose position has been shifted
		struct ShiftedRepresentation
		{
			ShiftedRepresentation() { }

			ShiftedRepresentation(MortonCode64 mortonIdx)
				: mortonIdx(mortonIdx), vRef(-1)
			{ }

			ShiftedRepresentation(MortonCode64 mortonIdx, size_t vRef)
				: mortonIdx(mortonIdx), vRef(vRef)
			{ }

			MortonCode64 mortonIdx;
			size_t vRef;

			bool operator==(const ShiftedRepresentation& rhs) const { return vRef == rhs.vRef; }

			bool operator<(const ShiftedRepresentation& rhs) const
			{
				if (mortonIdx != rhs.mortonIdx)
					return mortonIdx < rhs.mortonIdx;
				else if (vRef == (size_t)-1 && rhs.vRef == (size_t)-1)
					return false;
				else if (vRef == (size_t)-1)
					return true;
				else if (rhs.vRef == (size_t)-1)
					return false;
				else
					return vRef < rhs.vRef;
			}
		};

		//a container that can hold values of T, addressed by MortonCodes efficiently (i.e. allows efficient insert and re-sort).
		//T must have a field MortonCode64 mortonIdx
		//T must have a constructor that takes a MortonCode64
		//T must have a field size_t vRef
		template <typename T>
		class MortonContainer
		{
		public:

			typedef typename std::vector<T>::iterator iterator;

			template <typename FwdIterator, typename RevIterator>
			void addSortedPoints(FwdIterator fwdBegin, FwdIterator fwdEnd, RevIterator revBegin, RevIterator revEnd)
			{
				if (data.size() == 0)
					data.insert(data.begin(), fwdBegin, fwdEnd);
				else
				{
					//merge
					size_t newData = fwdEnd - fwdBegin;
					size_t oldSize = data.size();
					data.resize(data.size() + newData);
					auto dataIt = data.rbegin() + newData;
					auto vectorIt = revBegin;
					auto outIt = data.rbegin();
					while (dataIt != data.rend() && vectorIt != revEnd)
					{
						if (*dataIt < *vectorIt)
							*outIt++ = *vectorIt++;
						else
							*outIt++ = *dataIt++;
					}
					while (vectorIt != revEnd)
					{
						*outIt++ = *vectorIt++;
					}
				}
			}

			typename std::vector<T>::iterator erase(typename std::vector<T>::iterator it) { return data.erase(it); }
			typename std::vector<T>::const_iterator erase(typename std::vector<T>::const_iterator it) { return data.erase(it); }

			//Erases all entries that have an invalid vertex reference
			void eraseInvalid()
			{
				data.erase(std::remove_if(data.begin(), data.end(), [](const T& d) { return d.vRef == (size_t)-1; }), data.end());
			}

			T& at(size_t i) { return data.at(i); }

			typename std::vector<T>::const_iterator find(const T &v) const
			{
				auto it = std::lower_bound(data.begin(), data.end(), v);
				if (*it == v)
					return it;
				else
					return end();
			}

			typename std::vector<T>::iterator find(const T &v)
			{
				auto it = std::lower_bound(data.begin(), data.end(), v);
				if (*it == v)
					return it;
				else
					return end();
			}

			typename std::vector<T>::const_iterator find(MortonCode64 i) const
			{
				auto it = std::lower_bound(data.begin(), data.end(), T(i));
				if (it->mortonIdx == i)
					return it;
				else
					return end();
			}

			typename std::vector<T>::iterator find(MortonCode64 i)
			{
				auto it = std::lower_bound(data.begin(), data.end(), T(i));
				if (it->mortonIdx == i)
					return it;
				else
					return end();
			}

			T& findOrCreate(MortonCode64 i, bool& existedBefore)
			{
				auto it = std::lower_bound(data.begin(), data.end(), T(i));
				if (it == data.end() || it->mortonIdx != i)
				{
					it = data.insert(it, T(i));
					existedBefore = false;
				}
				else
					existedBefore = true;
				return *it;
			}

			void reserve(size_t n) { data.reserve(n); }

			//Inserts a number of sorted elements into the data structure if they do not exist. The result vector contains
			//an entry for every input element with an iterator to the element position and a bool that represents if
			//the element has existed before the insertion.
			template <typename Iterator>
			void findOrCreate(Iterator elementsBegin, Iterator elementsEnd, std::vector<std::pair<iterator, bool>>& result)
			{
				auto elementCount = elementsEnd - elementsBegin;
				result.resize(elementCount);
				std::vector<T> insertVector; //elements that need to be inserted at the current position
				if (elementCount > 0)
				{
					data.reserve(data.size() + elementCount); //make sure iterators stay valid

					auto elementsIt = elementsBegin;

					auto dataIt = std::lower_bound(data.begin(), data.end(), *elementsIt);

					auto insertData = [&]()
					{
						if (insertVector.size() == 0)
							return;
						dataIt = data.insert(dataIt, insertVector.begin(), insertVector.end());
						for (auto it = elementsIt - insertVector.size(); it < elementsIt; ++it)
							result[it - elementsBegin] = std::make_pair(dataIt++, false);
						insertVector.clear();
					};

					while (elementsIt < elementsEnd)
					{
						if (dataIt->mortonIdx < elementsIt->mortonIdx)
							dataIt = std::lower_bound(dataIt, data.end(), *elementsIt);

						//if we are at the end of the structure, just insert all remaining elements
						if (dataIt == data.end())
						{
							for (; elementsIt < elementsEnd; ++elementsIt)
								insertVector.push_back(*elementsIt);
							insertData();
							break;
						}

						//if the current element is already present, add all previous elements
						if (dataIt->mortonIdx == elementsIt->mortonIdx)
						{
							insertData();

							result[elementsIt++ - elementsBegin] = std::make_pair(dataIt++, true);
						}
						else
						{
							//schedule the new element for insertion
							insertVector.push_back(*elementsIt++);
						}
					}
					insertData();
				}
			}

			//Searches for a node with the given index. If it does not exist, it creates a new one
			T& operator[](MortonCode64 i)
			{
				bool e;
				return findOrCreate(i, e);
			}

			//Searches for a node with the given index. If it does not exist, it creates a new one
			T& operator[](size_t i)
			{
				return data[i];
			}

			//Calls the callback for every element within the specified range.
			//callback: void(const T&)
			template <typename Function>
			void forEachInRange(MortonCode64 lowerInclusive, MortonCode64 upperExclusive, const Function& callback) const
			{
				auto it = std::lower_bound(data.begin(), data.end(), T(lowerInclusive));
				while (it != data.end() && it->mortonIdx < upperExclusive)
					callback(*it++);
			}

			size_t size() const { return data.size(); }

			typename std::vector<T>::const_iterator begin() const { return data.begin(); }
			typename std::vector<T>::const_iterator end() const { return data.end(); }
			typename std::vector<T>::iterator begin() { return data.begin(); }
			typename std::vector<T>::iterator end() { return data.end(); }

			size_t sizeInBytes() const
			{
				return data.size() * sizeof(T);
			}

			//Saves the current state of this collection to file
			void saveToFile(FILE* f) const
			{
				osr::saveToFile(data, f);
			}

			//Restores the state of this collection from file
			void loadFromFile(FILE* f)
			{
				osr::loadFromFile(data, f);
			}

		private:
			std::vector<T> data;
		};

		//Holds all relevant information for a level within the hierarchy.
		struct LevelInfo
		{
			//alignment improves performance by a tiny bit
			typedef PersistentIndexContainer<Node, boost::alignment::aligned_allocator<Node, 64>> DataContainer;

			//Set of nodes within this level
			DataContainer originalData;
			//Copies of the nodes, each shifted by a given offset
			MortonContainer<ShiftedRepresentation> shiftedSequences[SHIFTS];

			//specifies the offset to add to a global index in order to transform it to a tree-local index (all positive)
			MortonCode64 toLocalOffset;

			LevelInfo()
				: toLocalOffset(MortonCode64::Zero)
			{ }

			size_t sizeInBytes() const
			{
				size_t s = sizeof(LevelInfo);
				for (int i = 0; i < SHIFTS; ++i)
					s += shiftedSequences[i].sizeInBytes();
				s += originalData.sizeWithGaps() * 64;

				return s;
			}

			Node& node(const MortonCode64 i) { return originalData[shiftedSequences[0].find(i)->vRef]; }

			void erase(size_t i, MortonCode64 code)
			{
				for (int s = 0; s < SHIFTS; ++s)
				{
					shiftedSequences[s].erase(shiftedSequences[s].find(ShiftedRepresentation(code, i)));
					code += offsetMorton;
				}
				originalData.erase(i);
			}

			//Sets the vertex reference of the specified node to INVALID.
			void prepareForErasure(size_t i, MortonCode64 code)
			{
				for (int s = 0; s < SHIFTS; ++s)
				{
					auto it = shiftedSequences[s].find(ShiftedRepresentation(code, i));
					it->vRef = (size_t)-1;
					while (it != shiftedSequences[s].begin() && (it - 1)->mortonIdx == it->mortonIdx) //all invalid indices must be the first entries in the cell
					{
						std::swap(*it, *(it - 1));
						--it;
					}
					code += offsetMorton;
				}
				originalData.erase(i);
			}

			//Erases all nodes with an INVALID vertex reference.
			void finalizeErasure()
			{
				for (int s = 0; s < SHIFTS; ++s)
				{
					shiftedSequences[s].eraseInvalid();
				}
			}
		};

		struct Index
		{
			Index() { }

			Index(size_t idx, unsigned int level)
				: idx(idx), level(level)
			{ }

			bool operator==(const Index& other) const { return idx == other.idx && level == other.level; }
			bool operator<(const Index& other) const { return idx < other.idx; }

			size_t idx;
			unsigned int level;
		};
	}
}

namespace std
{
	template <>
	struct hash<osr::HierarchyMortonMultiPass::Index>
	{
		size_t operator()(const osr::HierarchyMortonMultiPass::Index& o) const
		{
			return o.idx;
		}
	};
}

namespace osr
{

	namespace HierarchyMortonMultiPass
	{
		class Hierarchy : public AbstractHierarchy<Index>, public IPointQueryable<size_t>
		{
		public:

			Hierarchy(const Optimizer& optimizer, ExtractedMesh& extractionResult);
			~Hierarchy();

			// ---- begin public hierarchy interface  ----

			//this is the type referred to by vertices()
			typedef Index VertexIndex;

			//access to the data stored in the hierarchy
			template<Attribute A> typename AttributeTraits<A>::Type& attribute(const Index& i) { return mLevels[i.level].originalData[i.idx].attribute<A>(); }
			template<Attribute A> const typename AttributeTraits<A>::Type& attribute(const Index& i) const { return mLevels[i.level].originalData[i.idx].attribute<A>(); }

			void addPoints(const Matrix3Xf& V, const Matrix3Xf& N, const Matrix3Xus& C);

			void modifyPoints(const std::vector<VertexIndex>& points, const Matrix3Xf& newV, const Matrix3Xf& newN, const Matrix3Xus& newC);

			void removePoints(std::vector<VertexIndex>& points);

			void optimizeFull();

			void reset();

			class VertexIterator;
			//returns an iterator for all vertices of the finest level
			ForEachHelper<VertexIterator> vertices(int level = 0);

			template <typename Callback>
			void forEachNeighbor(const Index& v, const Callback& callback);

			template <typename Callback> // void(const Index&)
			void findNearestPointsRadius(const Vector3f& p, Float radius, const Callback& callback) const;

			//returns the number of vertices on the finest level of the hierarchy
			size_t vertexCount() const { return mVertexCount; }
			size_t vertexCount(int level) const { return mVertexCount == 0 ? 0 : mLevels[level].originalData.sizeNotDeleted(); }

			size_t sizeInBytes() const;

			int levels() const;

			size_t optimizedPoints;

			float averagePointSpacing() const { return pointSpacing; }

			void saveToFile(FILE* f) const;
			void loadFromFile(FILE* f);

			// ---- end public hierarchy interface  ----

			// ----       other interfaces          ----
			size_t findClosestCompatiblePoint(const Vector3f& p, const Vector3f& n) const;
			bool isIndexValid(const size_t& idx) const;
			Vector3f neighborP(const size_t& i) const;
			Vector3f neighborN(const size_t& i) const;

			// ----     end other interfaces        ----

			class VertexIterator : public std::iterator<std::forward_iterator_tag, Index>
			{
			public:
				VertexIterator(LevelInfo::DataContainer::iterator it, int level);

				const VertexIterator& operator++();
				bool operator!=(VertexIterator& other) const;
				Index operator*();

			private:
				LevelInfo::DataContainer::iterator it;
				int level;
			};

			template <typename Iterator>
			class VertexIteratorAdaptor : public std::iterator<std::forward_iterator_tag, Index>
			{
			public:
				VertexIteratorAdaptor(Iterator it, int level);

				const VertexIteratorAdaptor& operator++();
				bool operator!=(VertexIteratorAdaptor& other) const;
				Index operator*();

			private:
				Iterator it;
				int level;
			};
			template <typename Iterator>
			VertexIteratorAdaptor<Iterator> adaptToVertexIterator(Iterator it, int level) const;

			MortonCode64 mortonCode(const Vector3f& p, int level = 0) const;

		private:
			MortonCode64 parent(MortonCode64 idx, int childNodeLevel, int levelsUp = 1) const;
			void children(const MortonCode64 idx, int parentNodeLevel, MortonCode64& childLowerInclusive, MortonCode64& childUpperExclusive) const;

			void init();
			void updateHierarchy(const Matrix3Xf & V, const Matrix3Xf& N, const Matrix3Xus& C, const std::vector<VertexIndex>& originalIndices, std::vector<Index>& removedVertices);

			// Filters the changes described by levelStates up the hierarchy. levelStates[0] is ignored.
			template <Attribute ... Attributes>
			void ApplyChangesToHierarchy(std::vector<MortonContainer<NodeState>>& levelStates, bool storeOldAttributes);

			template <Attribute ... Attributes>
			PreparedVertexSet<Hierarchy, Index, true, false> OptimizePart(std::vector<MortonContainer<NodeState>>& levelStates, bool checkForChildChange);

			template <Attribute ... Attributes>
			PreparedVertexSet<Hierarchy, Index, true, false> Optimize();

			template <bool IncludeReference, typename Callback>
			void approximateKNN(ShiftedRepresentation& reference, int levelNo, unsigned int k, unsigned int lookAroundSize, float maxDistanceSq, const Callback& callback);

			template <Attribute A>
			void initializeWithParentSolution(Node& node, Node& parentNode);

			Vector3f origin;
			Int3Index gridMin, gridMax;
			float gridSize = 0;

			std::vector<LevelInfo> mLevels;

			size_t mVertexCount;

			float pointSpacing;
		};

		// ----  Template implementations  ----

		template <bool IncludeReference, typename Callback>
		void Hierarchy::approximateKNN(ShiftedRepresentation& reference, int levelNo, unsigned int k, unsigned int lookAroundSize, float maxDistanceSq, const Callback& callback)
		{
			auto& level = mLevels[levelNo];
			const Vector3f& currentP = level.originalData[reference.vRef].position;

			std::set<NeighborStrictOrder<size_t>> neighbors;
			for (int m = 0; m < SHIFTS; ++m)
			{
				auto it = level.shiftedSequences[m].find(reference);
				auto startIt = it;
				for (int j = 0; j < lookAroundSize; ++j)
				{
					++it;
					if (it == level.shiftedSequences[m].end())
						break;

					auto& n = level.originalData[it->vRef];
					float l = (n.position - currentP).squaredNorm();
					if (l <= maxDistanceSq)
						neighbors.insert(NeighborStrictOrder<size_t>(it->vRef, l));
				}
				it = startIt;
				for (int j = 0; j < lookAroundSize; ++j)
				{
					if (it == level.shiftedSequences[m].begin())
						break;
					--it;
					auto& n = level.originalData[it->vRef];
					float l = (n.position - currentP).squaredNorm();
					if (l <= maxDistanceSq)
						neighbors.insert(NeighborStrictOrder<size_t>(it->vRef, l));
				}
				//shift the morton index
				reference.mortonIdx += offsetMorton;
			}
			int emittedNeighbors = 0;
			auto it = neighbors.begin();
			while (it != neighbors.end() && emittedNeighbors < k)
			{
				auto& neighbor = *it;
				++it;

				callback(Index(neighbor.idx, levelNo));
				++emittedNeighbors;
			}
		}

		template <typename Callback>
		void Hierarchy::forEachNeighbor(const Index& v, const Callback& callback)
		{
			if (vertexCount() == 0)
				return;

			if (v.level == mLevels.size() - 1)
				return;

			auto& level = mLevels[v.level];
			float allowedDistance = optimizer.meshSettings().scale() * (1 << v.level);
			ShiftedRepresentation rep(mortonCode(level.originalData[v.idx].position, v.level), v.idx);

			approximateKNN<false>(rep, v.level, 8, SHIFTED_NEIGHBOR_RANGE, allowedDistance * allowedDistance, callback);
		}


		template <typename Iterator>
		Hierarchy::VertexIteratorAdaptor<Iterator>::VertexIteratorAdaptor(Iterator it, int level)
			: it(it), level(level)
		{ }

		template <typename Iterator>
		const Hierarchy::VertexIteratorAdaptor<Iterator>& Hierarchy::VertexIteratorAdaptor<Iterator>::operator++() { ++it; return *this; }

		template <typename Iterator>
		bool Hierarchy::VertexIteratorAdaptor<Iterator>::operator!=(Hierarchy::VertexIteratorAdaptor<Iterator>& other) const { return it != other.it; }

		template <typename Iterator>
		Index Hierarchy::VertexIteratorAdaptor<Iterator>::operator*() { return Index(*it, level); }

		template <typename Iterator>
		Hierarchy::VertexIteratorAdaptor<Iterator> Hierarchy::adaptToVertexIterator(Iterator it, int level) const
		{
			return Hierarchy::VertexIteratorAdaptor<Iterator>(it, level);
		}

		template <typename Callback> // void(const Index&, float distanceSq)
		void Hierarchy::findNearestPointsRadius(const Vector3f& p, Float radius, const Callback& callback) const
		{
			if (vertexCount() == 0)
				return;

			const int searchLevel = std::max(0, (int)std::round(std::log(radius / gridSize) / std::log(2) - 1));
			const int searchLevelDiscretizer = 1 << searchLevel;
			const Float radiusSq = radius * radius;

			Int3Index searchMinIdx = ToIntIndex<3>(p - origin - Vector3f::Constant(radius), gridSize * searchLevelDiscretizer);
			Int3Index searchMaxIdx = ToIntIndex<3>(p - origin + Vector3f::Constant(radius), gridSize * searchLevelDiscretizer);
			for (int i = 0; i < 3; ++i)
			{
				if (searchMinIdx(i) * searchLevelDiscretizer < gridMin(i))
					searchMinIdx(i) = (int16_t)floor((float)gridMin(i) / searchLevelDiscretizer);
				if (searchMaxIdx(i) * searchLevelDiscretizer > gridMax(i))
					searchMaxIdx(i) = (int16_t)ceil((float)gridMax(i) / searchLevelDiscretizer);
			}
			Int3Index searchExtent = searchMaxIdx - searchMinIdx;

			std::vector<MortonCode64> inspectedCells;
			inspectedCells.reserve(20);
			MortonCode64 currentZ(searchMinIdx.x(), searchMinIdx.y(), searchMinIdx.z());
			for (int z = 0; z <= searchExtent.z(); ++z)
			{
				MortonCode64 currentY = currentZ;
				for (int y = 0; y <= searchExtent.y(); ++y)
				{
					MortonCode64 currentX = currentY;
					for (int x = 0; x <= searchExtent.x(); ++x)
					{
						inspectedCells.push_back(currentX);
						currentX += MortonCode64::UnitX;
					}
					currentY += MortonCode64::UnitY;
				}
				currentZ += MortonCode64::UnitZ;
			}
			std::sort(inspectedCells.begin(), inspectedCells.end());

			//query consecutive regions together
			auto regionBegin = inspectedCells.begin();
			auto regionEnd = regionBegin;
			while (regionBegin != inspectedCells.end())
			{
				std::vector<MortonCode64>::iterator next;
				//extend the current region as much as possible
				while (true)
				{
					next = regionEnd;
					++next;
					if (next == inspectedCells.end())
						break;
					if ((uint64_t)(*next) == (uint64_t)(*regionEnd) + 1)
						regionEnd = next;
					else
						break;
				}

				mLevels[0].shiftedSequences[0].forEachInRange((*regionBegin) << (CELL_OVER_SUBDIV + searchLevel), ((*regionEnd + 1) << (CELL_OVER_SUBDIV + searchLevel)), [&](const ShiftedRepresentation& n)
				{
					const Vector3f& pn = mLevels[0].originalData[n.vRef].position;
					float distSq = (p - pn).squaredNorm();
					if (distSq <= radiusSq)
						callback(Index(n.vRef, 0), distSq);
				});

				regionBegin = next;
				regionEnd = regionBegin;
			}
		}
	}

#undef DefineAccessors
#undef DefineAccessorsCell

	template <>
	struct HierarchyCapabilities<HierarchyMortonMultiPass::Hierarchy>
	{
		static const bool AllowAccessToAllLevels = true;
	};
}