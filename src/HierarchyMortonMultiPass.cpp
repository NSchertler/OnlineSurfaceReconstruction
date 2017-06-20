/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "HierarchyMortonMultiPass.h"
#include "HierarchyOptimizationHelper.h"

#include "Attributes.h"
#include "AttributeConsistency.h"
#include "IndentationLog.h"

#include <random>
#include <fstream>

#include <boost/iterator/transform_iterator.hpp>

using namespace osr;
using namespace HierarchyMortonMultiPass;

//#define MEASURE_REOPTIMIZATION

HierarchyMortonMultiPass::Node::Node()
	: position(Vector3f::Zero()), normal(Vector3f::UnitX()), area(0)
{
	static_assert(sizeof(Node) <= 64, "Node is larger than 64 bytes. You should disable alignment of the node buffer.");
	attribute<MeshVertex>() = INVALID;
	dirField.setConstant(std::numeric_limits<float>::quiet_NaN());
	posField.setConstant(std::numeric_limits<float>::quiet_NaN());
}

HierarchyMortonMultiPass::Node::Node(const Vector3f& position, const Vector3f& normal)
	: position(position), normal(normal), area(1)
{
	attribute<MeshVertex>() = INVALID;
	dirField.setConstant(std::numeric_limits<float>::quiet_NaN());
	posField.setConstant(std::numeric_limits<float>::quiet_NaN());
}

HierarchyMortonMultiPass::Node::Node(const Vector3f& position, const Vector3f& normal, float area)
	: position(position), normal(normal), area(area)
{
	attribute<MeshVertex>() = INVALID;
	dirField.setConstant(std::numeric_limits<float>::quiet_NaN());
	posField.setConstant(std::numeric_limits<float>::quiet_NaN());
}

HierarchyMortonMultiPass::Node::Node(const Node& copy)
			: position(copy.position), normal(copy.normal), dirField(copy.dirField), posField(copy.posField), area(copy.area), forFinestLevel(copy.forFinestLevel)
{ }

Node& HierarchyMortonMultiPass::Node::operator=(const Node& copy)
{
	memcpy(this, &copy, sizeof(Node));
	return *this;
}

Node& HierarchyMortonMultiPass::Node::operator=(Node&& move)
{
	memcpy(this, &move, sizeof(Node));
	return *this;
}

template<> typename AttributeTraits<Position>::Type& HierarchyMortonMultiPass::Node::attribute<Position>() { return position; }
template<> typename AttributeTraits<Normal>::Type& HierarchyMortonMultiPass::Node::attribute<Normal>() { return normal; }
template<> typename AttributeTraits<DirField>::Type& HierarchyMortonMultiPass::Node::attribute<DirField>() { return dirField; }
template<> typename AttributeTraits<PosField>::Type& HierarchyMortonMultiPass::Node::attribute<PosField>() { return posField; }
template<> typename AttributeTraits<Area>::Type& HierarchyMortonMultiPass::Node::attribute<Area>() { return area; }
template<> typename AttributeTraits<MeshVertex>::Type& HierarchyMortonMultiPass::Node::attribute<MeshVertex>() { return *reinterpret_cast<uint32_t*>(forFinestLevel.meshVertexData); }
template<> typename AttributeTraits<MeshVertexGeneration>::Type& HierarchyMortonMultiPass::Node::attribute<MeshVertexGeneration>() { return forFinestLevel.meshVertexData[5]; }
template<> typename AttributeTraits<Color>::Type& HierarchyMortonMultiPass::Node::attribute<Color>() { return forFinestLevel.color; }
template<> typename AttributeTraits<DirFieldConstraint>::Type& HierarchyMortonMultiPass::Node::attribute<DirFieldConstraint>() { return forCoarserLevels.directionConstraint; }

template<> typename AttributeTraits<Position>::Type& HierarchyMortonMultiPass::NodeState::avgAdded<Position>() { return avgPositionAdded; }
template<> typename AttributeTraits<Normal>::Type& HierarchyMortonMultiPass::NodeState::avgAdded<Normal>() { return avgNormalAdded; }
template<> const typename AttributeTraits<Position>::Type& HierarchyMortonMultiPass::NodeState::avgAdded<Position>() const { return avgPositionAdded; }
template<> const typename AttributeTraits<Normal>::Type& HierarchyMortonMultiPass::NodeState::avgAdded<Normal>() const { return avgNormalAdded; }
template<> typename AttributeTraits<Position>::Type& HierarchyMortonMultiPass::NodeState::avgRemoved<Position>() { return avgPositionRemoved; }
template<> typename AttributeTraits<Normal>::Type& HierarchyMortonMultiPass::NodeState::avgRemoved<Normal>() { return avgNormalRemoved; }
template<> const typename AttributeTraits<Position>::Type& HierarchyMortonMultiPass::NodeState::avgRemoved<Position>() const { return avgPositionRemoved; }
template<> const typename AttributeTraits<Normal>::Type& HierarchyMortonMultiPass::NodeState::avgRemoved<Normal>() const { return avgNormalRemoved; }

Hierarchy::Hierarchy(const Optimizer& optimizer, ExtractedMesh& extractionResult)
	: AbstractHierarchy(optimizer, extractionResult), mVertexCount(0)
{
	init();
}

Hierarchy::~Hierarchy()
{
}

void Hierarchy::reset()
{
	mLevels.clear();

	mVertexCount = 0;

	init();

	PositionsChanged();
	NormalsChanged();
	AdjacencyChanged();
	DirFieldChanged();
	PosFieldChanged();
}

void Hierarchy::init()
{
	AbstractHierarchy::init();
}

void Hierarchy::updateHierarchy(const Matrix3Xf & V, const Matrix3Xf& N, const Matrix3Xus& C, const std::vector<VertexIndex>& originalIndices, std::vector<Index>& removedVertices)
{
	const int cleanNormalsIterations = 0;
	const double targetVerticesPerCell = 15;

	bool initializeHierarchy = mVertexCount == 0;

	bbox.expand(V);

	// ---  STEP 1: Add more levels to the hierarchy if needed  ---

	if (initializeHierarchy)
	{
		//guess the grid size from the geometry		
		gridSize = bbox.diagonal().maxCoeff() / pow(meshSettings().scanSubsample * V.cols(), 0.33f) * 0.8f; //heuristic to find an appropriate grid size

		//check if this is a good number...
		TimedBlock b("Optimizing grid size ..");
		std::set<MortonCode64> allCells;
#pragma omp parallel
		{
			std::set<MortonCode64> cells;
#pragma omp for
			for (int i = 0; i < V.cols(); ++i)
			{
				Int3Index idx = ToIntIndex<3>(V.col(i), gridSize);
				MortonCode64 code(idx.x(), idx.y(), idx.z());
				cells.insert(code);
			}
#pragma omp critical
			{
				allCells.insert(cells.begin(), cells.end());
			}
		}
		double verticesPerCell = meshSettings().scanSubsample * V.cols() / allCells.size();		

		double gridSizeAdaptation = std::pow(targetVerticesPerCell / verticesPerCell, 0.33);
		gridSize *= gridSizeAdaptation;

		int totalSubdivs = std::max(0, (int)ceil(log(bbox.diagonal().maxCoeff() / gridSize) / log(2)));

		Vector3f gridExtent = Vector3f::Constant((1 << totalSubdivs) * gridSize);
		gridMin = Int3Index::Zero();
		gridMax = Int3Index::Constant((1 << totalSubdivs) - 1);

		origin = 0.5f * (bbox.min + bbox.max - gridExtent);
		mLevels.resize(totalSubdivs + 2);
	}
	else
	{
		//find how far we must extend in every direction
		Vector3f expandMax = (bbox.max - (gridSize * (gridMax + Int3Index::Ones()).cast<float>() + origin)).cwiseMax(0);
		Vector3f expandMin = ((gridSize * gridMin.cast<float>() + origin) - bbox.min).cwiseMax(0);

		const char dimNames[3] = { 'x', 'y', 'z' };
		while (!expandMax.isZero() || !expandMin.isZero())
		{
			int16_t gridWidth = (gridMax(0) + 1 - gridMin(0));

			std::cout << "Grid expansion: ";
			//find the expand directions
			int localOffsetUpdates[3] = { 0, 0, 0 }; //specifies for each dimension how the to-local offsets must be updated
			for (int dim = 0; dim < 3; ++dim)
				if (expandMin(dim) != 0 && expandMin(dim) > expandMax(dim)) //start expansion in the direction of the smaller change
				{
					localOffsetUpdates[dim] = 1;
					gridMin(dim) -= gridWidth;
					expandMin(dim) = std::max(expandMin(dim) - gridWidth * gridSize, 0.0f);
					std::cout << "-" << dimNames[dim] << " ";
				}
				else
				{
					gridMax(dim) += gridWidth;
					expandMax(dim) = std::max(expandMax(dim) - gridWidth * gridSize, 0.0f);
					std::cout << "+" << dimNames[dim] << " ";
				}
			std::cout << std::endl;

			//add shifted representations for the root node (the root level does not have those)
			auto& rootLevel = mLevels.back();
			MortonCode64 mortonCode = offsetMorton;
			for (int shift = 1; shift < SHIFTS; ++shift)
			{
				rootLevel.shiftedSequences[shift][mortonCode].vRef = 0;
				mortonCode += offsetMorton;
			}

			//add a new root
			mLevels.emplace_back();
			auto& oldRoot = mLevels.at(mLevels.size() - 2).originalData[0];
			auto rootIdx = mLevels.back().originalData.insert();
			mLevels.back().originalData[rootIdx] = Node(oldRoot); //copy root
			mLevels.back().shiftedSequences[0][MortonCode64::Zero].vRef = rootIdx;

			//update to-local offsets		
			if (localOffsetUpdates[0] + localOffsetUpdates[1] + localOffsetUpdates[2] > 0)
			{
				MortonCode64 offset(localOffsetUpdates[0], localOffsetUpdates[1], localOffsetUpdates[2]);
				for (int level = mLevels.size() - 2; level >= 0; --level)
				{
					mLevels.at(level).toLocalOffset += offset;
					if (level == 1)
						offset = offset << CELL_OVER_SUBDIV;
					else
						offset = offset << 1;
				}
			}
		}
	}

	// ---  STEP 2: Add the actual data  ---

	
	std::vector<MortonContainer<NodeState>> levelStates(mLevels.size());
	std::vector<MeshVertexType> removedMeshVertices;
	{
		TimedBlock gridConstruction("Updating grid ..", true);

		//Added vertices are new vertices and modified vertices whose Morton index changed
		std::vector<ShiftedRepresentation> addedVertices(V.cols());
		std::vector<MeshVertexType> addedVerticesMeshVertex(V.cols());

		//Modified vertices preserve their Morton index
		std::vector<std::pair<ShiftedRepresentation, size_t>> modifiedVertices(V.cols()); //pair.second references position in V and N

		//Removed vertices are deleted vertices and old modified vertices whose Morton index changed

		std::atomic<size_t> nextAddedIndex(0), nextModifiedIndex(0), nextRemovedIndex(removedVertices.size());
		removedVertices.resize(removedVertices.size() + V.cols());

		std::mt19937 rnd;
		std::uniform_real_distribution<double> rejectDist(0.0, 1.0);
		std::atomic<unsigned int> ignoredDueToCellSize(0);

		//start by calculating the morton codes
#pragma omp parallel for
		for (int i = 0; i < V.cols(); ++i)
		{
			if (std::isnan(N.col(i).x()) || std::isnan(N.col(i).y()) || std::isnan(N.col(i).z()) || std::isnan(V.col(i).x()) || std::isnan(V.col(i).y()) || std::isnan(V.col(i).z()))
				continue;

			assert(originalIndices.size() == 0 || originalIndices[i].level == 0);

			if (originalIndices.size() == 0 || originalIndices[i].idx >= mVertexCount)
			{
				if (rejectDist(rnd) >= meshSettings().scanSubsample)
					continue;
				//this is a new point

				auto code = mortonCode(V.col(i));
				if (!initializeHierarchy)
				{
					auto p = parent(code, 0);
					auto pNodeIt = mLevels[1].shiftedSequences[0].find(p);
					if (pNodeIt != mLevels[1].shiftedSequences[0].end() && mLevels[1].originalData[pNodeIt->vRef].area > 6 * targetVerticesPerCell)
					{
						ignoredDueToCellSize++;
						continue; //the cell is full - don't add more points
					}
				}

				size_t idx = nextAddedIndex++;
				addedVertices[idx].vRef = i;
				addedVertices[idx].mortonIdx = code;
				addedVerticesMeshVertex[idx] = INVALID;
			}
			else
			{
				//this is a modified point
				size_t oldIdx = originalIndices[i].idx;
				auto& v = mLevels[0].originalData[oldIdx];
				auto oldMorton = mortonCode(v.position);
				auto newMorton = mortonCode(V.col(i));
				if (oldMorton == newMorton)
				{
					size_t idx = nextModifiedIndex++;
					modifiedVertices[idx].first.vRef = oldIdx;
					modifiedVertices[idx].first.mortonIdx = newMorton;
					modifiedVertices[idx].second = i;
				}
				else
				{
					size_t idx = nextAddedIndex++;
					addedVertices[idx].vRef = i;
					addedVertices[idx].mortonIdx = newMorton;
					addedVerticesMeshVertex[idx] = v.attribute<MeshVertex>();

					idx = nextRemovedIndex++;
					removedVertices[idx].idx = oldIdx;
				}
			}
		}

		addedVertices.resize(nextAddedIndex.load());
		addedVerticesMeshVertex.resize(nextAddedIndex.load());
		modifiedVertices.resize(nextModifiedIndex.load());
		removedVertices.resize(nextRemovedIndex.load());

		for (auto& i : removedVertices)
		{
			if(mLevels[0].originalData[i.idx].attribute<MeshVertex>() != INVALID)
				removedMeshVertices.push_back(MeshVertexType(mLevels[0].originalData[i.idx].attribute<MeshVertex>(), mLevels[0].originalData[i.idx].attribute<MeshVertexGeneration>()));
		}

		std::cout << "New vertices: " << addedVertices.size() << ", modified vertices: " << modifiedVertices.size() << ", removed vertices: " << removedVertices.size() << ", ignored due to cell size: " << ignoredDueToCellSize.load() << std::endl;

		tbb::parallel_sort(addedVertices.begin(), addedVertices.end());

		mVertexCount += addedVertices.size() - removedVertices.size();

		//now, we can fill the data set
		mLevels[0].originalData.reserve(addedVertices.size() - removedVertices.size());

		for (auto i : removedVertices)
		{
			auto& v = mLevels[0].originalData[i.idx];
			auto code = mortonCode(v.position);

			auto parentIdx = parent(code, 0);
			auto& parent = levelStates[1][parentIdx];
			parent.removeData<Position, Normal>(1, v.position, v.normal);

			mLevels[0].prepareForErasure(i.idx, code);
		}
		if(removedVertices.size() > 0)
			mLevels[0].finalizeErasure();

		for (auto& i : modifiedVertices)
		{
			auto& v = mLevels[0].originalData[i.first.vRef];		

			auto parentIdx = parent(i.first.mortonIdx, 0);
			auto& parent = levelStates[1][parentIdx];
			parent.removeData<Position, Normal>(1, v.position, v.normal);

			v.position = V.col(i.second);
			v.normal = N.col(i.second);
			v.attribute<Color>() = C.col(i.second);

			parent.addData<Position, Normal>(1, v.position, v.normal);
		}

		for (int i = 0; i < addedVertices.size(); ++i)
		{
			int idx = addedVertices[i].vRef;
			auto nodeIdx = mLevels[0].originalData.insert();
			mLevels[0].originalData[nodeIdx] = Node(V.col(idx), N.col(idx));
			mLevels[0].originalData[nodeIdx].attribute<Color>() = C.col(idx);
			mLevels[0].originalData[nodeIdx].attribute<MeshVertex>() = addedVerticesMeshVertex[i].vertexIndex;
			mLevels[0].originalData[nodeIdx].attribute<MeshVertexGeneration>() = addedVerticesMeshVertex[i].generation;
			addedVertices[i].vRef = nodeIdx;

			auto parentIdx = parent(addedVertices[i].mortonIdx, 0);
			auto& parent = levelStates[1][parentIdx];
			parent.addData<Position, Normal>(1, V.col(idx), N.col(idx));
		}		
		mLevels[0].shiftedSequences[0].addSortedPoints(addedVertices.begin(), addedVertices.end(), addedVertices.rbegin(), addedVertices.rend()); //already sorted

		

		for (int shift = 1; shift < SHIFTS; ++shift)
		{
#pragma omp parallel for
			for (int i = 0; i < addedVertices.size(); ++i)
			{
				addedVertices[i].mortonIdx += offsetMorton;
			}
			tbb::parallel_sort(addedVertices.begin(), addedVertices.end());
			mLevels[0].shiftedSequences[shift].addSortedPoints(addedVertices.begin(), addedVertices.end(), addedVertices.rbegin(), addedVertices.rend());
		}
	}

	{
		TimedBlock b("Updating hierarchy ..", true);

		ApplyChangesToHierarchy<Position, Normal>(levelStates, true);
		levelStates[0] = levelStates[1];
	}

	if (initializeHierarchy)
	{
		auto& root = *mLevels.back().originalData.begin();

		//initial random solution for root
		std::mt19937 rng;
		std::uniform_real_distribution<float> distDir(0, 2 * (float)M_PI);

		Vector3f s, t;
		coordinate_system(root.normal, s, t);
		float angle = distDir(rng);
		root.dirField = s * std::cos(angle) + t * std::sin(angle);
		root.posField = root.position;
	}

	// --- STEP 2a: Clean normals
	{
		TimedBlock b("Cleaning normals ..");

		for (int i = 0; i < cleanNormalsIterations; ++i)
		{
			//Mark the the old normals for removal
			tbb::parallel_for_each(levelStates[1].begin(), levelStates[1].end(), [&](auto& nodeState)
			{
				MortonCode64 childLower, childUpper;
				this->children(nodeState.mortonIdx, 1, childLower, childUpper);

				mLevels[0].shiftedSequences[0].forEachInRange(childLower, childUpper, [&](const ShiftedRepresentation& v)
				{
					auto& node = mLevels[0].originalData[v.vRef];

					nodeState.template removeData<Normal>(1, node.normal);
				});
			});

			auto vertexSet =
				(initializeHierarchy ? Optimize<DirField>() : OptimizePart<DirField>(levelStates, false));
			vertexSet.optimizeNormals(optimizer);


			//Add the new normals
			tbb::parallel_for_each(levelStates[1].begin(), levelStates[1].end(), [&](auto& nodeState)
			{
				MortonCode64 childLower, childUpper;
				this->children(nodeState.mortonIdx, 1, childLower, childUpper);

				mLevels[0].shiftedSequences[0].forEachInRange(childLower, childUpper, [&](const ShiftedRepresentation& v)
				{
					auto& node = mLevels[0].originalData[v.vRef];

					nodeState.template addData<Normal>(1, node.normal);
				});
			});

			//Update the hierarchy
			ApplyChangesToHierarchy<Normal>(levelStates, false);
		}
	}

	// ---  STEP 3: Re-optimize where necessary  ---
	auto finestLevelVertexSet = 
		(initializeHierarchy ? Optimize<DirField, PosField>() : OptimizePart<DirField, PosField>(levelStates, true));	

	// ---  STEP 4: Extract the mesh  ---
	extractionResult.extract(finestLevelVertexSet, true, removedMeshVertices);

	std::cout << (float)mVertexCount / mLevels[1].originalData.sizeNotDeleted() << " vertices per cell in average." << std::endl;
	pointSpacing = sqrt((float)mLevels[1].originalData.sizeNotDeleted() * gridSize * gridSize / mVertexCount);

	DirFieldChanged();
	PosFieldChanged();
	PositionsChanged();
	NormalsChanged();
	AdjacencyChanged();	
}

template <Attribute ... Attributes>
void Hierarchy::ApplyChangesToHierarchy(std::vector<MortonContainer<NodeState>>& levelStates, bool storeOldAttributes)
{
	for (int i = 1; i < mLevels.size(); ++i)
	{
		auto& level = mLevels[i];
		std::vector<ShiftedRepresentation> newNodes;

		//TODO: Parallelize, bulk insertion
		for (auto stateIt = levelStates[i].begin(); stateIt != levelStates[i].end();)
		{
			auto& state = *stateIt;

			bool nodeExisted;
			auto& nodeRef = level.shiftedSequences[0].findOrCreate(state.mortonIdx, nodeExisted);
			if (!nodeExisted)
			{				
				nodeRef.vRef = level.originalData.insert();
				level.originalData[nodeRef.vRef].attribute<DirFieldConstraint>().setZero();
				newNodes.push_back(nodeRef);
			}
			auto& node = level.originalData[nodeRef.vRef];
			if (nodeExisted && storeOldAttributes)
			{
				state.dirField = node.dirField;
				state.posField = node.posField;
			}

			state.applyChangesToNode<Attributes...>(node);			

			if (i < mLevels.size() - 1)
			{
				auto parentIdx = parent(state.mortonIdx, i);
				auto& parentChange = levelStates[i + 1][parentIdx];
				parentChange.addData<Attributes...>(state);				
			}

			state.reset();

			if (node.area == 0)
			{
				level.erase(nodeRef.vRef, nodeRef.mortonIdx);
				stateIt = levelStates[i].erase(stateIt);
			}
			else
				++stateIt;
		}

		//Shift + sort
		if (newNodes.size() > 0)
		{
			for (int shift = 1; shift < SHIFTS; ++shift)
			{
#pragma omp parallel for
				for (int j = 0; j < newNodes.size(); ++j)
				{
					newNodes[j].mortonIdx += offsetMorton;
				}
				tbb::parallel_sort(newNodes.begin(), newNodes.end());
				mLevels[i].shiftedSequences[shift].addSortedPoints(newNodes.begin(), newNodes.end(), newNodes.rbegin(), newNodes.rend());
			}
		}
	}
}

std::mt19937 rng;
std::uniform_real_distribution<float> distDir(0, 2 * (float)M_PI);

namespace osr
{
	namespace HierarchyMortonMultiPass
	{
		template <>
		void Hierarchy::initializeWithParentSolution<DirField>(Node& node, Node& parentNode)
		{
			auto& nodeAttribute = node.attribute<DirField>();
			auto& parentAttribute = parentNode.attribute<DirField>();

			//if (std::isnan(nodeAttribute.x()))
			nodeAttribute = parentAttribute; //just copy if the node has no solution
		//else
			{
				//average the previous and the parent's solution
				//Vector3f compat1, compat2;
				//optimizer.meshSettings().rosy->findCompatible(nodeAttribute, node.attribute<Normal>(), parentAttribute, parentNode.attribute<Normal>(), compat1, compat2);
				//nodeAttribute = (compat1 + compat2).normalized();
			}
		}

		template <>
		void Hierarchy::initializeWithParentSolution<PosField>(Node& node, Node& parentNode)
		{
			auto& nodeAttribute = node.attribute<PosField>();
			auto& parentAttribute = parentNode.attribute<PosField>();

			//if (std::isnan(nodeAttribute.x()))
			nodeAttribute = parentAttribute; //just copy if the node has no solution
		//else
			{
				//average the previous and the parent's solution
				//Vector3f compat1, compat2;
				//optimizer.meshSettings().posy->findCompatible(node.attribute<Position>(), node.attribute<Normal>(), node.attribute<DirField>(), nodeAttribute,
				//	parentNode.attribute<Position>(), parentNode.attribute<Normal>(), parentNode.attribute<DirField>(), parentAttribute, optimizer.meshSettings().scale(), 1.0f / optimizer.meshSettings().scale(), compat1, compat2);
				//nodeAttribute = 0.8f * compat1 + 0.2f * compat2;
			}
		}
	}
}

template <Attribute ... Attributes>
PreparedVertexSet<Hierarchy, Index, true, false> Hierarchy::OptimizePart(std::vector<MortonContainer<NodeState>>& levelStates, bool checkForChildChange)
{
	const float orientationThresholdSq = 0.01f; //the squared threshold above which an orientation is considered to have changed significantly
	const float positionThreshold = 0.1f; //the squared threshold above which a position is considered to have changed significantly

	TimedBlock b("Optimizing part ..", true);

	std::vector<size_t> optVertices;
#ifdef MEASURE_REOPTIMIZATION
	std::vector<std::vector<std::pair<float, float>>> errors(levels());
#endif
	for (int i = mLevels.size() - 2; i > 0; --i)
	{
		std::cout << "Optimizing " << levelStates.at(i).size() << " nodes on level " << i << " (" << (100.0 * levelStates.at(i).size() / mLevels.at(i).originalData.sizeNotDeleted()) << " %)" << std::endl;
		optVertices.clear();
		optVertices.reserve(levelStates.at(i).size());
		for (auto& nodeState : levelStates.at(i))
		{
			//copy solution from parent
			auto p = parent(nodeState.mortonIdx, i);
			auto& pNode = mLevels.at(i + 1).node(p);
			auto nodeIt = mLevels.at(i).shiftedSequences[0].find(nodeState.mortonIdx);
			auto& node = mLevels.at(i).originalData[nodeIt->vRef];
			optVertices.push_back(nodeIt->vRef);

			int dummy[] = { 0, (
				initializeWithParentSolution<Attributes>(node, pNode),
				AttributeConsistency<Attributes>::makeConsistent(node), 0)... };
			(void)dummy; //suppress compiler warnings for unused dummy
		}

		prepareForOptimization(adaptToVertexIterator(optVertices.begin(), i), adaptToVertexIterator(optVertices.end(), i), *this).optimize<true, Attributes...>(optimizer);

		//check if the solution has changed significantly

		if (checkForChildChange)
		{
			TimedBlock b("Checking how much fields changed ..");
			if (i == 1)
			{
				levelStates[0].reserve(levelStates[1].size());
			}
			
			float positionThresholdSq = positionThreshold * (1 << (i - 1)); //coarser levels allow bigger thresholds
			positionThresholdSq *= positionThresholdSq;

			tbb::concurrent_vector<std::pair<NodeState, size_t>> insert; //node state together with the according vertex reference
			int falseHits = 0;
			for (auto& nodeState : levelStates.at(i))
			{
				if (std::isnan(nodeState.dirField.x()))
					continue; //if the node has just been created, it and all its children are already queued for re-optimization
				auto& node = mLevels.at(i).node(nodeState.mortonIdx);
				Vector3f dirCompat1, dirCompat2;
				optimizer.meshSettings().rosy->findCompatible(nodeState.dirField, node.normal, node.dirField, node.normal, dirCompat1, dirCompat2);
				float dirError = (dirCompat1 - dirCompat2).squaredNorm();
				Vector3f posCompat1, posCompat2;

				//project on the tangent plane; otherwise findCompatible does not work
				float projectLength = node.normal.dot(nodeState.posField - node.position);
				Vector3f prevPosField = nodeState.posField - node.normal * projectLength; 
				optimizer.meshSettings().posy->findCompatible(node.position, node.normal, node.dirField, prevPosField, node.position, node.normal, node.dirField, node.posField, optimizer.meshSettings().scale(), 1.0f / optimizer.meshSettings().scale(), posCompat1, posCompat2);
				float posError = ((posCompat1 - posCompat2).squaredNorm()) / (optimizer.meshSettings().scale() * optimizer.meshSettings().scale());

#ifdef MEASURE_REOPTIMIZATION
				errors[i].push_back(std::make_pair(sqrt(dirError), sqrt(posError)));
#endif
				if (dirError > orientationThresholdSq || posError > positionThresholdSq)
				{
					//the node has changed too much, record all its children
					if (i > 1)
					{
						MortonCode64 childLower, childUpper;
						children(nodeState.mortonIdx, i, childLower, childUpper);

						mLevels.at(i - 1).shiftedSequences[0].forEachInRange(childLower, childUpper, [&](const ShiftedRepresentation& r)
						{
							if (parent(r.mortonIdx, i - 1) == nodeState.mortonIdx)
								insert.push_back(std::make_pair(NodeState(r.mortonIdx), r.vRef));
							else
								++falseHits;
						});
					}
					else
					{
						insert.push_back(std::make_pair(NodeState(nodeState.mortonIdx), -1));
						//levelStates[0][nodeState.mortonIdx]; //just create the node, we don't care for its content.
					}
				}
			}			
			std::cout << falseHits << " false hits." << std::endl;

			tbb::parallel_sort(insert.begin(), insert.end());

			std::vector<std::pair<MortonContainer<NodeState>::iterator, bool>> insertedNodes;

			auto mapToNode = [](const std::pair<NodeState, size_t>& pair) { return pair.first; };
			auto begin = boost::make_transform_iterator(insert.begin(), mapToNode);
			auto end = boost::make_transform_iterator(insert.end(), mapToNode);

			levelStates[i - 1].findOrCreate(begin, end, insertedNodes);
			if (i > 1)
			{
				for (int j = 0; j < insertedNodes.size(); ++j)
				{
					auto& node = insertedNodes[j];
					if (!node.second)
					{
						node.first->dirField = mLevels[i - 1].originalData[insertedNodes[j].second].dirField;
						node.first->posField = mLevels[i - 1].originalData[insertedNodes[j].second].posField;
					}
				}
			}

			insert.clear();
		}
	}
#ifdef MEASURE_REOPTIMIZATION
	std::ofstream errorFile("optError.csv");
	size_t maxSamples = 0;
	for (auto& v : errors)
		maxSamples = std::max(maxSamples, v.size());
	for (int l = levels() - 1; l >= 0; --l)
		errorFile << "Direction Level " << l << ",";
	for (int l = levels() - 1; l >= 0; --l)
		errorFile << "Position Level " << l << ",";
	errorFile << std::endl;
	for (int i = 0; i < maxSamples; ++i)
	{
		for (int l = levels() - 1; l >= 0; --l)
		{
			if (errors[l].size() > i)
				errorFile << errors[l][i].first;
			errorFile << ",";
		}
		for (int l = levels() - 1; l >= 0; --l)
		{
			if (errors[l].size() > i)
				errorFile << errors[l][i].second;
			errorFile << ",";
		}
		errorFile << std::endl;
	}
	errorFile.close();
#endif
	//finally, re-optimize the vertices in the cells
	optVertices.clear();
	std::cout << "Optimizing " << levelStates.at(0).size() << " cells on level 0 (" << (100.0 * levelStates.at(0).size() / mLevels.at(1).originalData.sizeNotDeleted()) << " %)" << std::endl;
	for (auto& nodeState : levelStates.at(0))
	{
		auto& pNode = mLevels.at(1).originalData[mLevels.at(1).shiftedSequences[0].find(nodeState.mortonIdx)->vRef];

		MortonCode64 childLower, childUpper;
		children(nodeState.mortonIdx, 1, childLower, childUpper);

		mLevels[0].shiftedSequences[0].forEachInRange(childLower, childUpper, [&](const ShiftedRepresentation& v)
		{
			auto& node = mLevels[0].originalData[v.vRef];

			//copy solution from parent
			int dummy[] = { 0, (
				node.attribute<Attributes>() = pNode.attribute<Attributes>(),
				AttributeConsistency<Attributes>::makeConsistent(node), 0)... };
			(void)dummy; //suppress compiler warnings for unused dummy

			optVertices.push_back(v.vRef);
		});
	}
	optimizedPoints = optVertices.size();

	auto vertexSet = prepareForOptimizationWithIntrinsicNeighborIndices(adaptToVertexIterator(optVertices.begin(), 0), adaptToVertexIterator(optVertices.end(), 0), *this);
	vertexSet.optimize<false, Attributes...>(optimizer);

	return vertexSet;
}

template<Attribute ... Attributes>
PreparedVertexSet<Hierarchy, Index, true, false> Hierarchy::Optimize()
{
	TimedBlock b("Optimizing full..");

	for (int level = mLevels.size() - 2; level > 0; --level)
	{
		//copy from coarser level
		auto& nodes = mLevels[level].originalData;
		tbb::parallel_for_each(nodes.begin(), nodes.end(), [&](Node& v)
		{
			auto p = parent(mortonCode(v.position, level), level);
			auto& parentNode = mLevels[level + 1].node(p);

			//copy solution from parent
			int dummy[] = { 0, (
				initializeWithParentSolution<Attributes>(v, parentNode),
				AttributeConsistency<Attributes>::makeConsistent(v), 0)... };
			(void)dummy; //suppress compiler warnings for unused dummy
		});

		//optimize
		auto vertexSet = prepareForOptimization(VertexIterator(mLevels[level].originalData.begin(), level), VertexIterator(mLevels[level].originalData.end(), level), *this);
		vertexSet.optimize<true, DirField, PosField>(optimizer);
	}

	//copy from coarser level
	auto& nodes = mLevels[0].originalData;
	tbb::parallel_for_each(nodes.begin(), nodes.end(), [&](Node& v)
	{
		auto p = parent(mortonCode(v.position), 0);
		auto& parentNode = mLevels[1].node(p);

		//copy solution from parent
		int dummy[] = { 0, (
			v.attribute<Attributes>() = parentNode.attribute<Attributes>(),
			AttributeConsistency<Attributes>::makeConsistent(v), 0)... };
		(void)dummy; //suppress compiler warnings for unused dummy
	});

	optimizedPoints = mLevels[0].originalData.sizeNotDeleted();

	//optimize
	auto vertexSet = prepareForOptimizationWithIntrinsicNeighborIndices(VertexIterator(mLevels[0].originalData.begin(), 0), VertexIterator(mLevels[0].originalData.end(), 0), *this);
	vertexSet.optimize<false, DirField, PosField>(optimizer);	

	DirFieldChanged();
	PosFieldChanged();

	return vertexSet;
}

void Hierarchy::addPoints(const Matrix3Xf & V, const Matrix3Xf & N, const Matrix3Xus& C)
{
	std::vector<Index> removed;
	updateHierarchy(V, N, C, std::vector<VertexIndex>(), removed);
}

void Hierarchy::modifyPoints(const std::vector<VertexIndex>& points, const Matrix3Xf& newV, const Matrix3Xf& newN, const Matrix3Xus& newC)
{
	std::vector<Index> removed;
	updateHierarchy(newV, newN, newC, points, removed);
}

void Hierarchy::removePoints(std::vector<VertexIndex>& points)
{
	Matrix3Xf V, N;
	Matrix3Xus C;
	updateHierarchy(V, N, C, std::vector<Index>(), points);
}

void Hierarchy::optimizeFull()
{
	auto vertexSet = Optimize<DirField, PosField>();
	extractionResult.reset();
	extractionResult.extract(vertexSet);
}

size_t Hierarchy::sizeInBytes() const
{
	size_t size = sizeof(*this);

	for (auto& level : mLevels)
		size += level.sizeInBytes();

	return size;
}

int HierarchyMortonMultiPass::Hierarchy::levels() const
{
	return mLevels.size();
}


size_t Hierarchy::findClosestCompatiblePoint(const Vector3f& p, const Vector3f& normal) const
{
	if (vertexCount() == 0)
		throw std::runtime_error("Cannot find the closest point in an empty hierarchy.");

	const int steps = 5;

	float radius = meshSettings().maxRegistrationError() / (1 << steps);
	float minDistanceSq = std::numeric_limits<float>::infinity();
	size_t closestPoint = -1;

	for(int i = 0; i < steps; ++i)
	{
		findNearestPointsRadius(p, radius, [&](const Index& n, float distanceSq)
		{
			if (distanceSq < minDistanceSq && normal.dot(mLevels[0].originalData[n.idx].normal) >= 0.7f)
			{
				closestPoint = n.idx;
				minDistanceSq = distanceSq;
			}
		});
		if (minDistanceSq < std::numeric_limits<float>::infinity())
			return closestPoint;
		radius *= 2;
	}
	return -1;
}

bool Hierarchy::isIndexValid(const size_t& idx) const { return idx != (size_t)-1; }

Vector3f Hierarchy::neighborP(const size_t & i) const
{
	return mLevels[0].originalData[i].position;
}

Vector3f Hierarchy::neighborN(const size_t & i) const
{
	return mLevels[0].originalData[i].normal;
}

ForEachHelper<Hierarchy::VertexIterator> Hierarchy::vertices(int level)
{
	return ForEachHelper<VertexIterator>(VertexIterator(mLevels[0].originalData.begin(), level), VertexIterator(mLevels[level].originalData.end(), level));
}

MortonCode64 Hierarchy::parent(MortonCode64 idx, int childNodeLevel, int levelsUp) const
{
	MortonCode64 localIdx = idx + mLevels[childNodeLevel].toLocalOffset;

	int offset = levelsUp;
	if (childNodeLevel == 0)
		offset += CELL_OVER_SUBDIV - 1;

	localIdx = localIdx >> offset;	

	return localIdx - mLevels[childNodeLevel + levelsUp].toLocalOffset;	
}

void HierarchyMortonMultiPass::Hierarchy::children(const MortonCode64 idx, int parentNodeLevel, MortonCode64 & childLowerInclusive, MortonCode64 & childUpperExclusive, int levelsDown) const
{
	MortonCode64 localIdx = idx + mLevels[parentNodeLevel].toLocalOffset;

	int offset = levelsDown;
	if (parentNodeLevel - levelsDown == 0)
		offset += CELL_OVER_SUBDIV - 1;

	childLowerInclusive = localIdx << offset;
	childUpperExclusive = (localIdx + 1) << offset;

	childLowerInclusive = childLowerInclusive - mLevels[parentNodeLevel - levelsDown].toLocalOffset;
	childUpperExclusive = childUpperExclusive - mLevels[parentNodeLevel - levelsDown].toLocalOffset;
}

MortonCode64 Hierarchy::mortonCode(const Vector3f& p, int level) const
{
	Int3Index idx = ToIntIndex<3>(p - origin, gridSize * ldexpf(1, -CELL_OVER_SUBDIV)); // ... * 2^(-CELL_OVER_SUBDIV));
	auto code = MortonCode64(idx.x(), idx.y(), idx.z());
	if (level == 0)
		return code;
	else
		return parent(code, 0, level);
}

template<>
void osr::saveToFile(const LevelInfo& level, FILE* f)
{
	for (int i = 0; i < SHIFTS; ++i)
		level.shiftedSequences[i].saveToFile(f);
	level.originalData.saveToFile(f);
	osr::saveToFile(level.toLocalOffset, f);
}

template<>
void osr::loadFromFile(LevelInfo& level, FILE* f)
{
	for (int i = 0; i < SHIFTS; ++i)
		level.shiftedSequences[i].loadFromFile(f);
	level.originalData.loadFromFile(f);
	osr::loadFromFile(level.toLocalOffset, f);
}

void Hierarchy::saveToFile(FILE* f) const
{
	::saveToFile(bbox, f);
	::saveToFile(maxNeighborRadius, f);
	::saveToFile(origin, f);
	::saveToFile(gridMin, f);
	::saveToFile(gridMax, f);
	::saveToFile(gridSize, f);
	::saveToFile(mLevels, f);
	::saveToFile(mVertexCount, f);
	::saveToFile(pointSpacing, f);
}

void Hierarchy::loadFromFile(FILE* f)
{
	::loadFromFile(bbox, f);
	::loadFromFile(maxNeighborRadius, f);
	::loadFromFile(origin, f);
	::loadFromFile(gridMin, f);
	::loadFromFile(gridMax, f);
	::loadFromFile(gridSize, f);
	::loadFromFile(mLevels, f);
	::loadFromFile(mVertexCount, f);
	::loadFromFile(pointSpacing, f);
}

// -------------  VertexIterator  ---------------

Hierarchy::VertexIterator::VertexIterator(LevelInfo::DataContainer::iterator it, int level)
	: it(it), level(level)
{ }

const Hierarchy::VertexIterator& Hierarchy::VertexIterator::operator++()
{
	++it;
	return *this;
}

bool Hierarchy::VertexIterator::operator!=(VertexIterator& other) const
{
	return it != other.it;
}

Index Hierarchy::VertexIterator::operator*()
{
	return Index(it.index(), level);
}
