#include "osr/HierarchyRegularSubdiv.h"

#include <random>
#include <algorithm>
#include "osr/AttributeConsistency.h"
#include "osr/Optimizer.h"
#include <nsessentials/util/IndentationLog.h>

#include "osr/HierarchyOptimizationHelper.h"
#include <fstream>

//#define MEASURE_REOPTIMIZATION

template <>
struct IndexOptimizationTraits<HierarchyRegularSubdiv::InnerIndex, HierarchyRegularSubdiv::Hierarchy>
{
	static const bool HasImplicitPhases = true;

	static const bool StoreNeighbors = false;
};

using namespace HierarchyRegularSubdiv;

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
	grid.clear();
	innerLevels.clear();
	subdivs.clear();

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
	subdivCount.setZero();
}

size_t Hierarchy::phase(const InnerIndex& idx) const
{
	unsigned xMod2 = idx.node.x() - 2 * floor(idx.node.x() / 2.0);
	unsigned yMod2 = idx.node.y() - 2 * floor(idx.node.y() / 2.0);
	unsigned zMod2 = idx.node.z() - 2 * floor(idx.node.z() / 2.0);
	return xMod2 + 2 * yMod2 + 4 * zMod2;
}

void Hierarchy::initialize(const Matrix3Xf & V, const Matrix3Xf & N)
{
	mVertexCount = V.cols();

	//initialize the structure
	bbox.expand(V);

	gridSize = bbox.diagonal().maxCoeff() / pow(V.cols(), 0.33f) * 0.8f;

	Vector3f gridExtent;
	for (int i = 0; i < 3; ++i)
	{
		subdivCount(i) = std::max(0, (int)ceil(log(bbox.diagonal()(i) / gridSize) / log(2)));
		gridExtent(i) = (1 << subdivCount(i)) * gridSize;
		gridMin(i) = 0;
		gridMax(i) = (1 << subdivCount(i)) - 1;
	}

	origin = 0.5f * (bbox.min + bbox.max - gridExtent);

	int totalSubdivs = subdivCount.sum();

	innerLevels.resize(totalSubdivs + 1);
	std::cout << "Subdivision order (coarse to fine): ";
	//construct the subdivision order
	char dims[3] = { 'x', 'y', 'z' };
	for (int i = 0; i < totalSubdivs; ++i)
	{
		//divide along the longest axis
		int subDivDim;
		if (gridExtent(0) > gridExtent(1) && gridExtent(0) > gridExtent(2))
			subDivDim = 0;
		else if (gridExtent(1) > gridExtent(2))
			subDivDim = 1;
		else
			subDivDim = 2;
		subdivs.emplace_front(subDivDim);
		gridExtent(subDivDim) /= 2;
		std::cout << dims[subDivDim] << " ";
	}
	std::cout << std::endl;

	//construct grid cells and fill them with data
	{
		nse::util::TimedBlock gridConstruction("Constructing grid ..");
		for (int i = 0; i < V.cols(); ++i)
		{
			if (std::isnan(N.col(i).x()) || std::isnan(N.col(i).y()) || std::isnan(N.col(i).z()))
			{
				//invalid normal
				mVertexCount--;
				continue;
			}

			auto idx = ToIntIndex<3>(V.col(i) - origin, gridSize);
			auto& cell = grid[idx];
			cell.position.push_back(V.col(i));
			cell.normal.push_back(N.col(i));

			//filter to the next coarser level
			if (totalSubdivs > 0)
			{
				auto parentIdx = idx;
				auto& parent = innerLevels[0].nodes[parentIdx];
				parent.area += 1;
				parent.position += 1.0f / parent.area * (V.col(i) - parent.position);
				parent.normal += 1.0f / parent.area * (N.col(i) - parent.normal);;
			}
		}
		if (V.cols() > mVertexCount)
			std::cout << "Ignored " << (V.cols() - mVertexCount) << " points due to invalid normals." << std::endl;
		for (auto& cell : grid)
		{
			cell.second.dirField.resize(cell.second.position.size());
			cell.second.posField.resize(cell.second.position.size());
			cell.second.meshVertex.resize(cell.second.position.size(), -1);
		}
	}
	{
		nse::util::TimedBlock hierarchyConstruction("Constructing hierarchy ..");
		//filter up to coarser levels
		for (int nodeLevel = 0; nodeLevel < innerLevels.size() - 1; ++nodeLevel)
		{
			for (auto& node : innerLevels.at(nodeLevel).nodes)
			{
				if (node.second.normal.squaredNorm() >= 0.001f)
					node.second.normal.normalize();
				else
					node.second.normal = Vector3f::UnitX();

				auto parentIdx = parent<3>(node.first, nodeLevel);
				auto& parent = innerLevels.at(nodeLevel + 1).nodes[parentIdx];

				parent.area += node.second.area;
				parent.position += node.second.area / parent.area * (node.second.position - parent.position);
				parent.normal += node.second.area / parent.area * (node.second.normal - parent.normal);
			}
		}

		auto& root = *innerLevels.back().nodes.begin();
		if (root.second.normal.squaredNorm() >= 0.001f)
			root.second.normal.normalize();
		else
			root.second.normal = Vector3f::UnitX();

		//initial random solution for root
		std::mt19937 rng;
		std::uniform_real_distribution<float> distDir(0, 2 * (float)M_PI);

		Vector3f s, t;
		coordinate_system(root.second.normal, s, t);
		float angle = distDir(rng);
		root.second.dirField = s * std::cos(angle) + t * std::sin(angle);
		root.second.posField = root.second.position;
	}

	optimizeFull();
}

void Hierarchy::updateHierarchy(const Matrix3Xf& V, const Matrix3Xf N)
{
	mVertexCount += V.cols();

	bbox.expand(V);

	// ---  STEP 1: Add more levels to the hierarchy if needed  ---

	//find how far we must extend in every direction
	Vector3f expandMax = (bbox.max - (gridSize * (gridMax + Int3Index::Ones()).cast<float>() + origin)).cwiseMax(0);
	Vector3f expandMin = ((gridSize * gridMin.cast<float>() + origin) - bbox.min).cwiseMax(0);

	while (!expandMax.isZero() || !expandMin.isZero())
	{
		//order the subdivision dimensions by number of subdivisions in order to prefer quadratic cells
		uint8_t subdivDimensionPrecedence[3] = { 0, 1, 2 };
		std::sort(subdivDimensionPrecedence, subdivDimensionPrecedence + 3, [this](uint8_t first, uint8_t second) { return subdivCount(first) < subdivCount(second); });

		for (uint8_t dim : subdivDimensionPrecedence)
		{
			if (expandMax(dim) != 0 || expandMin(dim) != 0)
			{
				//subdivide in the current dimension

				++subdivCount(dim);
				subdivs.emplace_back(dim);

				float gridWidth = (gridMax(dim) + 1 - gridMin(dim));

				if (expandMin(dim) > expandMax(dim))
				{
					//expand in the negative direction, update the level offsets
					int offset = 1;
					for (int level = innerLevels.size() - 2; level >= 0; --level)
					{
						innerLevels.at(level).toLocalOffset(dim) += offset;

						if (level > 0 && subdivs[level - 1].dimension == dim)
							offset = offset << 1;
					}

					gridMin(dim) -= gridWidth;
					expandMin(dim) = std::max(expandMin(dim) - gridWidth * gridSize, 0.0f);

					std::cout << "Expanding in negative direction of dimension " << (int)dim << std::endl;
				}
				else
				{
					gridMax(dim) += gridWidth;
					expandMax(dim) = std::max(expandMax(dim) - gridWidth * gridSize, 0.0f);
					std::cout << "Expanding in positive direction of dimension " << (int)dim << std::endl;
				}

				Int3Index idx(0, 0, 0);
				auto root = innerLevels.back().nodes.begin()->second;
				innerLevels.emplace_back(); innerLevels.back().nodes[idx] = Node(root);

				break;
			}
		}
	}

	std::cout << "New subdivision order (coarse to fine): ";
	char dims[3] = { 'x', 'y', 'z' };
	for (auto dimIt = subdivs.rbegin(); dimIt != subdivs.rend(); ++dimIt)
		std::cout << dims[dimIt->dimension] << " ";
	std::cout << std::endl;

	// ---  STEP 2: Add the actual data  ---

	struct NodeAddition
	{
		NodeAddition()
			: position(Vector3f::Zero()), normal(Vector3f::Zero()), area(0)
		{ }

		Vector3f position;
		Vector3f normal;
		float area;
	};
	struct OldNodeState
	{
		OldNodeState()
			: posField(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()),
			dirField(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN())
		{ }

		OldNodeState(Vector3f dir, Vector3f pos)
			: posField(pos), dirField(dir)
		{ }

		Vector3f posField, dirField;
	};
	std::unordered_map<Int3Index, NodeAddition> parentLevelChange;
	std::vector<std::unordered_map<Int3Index, OldNodeState>> oldNodeStates(innerLevels.size()); //saves the contents of the nodes before re-optimization
	{
		nse::util::TimedBlock gridUpdate("Updating grid..");
		int ignored = 0;
		for (int i = 0; i < V.cols(); ++i)
		{
			if (std::isnan(N.col(i).x()) || std::isnan(N.col(i).y()) || std::isnan(N.col(i).z()))
			{
				//invalid normal
				++ignored;
				continue;
			}

			auto idx = ToIntIndex<3>(V.col(i) - origin, gridSize);
			auto& cell = grid[idx];
			cell.position.push_back(V.col(i));
			cell.normal.push_back(N.col(i));

			cell.dirField.emplace_back();
			cell.posField.emplace_back();
			cell.meshVertex.push_back((uint32_t)-1);

			//filter to the next coarser level			
			auto parentIdx = idx;
			auto& parent = parentLevelChange[parentIdx];
			parent.area += 1;
			parent.position += 1.0f / parent.area * (V.col(i) - parent.position);
			parent.normal += 1.0f / parent.area * (N.col(i) - parent.normal);
		}
		mVertexCount -= ignored;
		if (ignored > 0)
			std::cout << "Ignored " << ignored << " points due to invalid normals." << std::endl;
	}

	//filter up the hierarchy
	{
		nse::util::TimedBlock hierarchyUpdate("Updating hierarchy ..");
		std::unordered_map<Int3Index, NodeAddition> currentLevelChange;
		for (int i = 0; i < innerLevels.size(); ++i)
		{
			auto& level = innerLevels.at(i);

			std::swap(currentLevelChange, parentLevelChange);
			parentLevelChange.clear();

			for (auto& change : currentLevelChange)
			{
				auto node = level.nodes.find(change.first);
				auto& oldState = oldNodeStates.at(i)[change.first];
				if (node == level.nodes.end())
				{
					//create a new node if it does not exist
					node = level.nodes.emplace(change.first, Node()).first;
				}
				else
				{
					oldState.dirField = node->second.dirField;
					oldState.posField = node->second.posField;
				}

				node->second.area += change.second.area;
				node->second.position += change.second.area / node->second.area * (change.second.position - node->second.position);
				Vector3f prevNormal = node->second.normal;
				node->second.normal += change.second.area / node->second.area * (change.second.normal - node->second.normal);
				if (node->second.normal.squaredNorm() <= 0.001f)
					node->second.normal = prevNormal;
				else
					node->second.normal.normalize();

				if (i < innerLevels.size() - 1)
				{
					auto parentIdx = parent<3>(node->first, i);
					auto& parentChange = parentLevelChange[parentIdx];
					parentChange.area += change.second.area;
					parentChange.position += change.second.area / parentChange.area * (change.second.position - parentChange.position);
					parentChange.normal += change.second.area / parentChange.area * (change.second.normal - parentChange.normal);
				}
			}
		}
	}

	// ---  STEP 3: Re-optimize where necessary  ---

	{
		nse::util::TimedBlock reopt("Re-optimizing ..");
		const float orientationThresholdSq = 0.01f; //the squared threshold above which an orientation is considered to have changed significantly
		const float positionThresholdSq = 0.01f; //the squared threshold above which a position is considered to have changed significantly
#ifdef MEASURE_REOPTIMIZATION
		std::vector<std::vector<std::pair<float, float>>> errors(innerLevels.size());
#endif
		for (int i = innerLevels.size() - 2; i >= 0; --i)
		{
			std::cout << "Optimizing " << oldNodeStates.at(i).size() << " nodes on level " << i << " (" << (100.0 * oldNodeStates.at(i).size() / innerLevels.at(i).nodes.size()) << " %)" << std::endl;
			//construct the optimization phases
			for (auto& nodeState : oldNodeStates.at(i))
			{
				//copy solution from parent
				auto p = parent<3>(nodeState.first, i);
				auto& pNode = innerLevels.at(i + 1).nodes.at(p);
				auto& node = innerLevels.at(i).nodes.at(nodeState.first);
				node.dirField = pNode.dirField;
				node.posField = pNode.posField;
				AttributeConsistency<DirField>::makeConsistent(node);
				AttributeConsistency<PosField>::makeConsistent(node);
			}

			prepareForOptimization(adaptToInnerIterator(oldNodeStates.at(i).begin(), i), adaptToInnerIterator(oldNodeStates.at(i).end(), i), *this).optimize(optimizer);			
			
			//check if the solution has changed significantly
			if (i > 0)
			{
				for (auto& nodeState : oldNodeStates.at(i))
				{
					if (std::isnan(nodeState.second.dirField.x()))
						continue; //if the node has just been created, it and all its children are already queued for re-optimization
					auto& node = innerLevels.at(i).nodes.at(nodeState.first);
					Vector3f dirCompat1, dirCompat2;
					optimizer.meshSettings().rosy->findCompatible(nodeState.second.dirField, node.normal, node.dirField, node.normal, dirCompat1, dirCompat2);
					float dirError = (dirCompat1 - dirCompat2).squaredNorm();
					Vector3f posCompat1, posCompat2;
					optimizer.meshSettings().posy->findCompatible(node.position, node.normal, node.dirField, nodeState.second.posField, node.position, node.normal, node.dirField, node.posField, optimizer.meshSettings().scale, 1.0f / optimizer.meshSettings().scale, posCompat1, posCompat2);
					float posError = (posCompat1 - posCompat2).squaredNorm() / (optimizer.meshSettings().scale * optimizer.meshSettings().scale);
#ifdef MEASURE_REOPTIMIZATION
					errors[i].push_back(std::make_pair(sqrt(dirError), sqrt(posError)));
#endif
					if (dirError > orientationThresholdSq || posError > positionThresholdSq)
					{
						//the node has changed too much, record all its children
						Int3Index child1, child2;
						children<3>(nodeState.first, i, child1, child2);
						auto child1StateIt = oldNodeStates.at(i - 1).find(child1);
						auto child1NodeIt = innerLevels.at(i - 1).nodes.find(child1);
						if (child1NodeIt != innerLevels.at(i - 1).nodes.end() && child1StateIt == oldNodeStates.at(i - 1).end())
							oldNodeStates.at(i - 1)[child1] = OldNodeState(child1NodeIt->second.dirField, child1NodeIt->second.posField);
						auto child2StateIt = oldNodeStates.at(i - 1).find(child2);
						auto child2NodeIt = innerLevels.at(i - 1).nodes.find(child2);
						if (child2NodeIt != innerLevels.at(i - 1).nodes.end() && child2StateIt == oldNodeStates.at(i - 1).end())
							oldNodeStates.at(i - 1)[child2] = OldNodeState(child2NodeIt->second.dirField, child2NodeIt->second.posField);
					}
				}
			}
		}
#ifdef MEASURE_REOPTIMIZATION
		std::ofstream errorFile("optError.csv");
		size_t maxSamples = 0;
		for (auto& v : errors)
			maxSamples = std::max(maxSamples, v.size());
		for (int l = errors.size() - 1; l >= 0; --l)
			errorFile << "Direction Level " << l << ",";
		for (int l = errors.size() - 1; l >= 0; --l)
			errorFile << "Position Level " << l << ",";
		errorFile << std::endl;
		for (int i = 0; i < maxSamples; ++i)
		{
			for (int l = errors.size() - 1; l >= 0; --l)
			{
				if (errors[l].size() > i)
					errorFile << errors[l][i].first;
				errorFile << ",";
			}
			for (int l = errors.size() - 1; l >= 0; --l)
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
		//construct the optimization phases
		for (auto& nodeState : oldNodeStates.at(0))
		{
			//copy solution from parent
			auto& pNode = innerLevels.at(0).nodes.at(nodeState.first);
			auto& cell = grid.at(nodeState.first);
			for (int i = 0; i < cell.position.size(); ++i)
			{
				cell.dirField[i] = pNode.dirField;
				cell.posField[i] = pNode.posField;

				AttributeConsistency<DirField>::makeConsistent(SpecificCellAttributeAccess(cell, i));
				AttributeConsistency<PosField>::makeConsistent(SpecificCellAttributeAccess(cell, i));
			}
		}

		auto vertexSet = prepareForOptimizationWithIntrinsicNeighborIndices(adaptToFinestIterator(oldNodeStates.at(0).begin()), adaptToFinestIterator(oldNodeStates.at(0).end()), *this);
		vertexSet.optimize(optimizer);
		reopt.earlyExit();
		
		extractionResult.extract(vertexSet, true);
	}

	DirFieldChanged();
	PosFieldChanged();
}

void Hierarchy::addPoints(const Matrix3Xf & V, const Matrix3Xf & N)
{
	if (mVertexCount == 0)
	{		
		initialize(V, N);
	}
	else
	{
		updateHierarchy(V, N);
	}

	std::cout << ((float)mVertexCount / grid.size()) << " vertices per cell in average." << std::endl;

	PositionsChanged();
	NormalsChanged();	
	AdjacencyChanged();
}

void Hierarchy::optimizeFull()
{
	nse::util::TimedBlock b("Optimizing full ..");
	//the root level is a single node and won't change in optimization
	copyToFinerLevel<DirField>(innerLevels.size());
	copyToFinerLevel<PosField>(innerLevels.size());
	for (int iLevel = innerLevels.size() - 2; iLevel >= 0; --iLevel) 
	{
		prepareForOptimization(adaptToInnerIterator(innerLevels[iLevel].nodes.begin(), iLevel), adaptToInnerIterator(innerLevels[iLevel].nodes.end(), iLevel), *this).optimize(optimizer);

		copyToFinerLevel<DirField>(iLevel + 1);
		copyToFinerLevel<PosField>(iLevel + 1);
	}
	auto v = vertices();
	auto vertexSet = prepareForOptimizationWithIntrinsicNeighborIndices(v.begin(), v.end(), *this);
	vertexSet.optimize(optimizer);

	b.earlyExit();
	extractionResult.extract(vertexSet);

	DirFieldChanged();
	PosFieldChanged();
}

ForEachHelper<Hierarchy::VertexFinestIterator> Hierarchy::vertices() const
{
	return ForEachHelper<VertexFinestIterator>(VertexFinestIterator(0, grid.cbegin()), VertexFinestIterator(0, grid.cend()));
}

void Hierarchy::findNearestPointsKNN(const Vector3f & p, int k, std::priority_queue<Neighbor<FinestIndex>>& neighborQueue, Float maxRadiusSq) const
{	
	//neighborQueue -> neighbor with greatest distance is on top
		
	auto pLocal = p - origin;
	Int3Index pCellIdx = ToIntIndex<3>(pLocal, gridSize);

	Float gridSizeSq = gridSize * gridSize;

	std::priority_queue<Neighbor<Int3Index>, std::vector<Neighbor<Int3Index>>, std::greater<Neighbor<Int3Index>>> cellQueue; //neighbor with smallest distance is on top
	cellQueue.emplace(pCellIdx, 0);

	while (!cellQueue.empty())
	{										
		auto cellEntry = cellQueue.top();
		cellQueue.pop();
		if (neighborQueue.size() == k && (cellEntry.distanceSq > neighborQueue.top().distanceSq || cellEntry.distanceSq > maxRadiusSq))
			break; //no more improvement possible
			
		auto& cellIdx = cellEntry.idx;

		//check if cell is within bounds
		if (cellIdx(0) < gridMin(0) || cellIdx(0) > gridMax(0) ||
			cellIdx(1) < gridMin(1) || cellIdx(1) > gridMax(1) ||
			cellIdx(2) < gridMin(2) || cellIdx(2) > gridMax(2))
			continue;

		auto& cell = grid.find(cellIdx);
		if (cell != grid.end())
		{
			for (int i = 0; i < cell->second.position.size(); ++i)
			{
				Float distSq = (cell->second.attribute<Position>(i) - p).squaredNorm();
				if (distSq > maxRadiusSq)
					continue;
				if (neighborQueue.size() < k)
					neighborQueue.emplace(FinestIndex(cellIdx, i), distSq);
				else if (distSq < neighborQueue.top().distanceSq)
				{
					neighborQueue.pop();
					neighborQueue.emplace(FinestIndex(cellIdx, i), distSq);
				}
			}
		}
			
		//propagate cell outwards
		Float maxDist = maxRadiusSq; //the maximum distance of valid cells
		if (neighborQueue.size() == k)
			maxDist = neighborQueue.top().distanceSq;

		Int3Index relIndex = cellIdx - pCellIdx;
		int ring = relIndex.cwiseAbs().maxCoeff();
		if (ring == 0)
		{
			//initialize with one-ring
			Vector3f diffsSq[3];
			for (int i = 0; i < 3; ++i)
			{
				diffsSq[0](i) = pLocal(i) - pCellIdx(i) * gridSize;
				diffsSq[0](i) *= diffsSq[0](i);
				diffsSq[1](i) = 0;
				diffsSq[2](i) = (pCellIdx(i) + 1) * gridSize - pLocal(i);
				diffsSq[2](i) *= diffsSq[2](i);
			}
				
			for (int x = -1; x <= 1; ++x)
				for (int y = -1; y <= 1; ++y)
					for (int z = -1; z <= 1; ++z)
					{
						if (x == 0 && y == 0 && z == 0)
							continue;
							
						Float distance = diffsSq[x + 1](0) + diffsSq[y + 1](1) + diffsSq[z + 1](2);
						if (distance <= maxDist)
							cellQueue.emplace(Int3Index(pCellIdx(0) + x, pCellIdx(1) + y, pCellIdx(2) + z), distance);
					}				
		}
		else
		{
			Eigen::Matrix<int16_t, 3, 1> offset;
			for (int i = 0; i < 3; ++i)
			{
				if (abs(relIndex(i)) == ring)
					offset(i) = signum(relIndex(i));
				else
					offset(i) = 0;
			}
			for (int x = 0; x <= abs(offset.x()); ++x)
				for (int y = 0; y <= abs(offset.y()); ++y)
					for (int z = 0; z <= abs(offset.z()); ++z)
					{
						if (x == 0 && y == 0 && z == 0)
							continue;

						Float distance = cellEntry.distanceSq + (x + y + z) * gridSizeSq;
						if (distance <= maxDist)
							cellQueue.emplace(cellIdx + offset.cwiseProduct(Int3Index(x, y, z)), distance);
					}

		}
	}
}

void Hierarchy::findNearestPointsKNN(const Vector3f & p, int k, std::vector<FinestIndex>& output, Float maxRadiusSq) const
{
	std::priority_queue<Neighbor<FinestIndex>> neighborQueue;
	findNearestPointsKNN(p, k, neighborQueue, maxRadiusSq);
	while (!neighborQueue.empty())
	{
		output.push_back(std::move(neighborQueue.top().idx));
		neighborQueue.pop();
	}
}

FinestIndex Hierarchy::findClosestPoint(const Vector3f & p) const
{
	std::priority_queue<Neighbor<FinestIndex>> neighborQueue;
	findNearestPointsKNN(p, 1, neighborQueue);
	return neighborQueue.top().idx;
}

const Vector3f& Hierarchy::p(const FinestIndex& i) const
{
	return grid.at(i.node).attribute<Position>(i.localIdx);
}

size_t Hierarchy::sizeInBytes() const
{
	size_t size = sizeof(*this);

	for (auto& c : grid)
	{
		size += sizeof(c.first);
		size += c.second.sizeInBytes();		
	}

	for (auto& level : innerLevels)
	{
		size += level.sizeInBytes();		
	}

	size += subdivs.size() * sizeof(SubdivisionInfo);

	return size;
}



template<Attribute A>
void Hierarchy::copyToFinerLevel(int srcLevel)
{
	assert(srcLevel > 0 && srcLevel < innerLevels.size() + 1);
	int dstLevel = srcLevel - 1;

	if (dstLevel == 0)
	{
		for (auto& cell : grid)
		{
			auto& parentNode = innerLevels.at(0).nodes.at(cell.first);
			for (int i = 0; i < cell.second.position.size(); ++i)
			{
				cell.second.attribute<A>(i) = parentNode.attribute<A>();
				AttributeConsistency<A>::makeConsistent(SpecificCellAttributeAccess(cell.second, i));
			}
		}
	}
	else
	{
		auto& nodes = innerLevels[dstLevel - 1].nodes;
		int dstNodeLevel = dstLevel - 1;
		tbb::parallel_for_each(nodes.begin(), nodes.end(), [&](auto& v)
		{			
			auto& node = v.second;
			auto p = parent<3>(v.first, dstNodeLevel);
			
			node.attribute<A>() = this->attribute<A>(InnerIndex(p, srcLevel - 1));
			AttributeConsistency<A>::makeConsistent(node);
		});
	}
}

template void Hierarchy::copyToFinerLevel<Normal>(int srcLevel);
template void Hierarchy::copyToFinerLevel<DirField>(int srcLevel);
template void Hierarchy::copyToFinerLevel<PosField>(int srcLevel);

template<Attribute A>
void Hierarchy::filterToCoarserLevel(int srcLevel)
{
	assert(srcLevel >= 0 && srcLevel < innerLevels.size());
	int dstLevel = srcLevel + 1;
	if (srcLevel == 0)
	{
		for (auto& cell : grid)
		{
			auto parentNode = innerLevels.at(0).nodes.at(cell.first);
			parentNode.attribute<A>() = AttributeTraits<A>::Type(); //set to 0
			for (int i = 0; i < cell.second.position.size(); ++i)
			{
				parentNode.attribute<A>() += 1.0f / parentNode.area * cell.second.attribute<A>(i);				
			}
			AttributeConsistency<A>::makeConsistent(parentNode);
		}
	}
	else
	{
		auto& nodes = innerLevels[dstLevel - 1].nodes;
		int dstNodeLevel = dstLevel - 1;
		int srcNodeLevel = srcLevel - 1;
		auto& srcLevelNodes = innerLevels.at(srcNodeLevel).nodes;
		tbb::parallel_for_each(nodes.begin(), nodes.end(), [&](auto& v)
		{			
			auto& node = v.second;
			Int3Index child1Idx, child2Idx;
			children<3>(v.first, dstNodeLevel, child1Idx, child2Idx);
			auto child1 = srcLevelNodes.find(child1Idx);
			auto child2 = srcLevelNodes.find(child2Idx);
			if (child1 == srcLevelNodes.end())
				node.attribute<A>() = child2->second.attribute<A>();
			else if(child2 == srcLevelNodes.end())
				node.attribute<A>() = child1->second.attribute<A>();
			else
			{
				Float a1 = child1->second.attribute<Area>();
				Float a2 = child2->second.attribute<Area>();
				node.attribute<A>() = (a1 * child1->second.attribute<A>() + a2 * child2->second.attribute<A>()) / (a1 + a2);
			}
			AttributeConsistency<A>::makeConsistent(node);
		});
	}
}

template<Attribute A>
void Hierarchy::filterToCoarsestLevel()
{
	for (int i = 0; i < innerLevels.size(); ++i)
		filterToCoarserLevel<A>(i);
}

template void Hierarchy::filterToCoarsestLevel<Normal>();

template<int DIM>
inline typename IntIndexHelper<DIM>::Type Hierarchy::parent(const typename IntIndexHelper<DIM>::Type& idx, int childNodeLevel) const
{
	//calculate the tree-local index
	IntIndexHelper<DIM>::Type localIdx = idx + innerLevels[childNodeLevel].toLocalOffset;

	//map to parent
	localIdx(subdivs[childNodeLevel].dimension) = floor(localIdx(subdivs[childNodeLevel].dimension) / 2.0f);

	//convert back to global index
	return localIdx - innerLevels[childNodeLevel + 1].toLocalOffset;
}

template <int DIM>
void Hierarchy::children(const typename IntIndexHelper<DIM>::Type& idx, int parentNodeLevel, typename IntIndexHelper<DIM>::Type& child1, typename IntIndexHelper<DIM>::Type& child2) const
{
	//calculate the tree-local index
	child1 = idx + innerLevels[parentNodeLevel].toLocalOffset;
	child2 = child1;

	child1(subdivs[parentNodeLevel - 1].dimension) = child1(subdivs[parentNodeLevel - 1].dimension) * 2;
	child2(subdivs[parentNodeLevel - 1].dimension) = child1(subdivs[parentNodeLevel - 1].dimension) + 1;

	child1 -= innerLevels[parentNodeLevel - 1].toLocalOffset;
	child2 -= innerLevels[parentNodeLevel - 1].toLocalOffset;
}

// --------  VertexFinestIterator  ----------

Hierarchy::VertexFinestIterator::VertexFinestIterator(size_t localIdx, GridType::const_iterator grid_current_iterator)
	: localIdx(localIdx), grid_current_iterator(grid_current_iterator)
{ }

const Hierarchy::VertexFinestIterator& Hierarchy::VertexFinestIterator::operator++()
{
	++localIdx;
	if (localIdx >= grid_current_iterator->second.position.size())
	{
		localIdx = 0;
		++grid_current_iterator;
	}
	return *this;
}

bool Hierarchy::VertexFinestIterator::operator!=(VertexFinestIterator& other) const
{
	return localIdx != other.localIdx || grid_current_iterator != other.grid_current_iterator;
}

FinestIndex Hierarchy::VertexFinestIterator::operator*()
{
	return FinestIndex(grid_current_iterator->first, localIdx);
}