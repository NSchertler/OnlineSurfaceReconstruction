#include "HierarchyOctree.h"
#include <random>

#include "AttributeConsistency.h"
#include "Optimizer.h"

using namespace HierarchyOctree;

#define CONSISTENT_NEIGHBOR_LEVELS
//#define PROJECT_TO_LEAF_NEIGHBORS

Hierarchy::Hierarchy()
{
	mBbxMin.setConstant(std::numeric_limits<float>::max());
	mBbxMax.setConstant(std::numeric_limits<float>::lowest());
}

void Hierarchy::addPoints(const Matrix3Xf & V, const Matrix3Xf & N)
{
	expandBoundingBox(V, mBbxMin, mBbxMax);

	coarsestGridSize = (mBbxMax - mBbxMin).maxCoeff() * 1.1f;
	origin = 0.5f * (mBbxMax + mBbxMin) - 0.5 * Vector3f(coarsestGridSize, coarsestGridSize, coarsestGridSize);

	std::vector<std::pair<Vector3f, Vector3f>> points;
	for (int i = 0; i < V.cols(); ++i)
		points.push_back(std::make_pair(V.col(i), N.col(i)));
	buildTree(points, 0, origin, coarsestGridSize);

	//find neighbors
	for (int level = 1; level < levels.size(); ++level)
	{
		auto& nodes = levels.at(level);
		auto& parentLevelNodes = levels.at(level - 1);
		for (auto& node : nodes)
		{
			auto parentIdx = parent(node.first);
			auto& parent = parentLevelNodes.at(parentIdx);
			for (int x = -1; x <= 1; ++x)
				for (int y = -1; y <= 1; ++y)
					for (int z = -1; z <= 1; ++z)
					{
						if (x == 0 && y == 0 && z == 0)
							continue;
						auto sameLevelNeighbor = nodes.find(node.first + Index(x, y, z));
						if (sameLevelNeighbor != nodes.end())
							node.second->neighbors.push_back(sameLevelNeighbor->second);
						else
						{
							//check the parent level
							auto parentLevelIdx = parentIdx + Index(x, y, z);
							auto parentLevelNeighbor = parentLevelNodes.find(parentLevelIdx);
							if (parentLevelNeighbor != parentLevelNodes.end())
							{								
#ifndef CONSISTENT_NEIGHBOR_LEVELS
								node.second->neighbors.push_back(parentLevelNeighbor->second);
#else
								//use the children if they exist
								std::vector<LevelType::iterator> children;
								findExistingChildren(parentLevelIdx, level - 1, children);
								if(children.size() == 0)
									node.second->neighbors.push_back(parentLevelNeighbor->second);
								else
								{
									for(auto it : children)
										node.second->neighbors.push_back(it->second);
								}
#endif
							}
						}
					}
		}
	}

	findTerminalNodes(Index(0, 0, 0), 0);

#ifdef PROJECT_TO_LEAF_NEIGHBORS
	//replace the neighborhood of leaf nodes with their according terminal nodes' neighborhood
	for (auto terminal : terminals)
	{
		std::vector<std::pair<int, Index>> leavesUnderTerminal;		
		findExistingLeaves(terminal->idx, terminal->level, leavesUnderTerminal);
		std::vector<std::pair<int, Index>> allLeaves(leavesUnderTerminal.begin(), leavesUnderTerminal.end());
		for (auto terminalNeighbor : terminal->neighbors)
			findExistingLeaves(terminalNeighbor->idx, terminalNeighbor->level, allLeaves);

		for (auto& tLeafAd : leavesUnderTerminal)
		{
			auto tLeaf = levels.at(tLeafAd.first).at(tLeafAd.second);
			tLeaf->neighbors.clear();
			for (auto& nAd : allLeaves)
			{
				auto nLeaf = levels.at(nAd.first).at(nAd.second);
				if (tLeaf == nLeaf)
					continue;
				tLeaf->neighbors.push_back(nLeaf);
			}
		}
	}
#endif

	auto root = levels.at(0).begin()->second;
	std::mt19937 rng;
	std::uniform_real_distribution<float> distDir(0, 2 * (float)M_PI);

	Vector3f s, t;
	coordinate_system(root->normal, s, t);
	float angle = distDir(rng);
	root->dirField = s * std::cos(angle) + t * std::sin(angle);	
	root->posField = root->position;
	
	optimizeFull();

	//print neighborhood stats
	int nSum = 0;
	for (auto v : leaves)		
		nSum += v->neighbors.size();
	std::cout << ((double)nSum / leaves.size()) << " neighbors in average." << std::endl;
}

void Hierarchy::optimizeFull()
{
	std::cout << "Optimizing .. ";
	for (int level = 1; level < levels.size(); ++level)
	{
		std::cout << level << " ";
		auto& parentNodes = levels.at(level - 1);
		auto& nodes = levels.at(level);
		std::vector<Node*> nodesInLevel;
		int leafNodes = 0;
		for (auto& node : nodes)
		{
			auto parentIdx = parent(node.first);
			auto& pNode = parentNodes.at(parentIdx);			
			node.second->dirField = pNode->dirField;
			node.second->posField = pNode->posField;			
			AttributeConsistency<DirField>::makeConsistent(*node.second);
			AttributeConsistency<PosField>::makeConsistent(*node.second);
			if (std::find(leaves.begin(), leaves.end(), node.second) != leaves.end())
				++leafNodes;
			else
				nodesInLevel.push_back(node.second);			
		}
		std::cout << "(" << nodesInLevel.size() << " + " << leafNodes << ") ";
		auto phases = ForEachHelper<PhasesIterator>(PhasesIterator(nodesInLevel.begin()), PhasesIterator(nodesInLevel.end()));
		optimizer->optimizeOrientations(*this, phases);
		optimizer->optimizePositions(*this, phases);
	}

	auto phasesLeaves = ForEachHelper<PhasesIterator>(PhasesIterator(leaves.begin()), PhasesIterator(leaves.end()));
	optimizer->optimizeOrientations(*this, phasesLeaves);
	optimizer->optimizePositions(*this, phasesLeaves);

	DirFieldChanged();
	PosFieldChanged();

	std::cout << "done." << std::endl;
}

#define FINEST_RES leaves

ForEachHelper<Hierarchy::PhasesIterator> Hierarchy::phases()
{
	return ForEachHelper<PhasesIterator>(FINEST_RES.begin(), FINEST_RES.end());
}

ForEachHelper<std::vector<Node*>::const_iterator> Hierarchy::vertices() const
{
	return ForEachHelper<std::vector<Node*>::const_iterator>(FINEST_RES.begin(), FINEST_RES.end());
}

size_t Hierarchy::vertexCount() const { return FINEST_RES.size(); }

size_t Hierarchy::toConsecutiveIndex(const Node* v)
{
	auto it = std::find(FINEST_RES.begin(), FINEST_RES.end(), v);
	if (it == FINEST_RES.end())
		return std::numeric_limits<size_t>::max();
	else
		return it - FINEST_RES.begin();
}
Node* Hierarchy::fromConsecutiveIndex(size_t v) { return FINEST_RES.at(v); }

ForEachHelper<std::vector<Node*>::const_iterator> Hierarchy::neighbors(const Node * v) const
{
	return ForEachHelper<std::vector<Node*>::const_iterator>(v->neighbors.begin(), v->neighbors.end());
}


void Hierarchy::buildTree(const std::vector<std::pair<Vector3f, Vector3f>>& points, int level, const Vector3f& bbxMin, float gridSize)
{
	auto node = new Node();	
	Vector3f bbxMax = bbxMin + Vector3f::Constant(gridSize);
	for (auto& p : points)
	{
		if (!pointInBoundingBox(p.first, bbxMin, bbxMax))
			continue;
		node->area += 1;
		node->position += 1.0f / node->area * (p.first - node->position);
		node->normal += 1.0f / node->area * (p.second - node->normal);
	}

	if (node->area == 0)
	{
		delete node;
		return;
	}

	if (levels.size() <= level)
		levels.resize(level + 1);

	node->level = level;
	node->idx = ToIntIndex<3, int32_t>(node->position - origin, gridSize);

	levels.at(level)[node->idx] = node;

	if (node->area == 1)
	{
		leaves.push_back(node);
		return;
	}

	gridSize /= 2;
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(0, 0, 0), gridSize);
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(0, 0, 1), gridSize);
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(0, 1, 0), gridSize);
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(0, 1, 1), gridSize);
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(1, 0, 0), gridSize);
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(1, 0, 1), gridSize);
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(1, 1, 0), gridSize);
	buildTree(points, level + 1, bbxMin + gridSize * Vector3f(1, 1, 1), gridSize);
}

void Hierarchy::findExistingChildren(const Index& idx, int level, std::vector<LevelType::iterator>& children)
{
	if (level + 1 < levels.size())
	{
		std::array<typename Index, 8> childr;
		this->children(idx, childr);

		auto& nextLevelNodes = levels.at(level + 1);
		for (auto& c : childr)
		{
			auto it = nextLevelNodes.find(c);
			if (it != nextLevelNodes.end())
			{
				children.push_back(it);				
			}
		}
	}
}

void Hierarchy::findExistingLeaves(const Index& idx, int level, std::vector<std::pair<int, Index>>& leaves)
{
	std::vector<LevelType::iterator> children;
	findExistingChildren(idx, level, children);
	if (children.size() == 0)
		leaves.push_back(std::make_pair(level, idx));
	else
	{
		for (auto it : children)
			findExistingLeaves(it->first, level + 1, leaves);
	}
}

void Hierarchy::findTerminalNodes(const Index& current, int currentLevel)
{	
	int sumNeighbors = 0;
	std::vector<LevelType::iterator> existingChildren;
	findExistingChildren(current, currentLevel, existingChildren);
	for (auto it : existingChildren)
		sumNeighbors += it->second->neighbors.size();
	
	if ((sumNeighbors < existingChildren.size() * 6 && currentLevel >= 4) || existingChildren.size() == 0)
	{
		//std::cout << "Recording terminal node at level " << currentLevel << " with " << ((float)sumNeighbors / existingChildren.size()) << " child neighbors in average." << std::endl;
		auto node = levels.at(currentLevel).at(current);
		terminals.push_back(node);
	}
	else
	{
		for (auto it : existingChildren)
			findTerminalNodes(it->first, currentLevel + 1);
	}
}

inline typename Index Hierarchy::parent(const typename Index& idx) const
{
	//map to parent
	return idx / 2;
	//for(int i = 0; i < DIM; ++i)
	//	idx(i) = floor(idx(i) / 2.0f);

	//return idx;
}

void Hierarchy::children(const typename Index& idx, std::array<typename Index, 8>& c) const
{
	auto cIdx = 2 * idx;

	c[0] = cIdx + Index(0, 0, 0);
	c[1] = cIdx + Index(0, 0, 1);
	c[2] = cIdx + Index(0, 1, 0);
	c[3] = cIdx + Index(0, 1, 1);
	c[4] = cIdx + Index(1, 0, 0);
	c[5] = cIdx + Index(1, 0, 1);
	c[6] = cIdx + Index(1, 1, 0);
	c[7] = cIdx + Index(1, 1, 1);
}