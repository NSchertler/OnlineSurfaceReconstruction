#pragma once

#include <Eigen/Dense>
#include <queue>

struct KdTreeNode
{
	KdTreeNode(long long vIndex) 
		:vertexIndex(vIndex), left(nullptr), right(nullptr)
	{ }

	~KdTreeNode()
	{
		if (left != nullptr)
			delete left;
		if (right != nullptr)
			delete right;
	}

	void print(std::ostream& os, int indentation)
	{
		for (int i = 0; i < indentation - 1; ++i)
			os << "| ";
		if (indentation > 0)
			os << "|-";
		if (this == nullptr)
			os << "*" << std::endl;
		else
		{
			os << vertexIndex << std::endl;
			left->print(os, indentation + 1);
			right->print(os, indentation + 1);
		}
	}

	long long vertexIndex;
	KdTreeNode *left, *right;
};

struct KnnEntry
{
	long long vertexId;
	float sqrDistance;

	KnnEntry(long vertexId, float sqrDistance) : vertexId(vertexId), sqrDistance(sqrDistance) {}	
};
extern inline bool operator<(const KnnEntry& lhs, const KnnEntry& rhs);

template <typename TVertexStore> class KdTree;
template <typename TVertexStore> std::ostream& operator<<(std::ostream& os, const KdTree<TVertexStore>& tree);


template <typename TVertexStore>
class KdTree
{
private:
	TVertexStore& vertexStore;
	long long& vertexStoreStartIndex;

	KdTreeNode* root;

	int dimFrom, dimTo;

	inline float getVertexValue(long long globalIndex, int dimension)
	{
		return vertexStore[(unsigned int)(globalIndex - vertexStoreStartIndex)].position[dimension];
	}

	KdTreeNode** findMin(KdTreeNode** root, int dimension, int cutDimension, int& cutDimensionOfMin)
	{
		if (root == nullptr || *root == nullptr)
			return nullptr;
		if (cutDimension > dimTo)
			cutDimension = dimFrom;
		if (cutDimension == dimension)
		{
			if ((*root)->left == nullptr)
			{
				cutDimensionOfMin = cutDimension;
				return root;
			}
			else
				return findMin(&(*root)->left, dimension, cutDimension + 1, cutDimensionOfMin);
		}
		else
		{
			int cutDimensionOfLeftMin, cutDimensionOfRightMin;
			cutDimensionOfMin = cutDimension;
			KdTreeNode** min = root;
			KdTreeNode** minLeft = findMin(&(*root)->left, dimension, cutDimension + 1, cutDimensionOfLeftMin);
			KdTreeNode** minRight = findMin(&(*root)->right, dimension, cutDimension + 1, cutDimensionOfRightMin);			
			if (minLeft != nullptr && *minLeft != nullptr && getVertexValue((*minLeft)->vertexIndex, dimension) < getVertexValue((*min)->vertexIndex, dimension))
			{
				min = minLeft;
				cutDimensionOfMin = cutDimensionOfLeftMin;
			}
			if (minRight != nullptr && *minRight != nullptr && getVertexValue((*minRight)->vertexIndex, dimension) < getVertexValue((*min)->vertexIndex, dimension))
			{
				min = minRight;
				cutDimensionOfMin = cutDimensionOfRightMin;
			}
			return min;
		}
	}

	KdTreeNode** searchNode(long long vertexIndex, bool searchForInsert, int& cutDimension)
	{
		auto vertexPos = vertexStore[(unsigned int)(vertexIndex - vertexStoreStartIndex)].position;

		cutDimension = dimFrom;
		KdTreeNode** target = &root;
		while (*target != nullptr)
		{
			if (!searchForInsert && (*target)->vertexIndex == vertexIndex)
				return target;

			target = vertexPos[cutDimension] < vertexStore[(unsigned int)((*target)->vertexIndex - vertexStoreStartIndex)].position[cutDimension]
				? &(*target)->left : &(*target)->right;
			++cutDimension;
			if (cutDimension > dimTo)
				cutDimension = dimFrom;
		}

		return target;
	}

	void remove(KdTreeNode** node, int cutDimension)
	{
		if (*node == nullptr || *node == nullptr)
			return;

		if ((*node)->right != nullptr)
		{
			int minCutDimension;
			KdTreeNode** min = findMin(&(*node)->right, cutDimension, cutDimension + 1, minCutDimension);
			(*node)->vertexIndex = (*min)->vertexIndex;
			remove(min, minCutDimension);
		}
		else if ((*node)->left != nullptr)
		{
			int minCutDimension;
			KdTreeNode** min = findMin(&(*node)->left, cutDimension, cutDimension + 1, minCutDimension);
			(*node)->vertexIndex = (*min)->vertexIndex;
			remove(min, minCutDimension);
			(*node)->right = (*node)->left;
			(*node)->left = nullptr;
		}
		else
		{
			//node is a leaf
			delete *node;
			*node = nullptr;
		}
	}

	
	template <bool insertSearchPoint, typename TNeighbor, bool allowOnlyPreviousNeighbors>
#ifdef GATHER_STATISTICS
	//Returns number of performed comparisons
	int
#else
	void
#endif
	findKnn(KdTreeNode** node, Eigen::Vector3f& search, float radiusSqr, unsigned int k, std::priority_queue<TNeighbor>& result, int cutDimension, long long vertexId)
	{
		if (*node == nullptr)
		{
			if (insertSearchPoint)
				*node = new KdTreeNode(vertexId);
			return
#ifdef GATHER_STATISTICS
				0
#endif
				;
		}
		if (result.size() == k)
			radiusSqr = result.top().sqrDistance;
		if (cutDimension > dimTo)
			cutDimension = dimFrom;

#ifdef GATHER_STATISTICS
		int comparisons = 1;
#endif

		long long nodeVid = (*node)->vertexIndex;

		if ((!allowOnlyPreviousNeighbors || nodeVid < vertexId) && nodeVid != vertexId)
		{
			Eigen::Vector3f diff = vertexStore[(unsigned int)(nodeVid - vertexStoreStartIndex)].position - search;
			float sqrDistance = diff.squaredNorm();
			if (sqrDistance < radiusSqr)
				result.push(TNeighbor(nodeVid, sqrDistance));
		}

		if (result.size() > k)
			result.pop();

		if (result.size() == k)
			radiusSqr = result.top().sqrDistance;

		float distanceToCutPlane = search[cutDimension] - getVertexValue(nodeVid, cutDimension);
		bool pointInLeftSubTree = distanceToCutPlane < 0;
		if (pointInLeftSubTree)
		{
#ifdef GATHER_STATISTICS
			comparisons += 
#endif
				findKnn<insertSearchPoint, TNeighbor, allowOnlyPreviousNeighbors>(&(*node)->left, search, radiusSqr, k, result, cutDimension + 1, vertexId);
			if (result.size() == k)
				radiusSqr = result.top().sqrDistance;
			
			if (distanceToCutPlane * distanceToCutPlane < radiusSqr)
			{
#ifdef GATHER_STATISTICS
				comparisons +=
#endif
					findKnn<false, TNeighbor, allowOnlyPreviousNeighbors>(&(*node)->right, search, radiusSqr, k, result, cutDimension + 1, vertexId);
				if (result.size() == k)
					radiusSqr = result.top().sqrDistance;
			}
		}
		else
		{
#ifdef GATHER_STATISTICS
			comparisons +=
#endif
				findKnn<insertSearchPoint, TNeighbor, allowOnlyPreviousNeighbors>(&(*node)->right, search, radiusSqr, k, result, cutDimension + 1, vertexId);
			if (result.size() == k)
				radiusSqr = result.top().sqrDistance;
			
			if (distanceToCutPlane * distanceToCutPlane < radiusSqr)
			{
#ifdef GATHER_STATISTICS
				comparisons +=
#endif
					findKnn<false, TNeighbor, allowOnlyPreviousNeighbors>(&(*node)->left, search, radiusSqr, k, result, cutDimension + 1, vertexId);
				if (result.size() == k)
					radiusSqr = result.top().sqrDistance;
			}
		}
#ifdef GATHER_STATISTICS
		return comparisons;
#endif
		
	}

public:
	KdTree(TVertexStore& vertexStore, long long& vertexStoreStartIndex, int dimFrom, int dimTo)
		: vertexStore(vertexStore), vertexStoreStartIndex(vertexStoreStartIndex), dimFrom(dimFrom), dimTo(dimTo), root(nullptr)
	{}

	void insert(long long vertexIndex)
	{		
		int cutDimension;
		*searchNode(vertexIndex, true, cutDimension) = new KdTreeNode(vertexIndex);
	}

	void remove(long long vertexIndex)
	{
		int cutDimension;
		KdTreeNode** node = searchNode(vertexIndex, false, cutDimension);
		remove(node, cutDimension);
	}

	//Returns number of comparisons
	template <bool insertSearchPoint, typename TNeighbor, bool allowOnlyPreviousNeighbors = false>
#ifdef GATHER_STATISTICS
	//Returns number of performed comparisons
	int
#else
	void
#endif
	findKnn(Eigen::Vector3f& search, float radiusSqr, int k, long long vertexId, std::priority_queue<TNeighbor>& result)
	{
#ifdef GATHER_STATISTICS
		return
#endif
		findKnn<insertSearchPoint, TNeighbor, allowOnlyPreviousNeighbors>(&root, search, radiusSqr, k, result, dimFrom, vertexId);
	}

	~KdTree()
	{
		if (root != nullptr)
			delete root;
	}

	friend std::ostream& operator<<<>(std::ostream&, const KdTree<TVertexStore>&);
};

template <typename TVertexStore>
std::ostream& operator<<(std::ostream& os, const KdTree<TVertexStore>& tree)
{
	tree.root->print(os, 0);
	return os;
}