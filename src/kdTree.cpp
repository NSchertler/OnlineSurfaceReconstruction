#include "kdTree.h"

inline bool operator<(const KnnEntry& lhs, const KnnEntry& rhs)
{
	return lhs.sqrDistance < rhs.sqrDistance;
}