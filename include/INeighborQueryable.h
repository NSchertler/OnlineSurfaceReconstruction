#pragma once

#include "common.h"

template <typename IndexType>
class IPointQueryable
{
public:
	typedef IndexType PointQueryableIndexType;

	virtual Vector3f neighborP(const IndexType& i) const = 0; //access to point position
	virtual Vector3f neighborN(const IndexType& i) const = 0; //access to point normal

	virtual bool isIndexValid(const IndexType& idx) const { return true; }

	virtual IndexType findClosestCompatiblePoint(const Vector3f& p, const Vector3f& n) const = 0;
};