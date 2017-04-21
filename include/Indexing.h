#pragma once

#include "common.h"

template <int DIMS, typename T = int16_t>
struct IntIndexHelper
{
	typedef Eigen::Matrix<T, DIMS, 1> Type;
};
typedef IntIndexHelper<3>::Type Int3Index;

template<int DIM, typename T = int16_t>
typename IntIndexHelper<DIM, T>::Type ToIntIndex(const Eigen::Matrix<Float, DIM, 1>& p, Float gridSize)
{
	typename IntIndexHelper<DIM, T>::Type idx;
	for (int j = 0; j < DIM; ++j)
		idx(j) = (T)floor(p(j) / gridSize);
	return idx;
}

namespace std
{
	template<>
	struct hash<typename IntIndexHelper<3, int16_t>::Type>
	{
		size_t operator()(const typename IntIndexHelper<3, int16_t>::Type& o) const
		{
			return o(0) * 73856093 + o(1) * 19349663 + o(2) * 83492791;
		}
	};
	template<>
	struct hash<typename IntIndexHelper<3, int32_t>::Type>
	{
		size_t operator()(const typename IntIndexHelper<3, int32_t>::Type& o) const
		{
			return o(0) * 73856093 + o(1) * 19349663 + o(2) * 83492791;
		}
	};
}
