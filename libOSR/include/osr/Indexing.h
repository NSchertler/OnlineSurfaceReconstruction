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

#include "osr/common.h"

namespace osr
{
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
}

namespace std
{
	template<>
	struct hash<typename osr::IntIndexHelper<3, int16_t>::Type>
	{
		size_t operator()(const typename osr::IntIndexHelper<3, int16_t>::Type& o) const
		{
			return o(0) * 73856093 + o(1) * 19349663 + o(2) * 83492791;
		}
	};
	template<>
	struct hash<typename osr::IntIndexHelper<3, int32_t>::Type>
	{
		size_t operator()(const typename osr::IntIndexHelper<3, int32_t>::Type& o) const
		{
			return o(0) * 73856093 + o(1) * 19349663 + o(2) * 83492791;
		}
	};
}