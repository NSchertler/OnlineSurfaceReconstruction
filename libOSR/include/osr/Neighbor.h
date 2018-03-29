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

#include "common.h"

namespace osr
{
	template <typename TData>
	struct Neighbor
	{
		Neighbor(TData idx, Float distanceSq)
			: idx(idx), distanceSq(distanceSq)
		{ }

		bool operator>(const Neighbor<TData>& other) const { return distanceSq > other.distanceSq; }
		bool operator<(const Neighbor<TData>& other) const { return distanceSq < other.distanceSq; }

		TData idx;
		Float distanceSq;
	};

	template <typename TData>
	struct NeighborStrictOrder
	{
		NeighborStrictOrder(TData idx, Float distanceSq)
			: idx(idx), distanceSq(distanceSq)
		{ }

		bool operator>(const NeighborStrictOrder<TData>& other) const
		{
			if (distanceSq != other.distanceSq)
				return distanceSq > other.distanceSq;
			return idx > other.idx;
		}
		bool operator<(const NeighborStrictOrder<TData>& other) const
		{
			if (distanceSq != other.distanceSq)
				return distanceSq < other.distanceSq;
			return idx < other.idx;
		}

		TData idx;
		Float distanceSq;
	};
}