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
	template <typename IndexType>
	class OSR_EXPORT IPointQueryable
	{
	public:
		typedef IndexType PointQueryableIndexType;

		virtual Vector3f neighborP(const IndexType& i) const = 0; //access to point position
		virtual Vector3f neighborN(const IndexType& i) const = 0; //access to point normal

		virtual bool isIndexValid(const IndexType& idx) const { return true; }

		virtual IndexType findClosestCompatiblePoint(const Vector3f& p, const Vector3f& n) const = 0;
	};
}