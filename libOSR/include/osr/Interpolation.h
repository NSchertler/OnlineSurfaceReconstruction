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
	template <typename T>
	T bilinear(const T& v1, const T& v2, const T& v3, const T& v4, const Vector2f& uv)
	{
		return (1 - uv.x()) * ((1 - uv.y()) * v1 + uv.y() * v4) + uv.x() * ((1 - uv.y()) * v2 + uv.y() * v3);
	}

	template <typename T>
	T barycentric(const T& v1, const T& v2, const T& v3, const Vector2f& uv)
	{
		return uv.x() * v1 + uv.y() * v2 + (1 - uv.x() - uv.y()) * v3;
	}
}