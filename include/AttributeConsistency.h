/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#pragma once

#include "Attributes.h"

namespace osr
{

	template <Attribute A>
	struct AttributeConsistency
	{
		//Re-establishes consistency for the given attribute of the specified vertex.
		//TAttributeAccess is an object that provides access to a vertex' attributes in the form
		//  attributeAccess.attribute<Attribute>().
		//Returns if consistency could be established successfully.
		template <typename TAttributeAccess>
		static bool makeConsistent(TAttributeAccess& attributeAccess) { return true; }
	};

	template <>
	struct AttributeConsistency<Normal>
	{
		template <typename TAttributeAccess>
		static bool makeConsistent(TAttributeAccess& attributeAccess)
		{
			//Normals should be of unit length
			if (attributeAccess.template attribute<Normal>().squaredNorm() < 0.001f)
				return false;
			attributeAccess.template attribute<Normal>().normalize();
			return true;
		}
	};

	template <>
	struct AttributeConsistency<DirField>
	{
		template <typename TAttributeAccess>
		static bool makeConsistent(TAttributeAccess& attributeAccess)
		{
			//The directional field should be perpendicular to the normal.
			Vector3f q = attributeAccess.template attribute<DirField>(),
				n = attributeAccess.template attribute<Normal>();
			attributeAccess.template attribute<DirField>() = (q - n * n.dot(q)).normalized();
			return true;
		}
	};

	template <>
	struct AttributeConsistency<PosField>
	{
		template <typename TAttributeAccess>
		static bool makeConsistent(TAttributeAccess& attributeAccess)
		{
			//The positional field should lie in the tangent plane.
			Vector3f o = attributeAccess.template attribute<PosField>(),
				n = attributeAccess.template attribute<Normal>(),
				p = attributeAccess.template attribute<Position>();
			attributeAccess.template attribute<PosField>() = o - n * n.dot(o - p);
			return true;
		}
	};
}