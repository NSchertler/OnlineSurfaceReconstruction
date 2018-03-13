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

	enum Attribute
	{
		Position,
		Normal,
		DirField,
		PosField,
		Area,
		MeshVertex,
		MeshVertexGeneration,
		Color,
		DirFieldConstraint,
	};

	template <Attribute A>
	struct AttributeTraits
	{
		typedef void Type;
		constexpr static const char* name() { return "unnamed attribute"; }
	};

	template <>
	struct AttributeTraits<Position>
	{
		typedef Vector3f Type;
		constexpr static const char* name() { return "Position"; }
	};

	template <>
	struct AttributeTraits<Normal>
	{
		typedef Vector3f Type;
		constexpr static const char* name() { return "Normal"; }
	};

	template <>
	struct AttributeTraits<DirField>
	{
		typedef Vector3f Type;
		constexpr static const char* name() { return "Orientation Field"; }
	};

	template <>
	struct AttributeTraits<PosField>
	{
		typedef Vector3f Type;
		constexpr static const char* name() { return "Position Field"; }
	};

	template <>
	struct AttributeTraits<Area>
	{
		typedef float Type;
		constexpr static const char* name() { return "Area"; }
	};

	struct MeshVertexType
	{
		uint32_t vertexIndex;
		unsigned char generation;

		MeshVertexType() : vertexIndex(INVALID) { }
		MeshVertexType(uint32_t idx) : vertexIndex(idx) { }
		MeshVertexType(uint32_t idx, unsigned char generation) : vertexIndex(idx), generation(generation) { }
	};

	template <>
	struct AttributeTraits<MeshVertex>
	{
		typedef uint32_t Type;
		constexpr static const char* name() { return "Mesh Vertex Index"; }
	};
	template <>
	struct AttributeTraits<MeshVertexGeneration>
	{
		typedef unsigned char Type;
		constexpr static const char* name() { return "Mesh Vertex Index Generation"; }
	};

	template <>
	struct AttributeTraits<Color>
	{
		typedef Vector3us Type; //CIE La*b* space
		constexpr static const char* name() { return "Color"; }
	};

	template <>
	struct AttributeTraits<DirFieldConstraint>
	{
		typedef Vector3f Type;
		constexpr static const char* name() { return "Orientation Field Constraint"; }
	};

	template <Attribute A>
	struct EigenAttributeTraits
	{
		typedef AttributeTraits<A> Traits;
		typedef typename Traits::Type Type;
		typedef typename Type::Scalar Scalar;

		typedef Eigen::Matrix<Scalar, Type::RowsAtCompileTime, -1> MatrixType;
	};

	template <>
	struct EigenAttributeTraits<Area>
	{
		typedef Eigen::Matrix<float, 1, -1> MatrixType;
	};

	template <>
	struct EigenAttributeTraits<Color>
	{
		typedef Eigen::Matrix<float, 3, -1> MatrixType;
	};

}