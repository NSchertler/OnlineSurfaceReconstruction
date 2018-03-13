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

#include <vector>

#include "osr/common.h"

namespace osr
{
	class IOrientationFieldTraits
	{
	public:
		IOrientationFieldTraits(int rosy);
		//returns the symmetry number of this RoSy object
		int rosy() { return _rosy; }

		//returns unit vectors for the first n/2 principal directions (e.g. if a 4-RoSy is used, returns two vectors)
		virtual void getPrincipalDirections(const Vector3f& dirField, const Vector3f& normal, Matrix3Xf& outDirections) = 0;

		//for two given representative input directions and their respective normals, calculates the two directions that are closest to each other
		virtual void findCompatible(const Vector3f &dirField0, const Vector3f &normal0, const Vector3f &dirField1, const Vector3f &normal1, Vector3f& compat1, Vector3f& compat2) = 0;

		//for two given representative input directions and their respective normals, calculates the two directions that are closest to each other
		virtual void findCompatible(const Vector3f &dirField0, const Vector3f &normal0, const Matrix3Xf& principalDirections, Matrix3Xf& compatOut, VectorXf& scoreOut) = 0;
	private:
		int _rosy;
	};

	template <int RoSy>
	class OrientationFieldTraits : public IOrientationFieldTraits
	{
	public:
		OrientationFieldTraits();

		void getPrincipalDirections(const Vector3f& dirField, const Vector3f& normal, Matrix3Xf& outDirections);
		void findCompatible(const Vector3f &dirField0, const Vector3f &normal0, const Vector3f &dirField1, const Vector3f &normal1, Vector3f& compat1, Vector3f& compat2);
		virtual void findCompatible(const Vector3f &dirField0, const Vector3f &normal0, const Matrix3Xf& principalDirections, Matrix3Xf& compatOut, VectorXf& scoreOut);
	};

	//Returns the RoSy object for the requested RoSy symmetry
	extern IOrientationFieldTraits* getOrientationFieldTraits(int roSy);

	class IPositionFieldTraits
	{
	public:
		IPositionFieldTraits(int posy);
		//returns the symmetry number of this PoSy object
		int posy() { return _posy; }

		//for two given representative positions with their respective tangent frames, returns the two positions that are closest to each other.
		virtual void findCompatible(
			const Vector3f &point0, const Vector3f &normal0, const Vector3f &dirField0, const Vector3f &posField0,
			const Vector3f &point1, const Vector3f &normal1, const Vector3f &dirField1, const Vector3f &posField1,
			Float scale, Float inv_scale, Vector3f& compat1, Vector3f& compat2) = 0;

		//for two given representative positions with their respective tangent frames, returns the integer translation for the points that are closest to each other.
		virtual std::pair<Vector2i, Vector2i> findCompatibleIndex(
			const Vector3f &point0, const Vector3f &normal0, const Vector3f &dirField0, const Vector3f &posField0,
			const Vector3f &point1, const Vector3f &normal1, const Vector3f &dirField1, const Vector3f &posField1,
			Float scale, Float inv_scale, Float* error = nullptr) = 0;

		//rounds a 3d position to the nearest integer positional field value
		virtual Vector3f positionRound(const Vector3f &posField, const Vector3f &dirField,
			const Vector3f &normal, const Vector3f &point,
			Float scale, Float inv_scale) = 0;

	private:
		int _posy;
	};

	template <int PoSy>
	class PositionFieldTraits : public IPositionFieldTraits
	{
	public:
		PositionFieldTraits();

		void findCompatible(
			const Vector3f &point0, const Vector3f &normal0, const Vector3f &dirField0, const Vector3f &posField0,
			const Vector3f &point1, const Vector3f &normal1, const Vector3f &dirField1, const Vector3f &posField1,
			Float scale, Float inv_scale, Vector3f& compat1, Vector3f& compat2);

		std::pair<Vector2i, Vector2i> findCompatibleIndex(
			const Vector3f &point0, const Vector3f &normal0, const Vector3f &dirField0, const Vector3f &posField0,
			const Vector3f &point1, const Vector3f &normal1, const Vector3f &dirField1, const Vector3f &posField1,
			Float scale, Float inv_scale, Float* error = nullptr);

		Vector3f positionRound(const Vector3f &posField, const Vector3f &dirField,
			const Vector3f &normal, const Vector3f &point,
			Float scale, Float inv_scale);
	};

	//Returns the PoSy object for the requested PoSy symmetry
	extern IPositionFieldTraits* getPositionFieldTraits(int poSy);
}