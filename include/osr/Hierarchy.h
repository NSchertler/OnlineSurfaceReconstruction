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

#include <boost/signals2.hpp>
#include "osr/common.h"
#include "osr/Optimizer.h"

#include <nsessentials/math/BoundingBox.h>

namespace osr
{
	class ExtractedMesh;

	//All implemented hierarchies must conform to this interface in order to work.
	template <typename TVertexIndex>
	class AbstractHierarchy
	{
	public:
		AbstractHierarchy(const Optimizer& optimizer, ExtractedMesh& extractionResult)
			: optimizer(optimizer), extractionResult(extractionResult)
		{ 	}

		//defines the type used to address points
		typedef TVertexIndex VertexIndex;

		//signals that are emitted whenever data are changed
		boost::signals2::signal<void()> PositionsChanged;
		boost::signals2::signal<void()> NormalsChanged;
		boost::signals2::signal<void()> AdjacencyChanged;
		boost::signals2::signal<void()> DirFieldChanged;
		boost::signals2::signal<void()> PosFieldChanged;

		//attribute access in the following form for every Index possibly emitted by the hierarchy:
		//  template<Attribute A> typename AttributeTraits<A>::Type& attribute(const Index& i);
		//  template<Attribute A> const typename AttributeTraits<A>::Type& attribute(const Index& i) const;

		//integrate new points in the hierarchy
		virtual void addPoints(const Matrix3Xf& V, const Matrix3Xf& N, const Matrix3Xus& C) = 0;

		//modifies the given points by changing their position, normal, and color
		virtual void modifyPoints(const std::vector<VertexIndex>& points, const Matrix3Xf& newV, const Matrix3Xf& newN, const Matrix3Xus& newC) = 0;

		//removes the specified points.
		virtual void removePoints(std::vector<VertexIndex>& points) = 0;

		//Starts the optimization for the entire hierarchy from scratch and extracts the final mesh afterwards.
		virtual void optimizeFull() = 0;

		virtual void reset() = 0;

		//access to the original vertices (on the finest levels) in the following form:
		//  IteratorHelper<Iterator> vertices()
		//    IteratorHelper<T> - exposes  T begin()  and  T end() 
		//    Iterator - any valid iterator that can be used to iterate the vertices. Must at 
		//               least conform to the forward iterator concept
		//if HierarchyCapabilities<>::AllowAccessToAllLevels, then vertices() must support the parameter int level

		//access to neighbors in the following form for every Index possibly emitted by the hierarchy:
		//  template <typename Callback>
		//  void forEachNeighbor(const Index& v, const Callback& callback) const;
		//    Callback - a callable object in the form void(const Index& v)

		//radius query. Does not need to be exact but must guarantee to include all vertices in the epsilon-ball.
		//  template <typename Callback>
		//  void findNearestPointsRadius(const Vector3f& p, Float radius, const Callback& callback) const;
		//    Callback of type void(const VertexIndex&)

		//returns the number of vertices on the finest level of the hierarchy
		//if HierarchyCapabilities<>::AllowAccessToAllLevels, then vertexCount() must support the parameter int level
		virtual size_t vertexCount() const = 0;

		const nse::math::BoundingBox<Float, 3>& boundingBox() const { return bbox; }

		void setMaxNeighborRadius(Float r) { maxNeighborRadius = r; }

		//returns the size of the hierarchy
		virtual size_t sizeInBytes() const = 0;

		const MeshSettings& meshSettings() const { return optimizer.meshSettings(); }

		ExtractedMesh& extractionResult;

		virtual float averagePointSpacing() const = 0;

	protected:
		virtual void init();

		nse::math::BoundingBox<Float, 3> bbox;

		const Optimizer& optimizer;


		Float maxNeighborRadius;
	};

	template<typename TVertexIndex>
	inline void AbstractHierarchy<TVertexIndex>::init()
	{
		bbox.reset();
	}
}