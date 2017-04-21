#pragma once

#include <Eigen/Dense>

//Represents an n-dimensional axis-aligned bounding box. The lower limits are inclusive,
//whereas the upper limits are exclusive.
template <typename T, int DIM>
struct BoundingBox
{
	Eigen::Matrix<T, DIM, 1> min, max;

	//Initializes the bounding box with an empty area.
	BoundingBox()
	{
		reset();
	}

	BoundingBox(const Eigen::Matrix<T, DIM, 1>& min, const Eigen::Matrix<T, DIM, 1>& max)
		: min(min), max(max)
	{ }

	//Resets the bounding box to an empty area.
	void reset()
	{
		min.setConstant(std::numeric_limits<float>::max());
		max.setConstant(std::numeric_limits<float>::lowest());
	}

	//Computes the intersection of two axis-aligned bounding boxes and returns if there is an intersection.
	bool intersect(const BoundingBox<T, DIM>& bb1, const BoundingBox<T, DIM>& bb2, BoundingBox<T, DIM>& out) const
	{
		for (int i = 0; i < DIM; ++i)
		{
			out.min[i] = std::max(bb1.min[i], bb2.min[i]);
			if (out.min[i] > bb1.max[i] || out.min[i] > bb2.max[i])
				return false;

			out.max[i] = std::min(bb1.max[i], bb2.max[i]);
			if (out.max[i] < bb1.min[i] || out.max[i] < bb2.min[i])
				return false;
		}
		return true;
	}

	//Computes the union of two axis-aligned bounding boxes.
	void unite(const BoundingBox<T, DIM>& bb1, const BoundingBox<T, DIM>& bb2, BoundingBox<T, DIM>& out) const
	{
		for (int i = 0; i < DIM; ++i)
		{
			out.min[i] = std::min(bb1.min[i], bb2.min[i]);
			out.max[i] = std::max(bb1.max[i], bb2.max[i]);
		}
	}

	//expands the bounding box to contain every point in V
	void expand(const Eigen::Matrix<T, DIM, -1>& V)
	{
		for (int v = 0; v < V.cols(); ++v)
		{
			for (int i = 0; i < DIM; ++i)
			{
				if (V(i, v) < min(i))
					min(i) = V(i, v);
				if (V(i, v) >= max(i))
					max(i) = nextafter(V(i, v), std::numeric_limits<T>::max());
			}
		}
	}

	//Returns if the bounding box contains the given point.
	bool containsPoint(const Eigen::Matrix<T, DIM, 1>& v) const
	{
		for (int i = 0; i < DIM; ++i)
		{
			if (v(i) < min(i) || v(i) >= max(i))
				return false;
		}
		return true;
	}

	//Returns the box's diagonal
	Eigen::Matrix<T, DIM, 1> diagonal() const { return max - min; }

	//Returns the box's center
	Eigen::Matrix<T, DIM, 1> center() const { return 0.5f * (min + max); }

	//Transforms the bounding box with a given affine transform and returns the axis-aligned bounding box
	//of the transformed box.
	BoundingBox<T, DIM> transform(const Eigen::Transform<T, DIM, Eigen::Affine>& t) const
	{
		//http://dev.theomader.com/transform-bounding-boxes/

		Eigen::Matrix<T, DIM, 1> tMin = t.translation();
		Eigen::Matrix<T, DIM, 1> tMax = t.translation();

		for (int i = 0; i < DIM; ++i)
		{
			Eigen::Matrix<T, DIM, 1> a = t.linear().col(i) * min(i);
			Eigen::Matrix<T, DIM, 1> b = t.linear().col(i) * max(i);

			tMin += a.cwiseMin(b);
			tMax += a.cwiseMax(b);
		}		

		return BoundingBox<T, DIM>(tMin, tMax);
	}
};