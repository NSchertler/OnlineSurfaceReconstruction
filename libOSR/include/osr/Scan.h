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
#include "osr/INeighborQueryable.h"
#include "osr/HierarchyDecl.h"
#include "osr/nanoflannForwardDeclare.h"

#include "3rd/ICP.h"

#include <nsessentials/math/Morton.h>
#include <nsessentials/math/BoundingBox.h>
#include <nsessentials/gui/GLBuffer.h>
#include <nsessentials/gui/GLVertexArray.h>
#include <nsessentials/util/TimedBlock.h>

#include <random>
#include <iostream>
#include <memory>

namespace osr
{
	class Scan;
	class OSR_EXPORT IScanRenderer
	{
	public:
		virtual void initialize(Scan& scan) = 0;
		virtual bool isInitialized() const = 0;
		virtual void updateData(const Scan& scan) = 0;
		virtual void draw(const Scan& scan, const Eigen::Matrix4f & v, const Eigen::Matrix4f & proj) const = 0;

		bool showInput;
		bool showNormals;
	};

	//Represents data of a single scan
	class OSR_EXPORT Scan : public IPointQueryable<size_t>
	{
	public:
		Scan(const Matrix3Xf& V = Matrix3Xf(), const Matrix3Xf& N = Matrix3Xf(), const Matrix3Xus& C = Matrix3Xus(), const MatrixXu& F = MatrixXu(), const std::string& name = "unnamed", const Eigen::Affine3f& transform = Eigen::Affine3f::Identity());
		~Scan();

		void initialize();		

		//Calculates the vertex normals if not already present.
		//If there are faces in the data set, uses averaged face normals.
		//Otherwise, uses PCA. PCA assumes normals to point towards the origin.
		void calculateNormals();		

		//Access to transformed attributes
		Vector3f p(size_t idx) const; //vertex position
		Vector3f n(size_t idx) const; //normal

		const std::string& getName() { return name; }

		const nse::math::BoundingBox<float, 3> boundingBox() const { return bbox; }
		nse::math::BoundingBox<float, 3> getTransformedBoundingBox() const;

		void updateData();

		const Matrix3Xf& V() const { return mV; }
		Matrix3Xf& V() { return mV; }
		const Matrix3Xf& N() const { return mN; }
		Matrix3Xf& N() { return mN; }
		const Matrix3Xus& C() const { return mC; }
		Matrix3Xus& C() { return mC; }		
		const MatrixXu& F() const { return mF; }
		MatrixXu& F() { return mF; }

		//Modifies the scan transform via ICP so as to register to other.
		template <typename Index>
		void alignTo(const IPointQueryable<Index>& other, int iterations = 20, double subsample = 0.1);

		//Removes all points that overlap the hierarchy (i.e. there is a point in the hierarchy with a distance of at most "distance").
		void cleanOverlap(const THierarchy& hierarchy, float distance);

		const Eigen::Affine3f& transform() const { return mTransform; }
		Eigen::Affine3f& transform() { return mTransform; }

		std::shared_ptr<IScanRenderer> renderer;

		// ----------  nanoflann interface  ----------
		typedef nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Adaptor<float, Scan, float>, Scan, 3, size_t>  KdTreeType;

		inline size_t kdtree_get_point_count() const { return mV.cols(); }
		inline float kdtree_distance(const float *p1, const size_t idx_p2, size_t size) const
		{
			float s = 0;
			for (size_t i = 0; i < size; ++i)
			{
				const float d = p1[i] - mV.coeff(i, idx_p2);
				s += d*d;
			}
			return s;
		}

		inline float kdtree_get_pt(const size_t idx, int dim) const { return mV.coeff(dim, idx); }

		template <class BBOX>
		bool kdtree_get_bbox(BBOX& bb) const
		{
			for (int i = 0; i < 3; ++i)
			{
				bb[i].low = bbox.min(i);
				bb[i].high = bbox.max(i);
			}
			return true;
		}

		// ----------  end nanoflann interface  ----------

		void buildTree();

		Vector3f neighborP(const size_t& i) const { return mV.col(i); } //access to point position
		Vector3f neighborN(const size_t& i) const { return mN.col(i); }; //access to point normal

		bool isIndexValid(const size_t& idx) const { return idx < mV.cols(); }

		//Finds the closest point that has a similar normal as the provided one
		size_t findClosestCompatiblePoint(const Vector3f& p, const Vector3f& n) const;

		float closestPointRadius = 30;

#ifdef USE_DAVIDVIVE
		struct
		{
			Eigen::Affine3f transformUncalibrated; //turntable + controller transform
			Eigen::Affine3f turntableRotation;
			Eigen::Affine3f davidToVive;
		} davidViveData;
#endif

	private:
		KdTreeType* kdTree = nullptr;

	private:

		void calculateNormalsFromFaces();
		void calculateNormalsPCA();

		Matrix3Xf mV; //positions
		Matrix3Xf mN; //normals
		Matrix3Xus mC; //colors
		MatrixXu mF;  //faces

		std::string name;

		nse::math::BoundingBox<float, 3> bbox;

		Eigen::Affine3f mTransform;
	};

	template <typename Index>
	void Scan::alignTo(const IPointQueryable<Index>& other, int iterations, double subsample)
	{
		nse::util::TimedBlock b("Registering scan ..");

		std::vector<Index> correspondences(mV.cols());
		//For each point, find the corresponding point in the other point cloud.
#pragma omp parallel for
		for (int i = 0; i < mV.cols(); ++i)
		{
			if (std::isnan(mV.col(i).x()))
				continue;
			correspondences[i] = other.findClosestCompatiblePoint(mTransform * mV.col(i), mTransform.linear() * mN.col(i));
		}

		//Distribute the points with a correspondence into normal buckets.
		std::map<nse::math::MortonCode64, std::vector<size_t>> normalBucketsMap;
		for (int i = 0; i < mV.cols(); ++i)
		{
			if (!std::isnan(mV.col(i).x()) && other.isIndexValid(correspondences[i]))
			{
				Vector3i discrete = (mN.col(i) * 10).cast<int>();
				nse::math::MortonCode64 code(discrete.x(), discrete.y(), discrete.z());
				normalBucketsMap[code].push_back(i);
			}
		}
		std::vector<std::vector<size_t>> normalBuckets;
		int potentialSamples = 0;
		for (auto& entry : normalBucketsMap)
		{
			potentialSamples += entry.second.size();
			normalBuckets.push_back(std::move(entry.second));
		}
		normalBucketsMap.clear();

		if (potentialSamples < 10)
		{
			std::cout << "Could not find enough overlap. Registration will abort." << std::endl;
			return;
		}
		int samples = (int)(potentialSamples * subsample);

		std::uniform_int_distribution<size_t> bucketDist(0, normalBuckets.size() - 1);
		std::mt19937 rnd;

		Matrix3Xf X(3, samples), N(3, samples);

		//subsample the point cloud for ICP
		for (int i = 0; i < samples; ++i)
		{
			size_t sample;

			if (subsample == 1)
				sample = i;
			else
			{
				//normal space sampling

				bool sampleOk = false;
				int attempt = 0;
				while (!sampleOk && attempt++ < 10)
				{
					auto bucketIdx = bucketDist(rnd);
					auto& bucket = normalBuckets[bucketIdx];

					std::uniform_int_distribution<size_t> sampleDist(0, bucket.size() - 1);
					auto sampleIdx = sampleDist(rnd);
					sample = bucket[sampleIdx];

					if (std::isnan(mV.coeff(0, sample)) || std::isnan(mN.coeff(0, sample)))
						continue;

					sampleOk = true;

					bucket.erase(bucket.begin() + sampleIdx);
					if (bucket.empty())
					{
						normalBuckets.erase(normalBuckets.begin() + bucketIdx);
						bucketDist = std::uniform_int_distribution<size_t>(0, normalBuckets.size() - 1);
					}
				}
			}
			X.col(i) = mTransform * mV.col(sample);
			N.col(i) = mTransform.linear() * mN.col(sample);
		}

		//Run ICP
		SICP::Parameters params;
		params.p = 1.5;
		params.max_icp = iterations;
		params.max_outer = 10;
		params.max_inner = 1;
		Eigen::setNbThreads(0);
		mTransform = SICP::point_to_plane(X, N, other, params) * mTransform;
		Eigen::setNbThreads(1);
	}
}