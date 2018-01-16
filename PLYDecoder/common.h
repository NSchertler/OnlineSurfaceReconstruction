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
#if defined(_WIN32)
#pragma warning(disable: 4244 4018 4100 4610 4510 4127 4512 4146 4267 4503 4800 4706)
#endif

#include <Eigen/Core>
/*#include <tbb/parallel_for_each.h>*/

namespace osr
{
#define SINGLE_PRECISION
#define GRAIN_SIZE 1024

	/* Application precision -- can be set to single or double precision */
#if defined(SINGLE_PRECISION)
	typedef float Float;
#else
	typedef double Float;
#endif

	const uint32_t INVALID = (uint32_t)-1;
	const uint64_t INVALID64 = (uint64_t)-1;

#if defined(_WIN32)
#define RCPOVERFLOW_FLT   2.93873587705571876e-39f
#define RCPOVERFLOW_DBL   5.56268464626800345e-309
#else
#define RCPOVERFLOW_FLT   0x1p-128f
#define RCPOVERFLOW_DBL   0x1p-1024
#endif

#if defined(SINGLE_PRECISION)
#define RCPOVERFLOW RCPOVERFLOW_FLT
#else
#define RCPOVERFLOW RCPOVERFLOW_DBL
#endif

	/* Useful Eigen typedefs based on the current precision */
	typedef Eigen::Matrix<int32_t, 2, 1>                            Vector2i;
	typedef Eigen::Matrix<int32_t, 3, 1>                            Vector3i;
	typedef Eigen::Matrix<int32_t, 4, 1>                            Vector4i;
	typedef Eigen::Matrix<uint32_t, 2, 1>                           Vector2u;
	typedef Eigen::Matrix<uint32_t, 3, 1>                           Vector3u;
	typedef Eigen::Matrix<uint32_t, 4, 1>                           Vector4u;
	typedef Eigen::Matrix<uint8_t, 4, 1>                            Vector4u8;
	typedef Eigen::Matrix<Float, 2, 1>                              Vector2f;
	typedef Eigen::Matrix<Float, 3, 1>                              Vector3f;
	typedef Eigen::Matrix<Float, 4, 1>                              Vector4f;
	typedef Eigen::Matrix<unsigned short, 3, 1>						Vector3us;
	typedef Eigen::Matrix<int32_t, Eigen::Dynamic, 1>               VectorXi;
	typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>              VectorXu;
	typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>               VectorXu8;
	typedef Eigen::Matrix<bool, Eigen::Dynamic, 1>                  VectorXb;
	typedef Eigen::Matrix<Float, Eigen::Dynamic, 1>                 VectorXf;
	typedef Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic>  MatrixXi;
	typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
	typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>  MatrixXu8;
	typedef Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>	MatrixXf;
	typedef Eigen::Matrix<Float, 3, Eigen::Dynamic>					Matrix3Xf;
	typedef Eigen::Matrix<Float, 4, Eigen::Dynamic>					Matrix4Xf;
	typedef Eigen::Matrix<Float, 2, 2>                              Matrix2f;
	typedef Eigen::Matrix<Float, 3, 3>                              Matrix3f;
	typedef Eigen::Matrix<Float, 4, 4>                              Matrix4f;
	typedef Eigen::Matrix<unsigned short, 3, Eigen::Dynamic>		Matrix3Xus;


	extern std::string memString(size_t size, bool precise = false);

	template <typename Matrix>
	inline size_t sizeInBytes(const Matrix &matrix)
	{
		return matrix.size() * sizeof(typename Matrix::Scalar);
	}

	extern void coordinate_system(const Vector3f &a, Vector3f &b, Vector3f &c);

	extern Float signum(Float value);

	/// Always-positive modulo function, Float precision version (assumes b > 0)
	Float modulo(Float a, Float b);

	extern float fast_acos(float x);

	template<typename TIterator>
	class ForEachHelper
	{
	public:
		ForEachHelper(TIterator begin, TIterator end)
			: _begin(begin), _end(end)
		{ }

		TIterator begin() const { return _begin; }
		TIterator end() const { return _end; }

		template <typename TFunction>
		void processInParallel(const TFunction& f)
		{
			//Dummy implementation, should be specialized to achieve better parallel performance
			tbb::parallel_for_each(_begin, _end, f);
			//for (auto it = _begin; it != _end; ++it)
			//	f(*it);

			//override in the global namespace with:
			/*template<>
			template<typename TFunction>
			void ForEachHelper<SpecialIterator>::processInParallel(const TFunction& f) {  }*/
		}

	private:
		TIterator _begin, _end;
	};

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