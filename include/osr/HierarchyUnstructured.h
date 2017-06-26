#pragma once

#include "osr/Attributes.h"
#include "osr/adjacency.h"
#include "osr/INeighborQueryable.h"
#include "osr/Hierarchy.h"

#include <nanoflann.hpp>

#include <tbb/tbb.h>

#include <iterator>

namespace HierarchyUnstructured
{
	extern AdjacencyMatrix
		downsample_graph(const AdjacencyMatrix adj, const std::vector<Vector3f> &V,
			const std::vector<Vector3f> &N, const std::vector<float> &areas, std::vector<Vector3f> &V_p,
			std::vector<Vector3f> &V_n, std::vector<float> &areas_p, MatrixXu &to_upper,
			VectorXu &to_lower, bool deterministic = false);

	struct Index
	{
		Index() { }
		Index(size_t idx, int level)
			: idx(idx), level(level)
		{ }

		bool operator==(const Index& other) const { return idx == other.idx && level == other.level; }

		size_t idx;
		int level;
	};

#define DefineAccessors(A, member) \
	template<> AttributeTraits<A>::Type& attribute<A>(const VertexIndex& i) { return member[i.level][i.idx]; } \
	template<> const AttributeTraits<A>::Type& attribute<A>(const VertexIndex& i) const { return member[i.level][i.idx]; } \

	class Hierarchy : public AbstractHierarchy<Index>, public INeighborQueryable<size_t>, public IKNNQueryable<size_t>
	{
		enum { MAX_DEPTH = 25 };
	public:

		Hierarchy(const Optimizer& optimizer, ExtractedMesh& extractionResult);
		~Hierarchy();

		// ---- begin public hierarchy interface  ----				

		//access to the data stored in the hierarchy
		template<Attribute A> typename AttributeTraits<A>::Type& attribute(const VertexIndex& i) { throw std::runtime_error("Field retrieval not implemented for type " + std::to_string(A) + "."); }
		template<Attribute A> const typename AttributeTraits<A>::Type& attribute(const VertexIndex& i) const { throw std::runtime_error("Field retrieval not implemented for type " + std::to_string(A) + "."); }
		DefineAccessors(Position, mV)
		DefineAccessors(Normal, mN)
		DefineAccessors(DirField, mDirField)
		DefineAccessors(PosField, mPosField)
		DefineAccessors(Area, mA)

		//integrate new points in the hierarchy
		void addPoints(const Matrix3Xf& V, const Matrix3Xf& N = Matrix3Xf(0, 0));

		void optimizeFull();

		void reset();

		struct VertexIterator;
		//returns an iterator for all vertices of the specified level
		ForEachHelper<VertexIterator> vertices(int level = 0) const;

		//returns an iterator for all neighbors of a given vertex
		//if the level is specified, it should not be 0.
		template <typename Callback>
		void forEachNeighbor(const VertexIndex& v, const Callback& callback) const;
		
		//returns the number of vertices on the finest level of the hierarchy
		size_t vertexCount() const { if (mV.size() == 0) return 0; return mV[0].size(); }		
		
		size_t sizeInBytes() const;

		// ---- end public hierarchy interface  ----

		void findNearestPointsRadius(const Vector3f& p, Float radius, const std::function<void(size_t)>& callback) const;
		void findNearestPointsKNN(const Vector3f& p, int k, const std::function<void(const size_t&)>& callback, Float maxRadiusSq = std::numeric_limits<Float>::infinity()) const;
		void findNearestPointsKNN(const Vector3f& p, int k, std::vector<size_t>& output) const;
		size_t findClosestPoint(const Vector3f& p) const;
		const Vector3f& p(const size_t& i) const;

		struct VertexIterator : public std::iterator<std::random_access_iterator_tag, VertexIndex>
		{
		public:
			VertexIterator() { }
			VertexIterator(VertexIndex v)
				: current(v)
			{ }
			VertexIterator(size_t idx, int level)
				: current({ idx, level })
			{ }

			const VertexIterator& operator++() { ++current.idx; return *this; }
			const VertexIterator& operator+=(size_t n) { current.idx += n; return *this; }
			VertexIterator operator+(size_t n) const { return VertexIterator(current.idx + n, current.level); }

			const VertexIterator& operator--() { --current.idx; return *this; }
			const VertexIterator& operator-=(size_t n) { current.idx -= n; return *this; }
			VertexIterator operator-(size_t n) const { return VertexIterator(current.idx - n, current.level); }

			size_t operator-(const VertexIterator& other) const { return current.idx - other.current.idx; }

			bool operator<(const VertexIterator& other) const { return current.idx < other.current.idx; }
			bool operator>(const VertexIterator& other) const { return current.idx > other.current.idx; }
			bool operator<=(const VertexIterator& other) const { return current.idx <= other.current.idx; }
			bool operator>=(const VertexIterator& other) const { return current.idx >= other.current.idx; }
			bool operator==(const VertexIterator& other) const { return current.idx == other.current.idx; }
			bool operator!=(const VertexIterator& other) const { return current.idx != other.current.idx; }
			VertexIndex& operator*() { return current; }

		private:
			VertexIndex current;
		};


		// ----  begin nanoflann hooks  ----
		// Must return the number of data points
		inline size_t kdtree_get_point_count() const;

		// Returns the L2 distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
		inline Float kdtree_distance(const Float *p1, const size_t idx_p2, size_t size) const;

		// Returns the dim'th component of the idx'th point in the class:
		inline Float kdtree_get_pt(const size_t idx, int dim) const;

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& bb) const
		{
			for (int i = 0; i < 3; ++i)
			{
				bb.elems[i].low = bbox.min(i);
				bb.elems[i].high = bbox.max(i);
			}
			return true;
		}
		// ----  end nanoflann hooks  ----

	private:
		//assigns a random solution to the coarsest level
		void resetSolution();

		void build();

		typedef nanoflann::KDTreeSingleIndexAdaptor< typename nanoflann::metric_L2_Simple::template traits<Float, Hierarchy>::distance_t, Hierarchy, 3, size_t>  kdTreeType;
		kdTreeType* kdTree;		

		std::vector<AdjacencyMatrix> mAdj;
		std::vector<std::vector<Vector3f>> mV;
		std::vector<std::vector<Vector3f>> mN;
		std::vector<std::vector<float>> mA;
		std::vector<std::vector<Vector3f>> mPosField;
		std::vector<std::vector<Vector3f>> mDirField;

		std::vector<VectorXu> mToCoarser;
		std::vector<MatrixXu> mToFiner; //mapping from vertices of level i+1 to vertices of level i

		template<Attribute A>
		//Copies the specified attribute for all vertices to the next-finer level.
		void copyToFinerLevel(int srcLevel);

		template<Attribute A>
		//Filters the specified attribute for all vertices to the next-coarser level.
		void filterToCoarserLevel(int srcLevel);

		template<Attribute A>
		//Filters the specified attribute for all vertices from the finest level to the coarsest level.
		void filterToCoarsestLevel();
	};

	// ---------  template implementations  ---------
	template <typename Callback>
	void Hierarchy::forEachNeighbor(const VertexIndex& v, const Callback& callback) const
	{
		for (size_t* l = mAdj[v.level][v.idx]; l != mAdj[v.level][v.idx + 1]; ++l)
			callback(Index(*l, v.level ));
	}
}

namespace std
{
	template<>
	struct hash<HierarchyUnstructured::Hierarchy::VertexIndex>
	{
		size_t operator()(const HierarchyUnstructured::Hierarchy::VertexIndex& o) const
		{
			return o.idx;
		}
	};
}

#undef DefineAccessors
