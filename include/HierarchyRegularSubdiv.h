#pragma once

#include "Attributes.h"
#include "INeighborQueryable.h"
#include "Indexing.h"

#include "common.h"
#include "Hierarchy.h"

#include <unordered_map>
#include <deque>
#include <array>

#define DefineAccessors(A, member) \
	template<> AttributeTraits<A>::Type& attribute<A>(const FinestIndex& i) { return grid.at(i.node).member.at(i.localIdx); } \
	template<> const AttributeTraits<A>::Type& attribute<A>(const FinestIndex& i) const {return grid.at(i.node).member.at(i.localIdx); } \
	template<> AttributeTraits<A>::Type& attribute<A>(const InnerIndex& i) { return innerLevels[i.level].nodes.at(i.node).member;  } \
	template<> const AttributeTraits<A>::Type& attribute<A>(const InnerIndex& i) const { return innerLevels[i.level].nodes.at(i.node).member; } \

#define DefineAccessorsCell(A, member) \
	template<> typename AttributeTraits<A>::Type& attribute<A>(int i) { return member[i]; } \
	template<> const typename AttributeTraits<A>::Type& attribute<A>(int i) const { return member[i]; } \

namespace HierarchyRegularSubdiv
{
	struct Node
	{
		Node()
			: position(Vector3f::Zero()), normal(Vector3f::Zero()), area(0)
		{ }

		Node(const Node& copy)
			: position(copy.position), normal(copy.normal), area(copy.area), dirField(copy.dirField), posField(copy.posField)
		{
		}

		Vector3f position, normal;
		Vector3f dirField, posField;
		float area;

		template<Attribute A> typename AttributeTraits<A>::Type& attribute() { }
		template<> typename AttributeTraits<Position>::Type& attribute<Position>() { return position; }
		template<> typename AttributeTraits<Normal>::Type& attribute<Normal>() { return normal; }
		template<> typename AttributeTraits<DirField>::Type& attribute<DirField>() { return dirField; }
		template<> typename AttributeTraits<PosField>::Type& attribute<PosField>() { return posField; }
		template<> typename AttributeTraits<Area>::Type& attribute<Area>() { return area; }
	};

	struct FinestIndex
	{
		FinestIndex() { }
		FinestIndex(Int3Index node, uint16_t localIdx = 0)
			: node(node), localIdx(localIdx)
		{ }

		bool operator==(const FinestIndex& other) const { return node == other.node && localIdx == other.localIdx; }
		bool operator<(const FinestIndex& other) const
		{
			for(int i = 0; i < 3; ++i)
				if (node(i) != other.node(i))
					return node(i) < other.node(i);
			return localIdx < other.localIdx;
		}

		Int3Index node;
		uint16_t localIdx;
	};

	struct InnerIndex
	{
		InnerIndex() { }
		InnerIndex(Int3Index node, int level)
			: node(node), level(level)
		{ }

		int level;
		Int3Index node;

		bool operator==(const InnerIndex& other) const { return node == other.node; }
	};

}

namespace std
{
	template <>
	struct hash<HierarchyRegularSubdiv::FinestIndex>
	{
		size_t operator()(const HierarchyRegularSubdiv::FinestIndex& o) const { return std::hash<Int3Index>{}(o.node) + o.localIdx; }
	};

	template <>
	struct hash<HierarchyRegularSubdiv::InnerIndex>
	{
		size_t operator()(const HierarchyRegularSubdiv::InnerIndex& o) const { return std::hash<Int3Index>{}(o.node); }
	};
}

namespace HierarchyRegularSubdiv
{

	struct Cell
	{
		std::vector<Vector3f> position, normal;
		std::vector<Vector3f> dirField, posField;
		std::vector<uint32_t> meshVertex;

		size_t sizeInBytes() const
		{
			size_t size = sizeof(*this);
			size += dirField.size() * sizeof(Vector3f);
			size += normal.size() * sizeof(Vector3f);
			size += posField.size() * sizeof(Vector3f);
			size += position.size() * sizeof(Vector3f);
			size += meshVertex.size() * sizeof(uint32_t);

			return size;
		}

		template<Attribute A> typename AttributeTraits<A>::Type& attribute(int i) { }
		template<Attribute A> const typename AttributeTraits<A>::Type& attribute(int i) const { }
		DefineAccessorsCell(Position, position)
		DefineAccessorsCell(Normal, normal)
		DefineAccessorsCell(DirField, dirField)
		DefineAccessorsCell(PosField, posField)
		DefineAccessorsCell(MeshVertex, meshVertex)
	};
	class SpecificCellAttributeAccess
	{
	public:
		SpecificCellAttributeAccess(Cell& cell, size_t v)
			: cell(cell), v(v)
		{ }

		template<Attribute A> typename AttributeTraits<A>::Type& attribute() { return cell.attribute<A>(v); }

	private:
		Cell& cell;
		size_t v;
	};

	struct LevelInfo
	{
		typedef std::unordered_map<Int3Index, Node> NodeMapType;
		NodeMapType nodes;

		Int3Index toLocalOffset; //specifies the offset to add to a global index in order to transform it to a tree-local index (all positive)

		LevelInfo()
			: toLocalOffset(0, 0, 0)
		{ }

		size_t sizeInBytes() const
		{
			size_t size = sizeof(*this);
			size += nodes.size() * sizeof(NodeMapType::value_type);
			return size;
		}
	};

	typedef std::unordered_map<Int3Index, Cell> GridType;


	class Hierarchy : public AbstractHierarchy<FinestIndex>, public INeighborQueryable<FinestIndex>
	{
	public:

		Hierarchy(const Optimizer& optimizer, ExtractedMesh& extractionResult);
		~Hierarchy();

		// ---- begin public hierarchy interface  ----

		//this is the type referred to by vertices()
		typedef FinestIndex VertexIndex;		

		//access to the data stored in the hierarchy
		template<Attribute A> typename AttributeTraits<A>::Type& attribute(const FinestIndex& i) { throw std::runtime_error("Field retrieval not implemented for type " + std::to_string(A) + "."); }
		template<Attribute A> const typename AttributeTraits<A>::Type& attribute(const FinestIndex& i) const { throw std::runtime_error("Field retrieval not implemented for type " + std::to_string(A) + "."); }
		template<Attribute A> typename AttributeTraits<A>::Type& attribute(const InnerIndex& i) { throw std::runtime_error("Field retrieval not implemented for type " + std::to_string(A) + "."); }
		template<Attribute A> const typename AttributeTraits<A>::Type& attribute(const InnerIndex& i) const { throw std::runtime_error("Field retrieval not implemented for type " + std::to_string(A) + "."); }
		DefineAccessors(Position, position)
		DefineAccessors(Normal, normal)
		DefineAccessors(DirField, dirField)
		DefineAccessors(PosField, posField)
		template<> const AttributeTraits<Area>::Type& attribute<Area>(const FinestIndex& i) const { return 1; }
		template<> const AttributeTraits<Area>::Type& attribute<Area>(const InnerIndex& i) const { return innerLevels[i.level - 1].nodes.at(i.node).area; }
		template<> AttributeTraits<MeshVertex>::Type& attribute<MeshVertex>(const FinestIndex& i) { return grid.at(i.node).meshVertex.at(i.localIdx); }
		template<> const AttributeTraits<MeshVertex>::Type& attribute<MeshVertex>(const FinestIndex& i) const { return grid.at(i.node).meshVertex.at(i.localIdx); }

		//integrate new points in the hierarchy
		void addPoints(const Matrix3Xf& V, const Matrix3Xf& N = Matrix3Xf(0, 0));

		void optimizeFull();

		void reset();		

		class VertexFinestIterator;
		//returns an iterator for all vertices of the finest level	
		ForEachHelper<VertexFinestIterator> vertices() const;		

		template <typename Callback>
		void forEachNeighbor(const FinestIndex& v, const Callback& callback);

		template <typename Callback>
		void forEachNeighbor(const InnerIndex& v, const Callback& callback);

		template <typename Callback> // void(const FinestIndex&)
		void findNearestPointsRadius(const Vector3f& p, Float radius, const Callback& callback) const;
		
		//returns the number of vertices on the finest level of the hierarchy
		size_t vertexCount() const { return mVertexCount; }						

		size_t sizeInBytes() const;		

		// ---- end public hierarchy interface  ----

		// ----       other interfaces          ----
		FinestIndex findClosestPoint(const Vector3f& p) const;
		const Vector3f& p(const FinestIndex& i) const;

		size_t phase(const InnerIndex&) const;

		// ----     end other interfaces        ----

		class VertexFinestIterator : public std::iterator<std::forward_iterator_tag, FinestIndex>
		{
		public:
			VertexFinestIterator(size_t localIdx, GridType::const_iterator grid_current_iterator);

			const VertexFinestIterator& operator++();
			bool operator!=(VertexFinestIterator& other) const;
			FinestIndex operator*();

		private:
			size_t localIdx;
			GridType::const_iterator grid_current_iterator;
		};

		//Converts any iterator (value type is a pair and pair.first is an Int3Index) to an iterator that emits all vertices in the grid cell addressed by pair.first
		template <typename Iterator>
		class VertexFinestIteratorAdaptor : public std::iterator<std::forward_iterator_tag, FinestIndex>
		{
		public:
			VertexFinestIteratorAdaptor(size_t localIdx, Iterator current_iterator, const GridType& grid);

			const VertexFinestIteratorAdaptor& operator++();
			bool operator!=(VertexFinestIteratorAdaptor& other) const;
			FinestIndex operator*();

		private:
			const GridType& grid;
			size_t localIdx;
			Iterator current_iterator;
		};
		template <typename Iterator>
		VertexFinestIteratorAdaptor<Iterator> adaptToFinestIterator(Iterator it) const;

		//Converts any iterator (value type is a pair and pair.first is an Int3Index) to an iterator that emits pair.first and the given level
		template <typename Iterator>
		class VertexInnerIterator : public std::iterator<std::forward_iterator_tag, InnerIndex>
		{
		public:
			VertexInnerIterator(Iterator current, int level);

			const VertexInnerIterator& operator++();
			bool operator!=(VertexInnerIterator& other) const;
			InnerIndex operator*();

		private:
			Iterator current;
			int level;
		};

		template <typename Iterator>
		VertexInnerIterator<Iterator> adaptToInnerIterator(Iterator it, int level) const;

	private:
		template <int DIM>
		typename IntIndexHelper<DIM>::Type parent(const typename IntIndexHelper<DIM>::Type& idx, int childNodeLevel) const;
		template <int DIM>
		void children(const typename IntIndexHelper<DIM>::Type& idx, int parentNodeLevel, typename IntIndexHelper<DIM>::Type& child1, typename IntIndexHelper<DIM>::Type& child2) const;

		void init();
		void initialize(const Matrix3Xf & V, const Matrix3Xf & N);
		void updateHierarchy(const Matrix3Xf & V, const Matrix3Xf N);

		Vector3f origin;
		Int3Index gridMin, gridMax;
		float gridSize = 0;		

		GridType grid; //data structure for the finest level of the hierarchy
		std::vector<LevelInfo> innerLevels; //data structure for all other levels

		struct SubdivisionInfo
		{
			SubdivisionInfo(uint8_t dimension)
				: dimension(dimension)
			{ }

			uint8_t dimension;

		};
		std::deque<SubdivisionInfo> subdivs; //ordered from finest to coarsest
		Int3Index subdivCount;

		size_t mVertexCount;

		template<Attribute A>
		//Copies the specified attribute for all vertices to the next-finer level.
		void copyToFinerLevel(int srcLevel);

		template<Attribute A>
		//Filters the specified attribute for all vertices to the next-coarser level.
		void filterToCoarserLevel(int srcLevel);

		template<Attribute A>
		//Filters the specified attribute for all vertices from the finest level to the coarsest level.
		void filterToCoarsestLevel();

		template <typename Callback> // void(GridType::iterator)
		void forEachCellInRadius(const Int3Index& referenceCell, Float radius, const Vector3f& cellInnerDistanceMin, const Vector3f& cellInnerDistanceMax, const Callback& callback);
		template <typename Callback> // void(GridType::iterator)
		void forEachCellInRadius(const Int3Index& referenceCell, Float radius, const Vector3f& cellInnerDistanceMin, const Vector3f& cellInnerDistanceMax, const Callback& callback) const;
		void findNearestPointsKNN(const Vector3f & p, int k, std::priority_queue<Neighbor<FinestIndex>>& neighborQueue, Float maxRadiusSq = std::numeric_limits<Float>::infinity()) const;
		
		template <typename Callback> // void(const FinestIndex&)
		void findNearestPointsKNN(const Vector3f& p, int k, const Callback& callback, Float maxRadiusSq = std::numeric_limits<Float>::infinity()) const;
		void findNearestPointsKNN(const Vector3f& p, int k, std::vector<FinestIndex>& output, Float maxRadiusSq = std::numeric_limits<Float>::infinity()) const;
	};

	// ----  Template implementations  ----

	template <typename Callback>
	void Hierarchy::forEachNeighbor(const FinestIndex& v, const Callback& callback)
	{
		auto currentP = grid.at(v.node).position.at(v.localIdx);

		findNearestPointsKNN(currentP, 9, [&](const FinestIndex& neighbor)
		{
			if (v.localIdx != neighbor.localIdx || v.node != neighbor.node)
			{
				callback(neighbor);
			}
		}, maxNeighborRadius * maxNeighborRadius);
	}

	template <typename Callback>
	void Hierarchy::forEachNeighbor(const InnerIndex& v, const Callback& callback)
	{
		auto& nodes = innerLevels[v.level].nodes;
		for(int z = -1; z <= 1; ++z)
			for(int y = -1; y <= 1; ++y)
				for (int x = -1; x <= 1; ++x)
				{
					if (x == 0 && y == 0 && z == 0)
						continue;
					Int3Index nIdx = v.node + Int3Index(x, y, z);
					if (nodes.find(nIdx) == nodes.end())
						continue;
					callback(InnerIndex(nIdx, v.level));
				}
	}

	// --------  VertexInnerIterator  ----------

	template <typename Iterator>
	Hierarchy::VertexInnerIterator<Iterator>::VertexInnerIterator(Iterator current, int level)
		: current(current), level(level)
	{ }
	
	template <typename Iterator>
	const Hierarchy::VertexInnerIterator<Iterator>& Hierarchy::VertexInnerIterator<Iterator>::operator++()
	{
		++current;
		return *this;
	}

	template <typename Iterator>
	bool Hierarchy::VertexInnerIterator<Iterator>::operator!=(VertexInnerIterator<Iterator>& other) const
	{
		return current != other.current;
	}

	template <typename Iterator>
	InnerIndex Hierarchy::VertexInnerIterator<Iterator>::operator*()
	{
		return InnerIndex(current->first, level);
	}

	template <typename Iterator>
	Hierarchy::VertexInnerIterator<Iterator> Hierarchy::adaptToInnerIterator(Iterator it, int level) const
	{
		return Hierarchy::VertexInnerIterator<Iterator>(it, level);
	}

	// --------  VertexFinestIteratorAdaptor  ----------

	template <typename Iterator>
	Hierarchy::VertexFinestIteratorAdaptor<Iterator>::VertexFinestIteratorAdaptor(size_t localIdx, Iterator current_iterator, const GridType& grid)
		: localIdx(localIdx), current_iterator(current_iterator), grid(grid)
	{ }

	template <typename Iterator>
	const Hierarchy::VertexFinestIteratorAdaptor<Iterator>& Hierarchy::VertexFinestIteratorAdaptor<Iterator>::operator++()
	{
		++localIdx;
		if (localIdx >= grid.at(current_iterator->first).position.size())
		{
			localIdx = 0;
			++current_iterator;
		}
		return *this;
	}

	template <typename Iterator>
	bool Hierarchy::VertexFinestIteratorAdaptor<Iterator>::operator!=(VertexFinestIteratorAdaptor<Iterator>& other) const
	{
		return localIdx != other.localIdx || current_iterator != other.current_iterator;
	}

	template <typename Iterator>
	FinestIndex Hierarchy::VertexFinestIteratorAdaptor<Iterator>::operator*()
	{
		return FinestIndex(current_iterator->first, localIdx);
	}

	template <typename Iterator>
	Hierarchy::VertexFinestIteratorAdaptor<Iterator> Hierarchy::adaptToFinestIterator(Iterator it) const
	{
		return Hierarchy::VertexFinestIteratorAdaptor<Iterator>(0, it, grid);
	}

	// -----------  Neighbor Search  ----------------

	template <typename Callback>
	void Hierarchy::findNearestPointsKNN(const Vector3f & p, int k, const Callback& callback, Float maxRadiusSq) const
	{
		std::priority_queue<Neighbor<FinestIndex>> neighborQueue;
		findNearestPointsKNN(p, k, neighborQueue, maxRadiusSq);
		while (!neighborQueue.empty())
		{
			callback(neighborQueue.top().idx);
			neighborQueue.pop();
		}
	}

	template <typename Callback>
	void Hierarchy::findNearestPointsRadius(const Vector3f & p, Float radius, const Callback& callback) const
	{
		auto cellIdx = ToIntIndex<3>(p - origin, gridSize);
		Vector3f cellInnerDistanceMin = p - origin - Vector3f::Constant(gridSize).cwiseProduct(cellIdx.cast<float>());
		Vector3f cellInnerDistanceMax = Vector3f::Constant(gridSize) - cellInnerDistanceMin;
		Float radiusSq = radius * radius;
		forEachCellInRadius(cellIdx, radius, cellInnerDistanceMin, cellInnerDistanceMax, [&](GridType::const_iterator n)
		{
			for (int i = 0; i < n->second.position.size(); ++i)
			{
				if ((n->second.attribute<Position>(i) - p).squaredNorm() <= radiusSq)
					callback(FinestIndex(n->first, i));
			}
		});
	}

	template <typename Callback>
	void Hierarchy::forEachCellInRadius(const Int3Index& referenceCell, Float radius, const Vector3f& cellInnerDistanceMin, const Vector3f& cellInnerDistanceMax, const Callback& callback)
	{
		static_cast<const Hierarchy*>(this)->forEachCellInRadius(referenceCell, radius, cellInnerDistanceMin, cellInnerDistanceMax, [&callback, this](auto it)
		{
			callback(grid.erase(it, it)); //just to convert to non-const iterator, see http://stackoverflow.com/a/10669041/1210053
		});
	}

	template <typename Callback>
	void Hierarchy::forEachCellInRadius(const Int3Index& referenceCell, Float radius, const Vector3f& cellInnerDistanceMin, const Vector3f& cellInnerDistanceMax, const Callback& callback) const
	{
		Float radiusSq = radius * radius;

		if (radius <= 1.5 * gridSize)
		{
			//simple box search for small radius
			Int3Index searchMinIdx, searchMaxIdx;
			for (int i = 0; i < 3; ++i)
			{
				searchMinIdx(i) = referenceCell(i) - ceil(std::max(0.0f, radius - cellInnerDistanceMin(i)) / gridSize);
				searchMaxIdx(i) = referenceCell(i) + ceil(std::max(0.0f, radius - cellInnerDistanceMax(i)) / gridSize);

				if (searchMinIdx(i) < gridMin(i))
					searchMinIdx(i) = gridMin(i);
				if (searchMaxIdx(i) > gridMax(i))
					searchMaxIdx(i) = gridMax(i);
			}
			Float radiusSq = radius * radius;
			for (int x = searchMinIdx(0); x <= searchMaxIdx(0); ++x)
				for (int y = searchMinIdx(1); y <= searchMaxIdx(1); ++y)
					for (int z = searchMinIdx(2); z <= searchMaxIdx(2); ++z)
					{
						Int3Index cellIdx(x, y, z);
						auto cell = grid.find(cellIdx);
						if (cell != grid.end())
						{
							callback(cell);
						}
					}
		}
		else
		{
			//more advanced spherical search for large radius
			int minX = referenceCell.x() - ceil(std::max(0.0f, radius - cellInnerDistanceMin.x()) / gridSize);
			if (minX < gridMin.x())
				minX = gridMin.x();
			int maxX = referenceCell.x() + ceil(std::max(0.0f, radius - cellInnerDistanceMax.x()) / gridSize);
			if (maxX > gridMax.x())
				maxX = gridMax.x();
			for (int x = minX; x <= maxX; ++x)
			{
				Float dx = 0; //lower bound
				if (x < referenceCell.x())
					dx = ((referenceCell.x() - x - 1) * gridSize + cellInnerDistanceMin.x());
				else if (x > referenceCell.x())
					dx = ((x - referenceCell.x() - 1) * gridSize + cellInnerDistanceMax.x());
				Float dxSq = dx * dx;
				Float radiusForY = sqrt(radiusSq - dxSq);
				int minY = referenceCell.y() - ceil(std::max(0.0f, radiusForY - cellInnerDistanceMin.y()) / gridSize);
				if (minY < gridMin.y())
					minY = gridMin.y();
				int maxY = referenceCell.y() + ceil(std::max(0.0f, radiusForY - cellInnerDistanceMax.y()) / gridSize);
				if (maxY > gridMax.y())
					maxY = gridMax.y();
				for (int y = minY; y <= maxY; ++y)
				{
					Float dy = 0; //lower bound
					if (y < referenceCell.y())
						dy = ((referenceCell.y() - y - 1) * gridSize + cellInnerDistanceMin.y());
					else if (y > referenceCell.y())
						dy = ((y - referenceCell.y() - 1) * gridSize + cellInnerDistanceMax.y());
					Float dySq = dy * dy;
					Float radiusForZ = sqrt(radiusSq - dxSq - dySq);
					int minZ = referenceCell.z() - ceil(std::max(0.0f, radiusForZ - cellInnerDistanceMin.z()) / gridSize);
					if (minZ < gridMin.z())
						minZ = gridMin.z();
					int maxZ = referenceCell.z() + ceil(std::max(0.0f, radiusForZ - cellInnerDistanceMax.z()) / gridSize);
					if (maxZ > gridMax.z())
						maxZ = gridMax.z();
					for (int z = minZ; z <= maxZ; ++z)
					{
						auto cell = grid.find(Int3Index(x, y, z));
						if (cell != grid.end())
						{
							callback(cell);
						}
					}
				}
			}
		}
	}

}

#undef DefineAccessors
#undef DefineAccessorsCell
