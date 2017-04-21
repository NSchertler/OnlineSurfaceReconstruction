#pragma once

#include "Attributes.h"
#include "INeighborQueryable.h"

#include "common.h"
#include "Indexing.h"

#include <boost/signals2.hpp>

#include <unordered_map>
#include <array>

class Optimizer;

namespace HierarchyOctree
{
	typedef IntIndexHelper<3, int32_t>::Type Index;

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

		int level;
		Index idx;

		std::vector<Node*> neighbors;

		template<Attribute A> typename AttributeTraits<A>::Type& attribute() { }
		template<> typename AttributeTraits<Position>::Type& attribute<Position>() { return position; }
		template<> typename AttributeTraits<Normal>::Type& attribute<Normal>() { return normal; }
		template<> typename AttributeTraits<DirField>::Type& attribute<DirField>() { return dirField; }
		template<> typename AttributeTraits<PosField>::Type& attribute<PosField>() { return posField; }
		template<> typename AttributeTraits<Area>::Type& attribute<Area>() { return area; }
	};	


	class Hierarchy : public INeighborQueryable<Node*>
	{
	public:

		Hierarchy();

		// ---- begin public hierarchy interface  ----

		//signals that are emitted whenever data are changed
		boost::signals2::signal<void()> PositionsChanged;
		boost::signals2::signal<void()> NormalsChanged;
		boost::signals2::signal<void()> AdjacencyChanged;
		boost::signals2::signal<void()> DirFieldChanged;
		boost::signals2::signal<void()> PosFieldChanged;

		//integrate new points in the hierarchy
		void addPoints(const Matrix3Xf& V, const Matrix3Xf& N = Matrix3Xf(0, 0));

		void setMaxNeighborRadius(Float r) { }

		//access to the data stored in the hierarchy
		template<Attribute A> typename AttributeTraits<A>::Type& attribute(Node* i) { return i->attribute<A>(); }
		template<Attribute A> const typename AttributeTraits<A>::Type& attribute(const Node* i) const { return i->attribute<A>(); }

		void optimizeFull();

		class PhasesIterator;
		//returns an independence partitioning of the vertices
		ForEachHelper<PhasesIterator> phases();

		//returns an iterator for all vertices of a level	
		ForEachHelper<std::vector<Node*>::const_iterator> vertices() const;

		void set_optimizer(Optimizer* o) { optimizer = o; }

		//returns an iterator for all neighbors of a given vertex
		ForEachHelper<std::vector<Node*>::const_iterator> neighbors(const Node* v) const;

		//Required for Disjoint Set handling during extraction
		//converts the hierarchy-specific index of a vertex to a consecutive zero-based index in the range [0, vertexCout)
		size_t toConsecutiveIndex(const Node* v);
		//{ if (consecutiveDirty) updateConsecutive(); return mToConsecutive.at(v); }
		//converts the consecutive zero-based index back to a hierarchy-specific index
		Node* fromConsecutiveIndex(size_t v);
		//{ if (consecutiveDirty) updateConsecutive(); return mFromConsecutive.at(v); }
		//returns the number of vertices on the lowest level of the hierarchy
		size_t vertexCount() const;

		const Vector3f& bbxMin() const { return mBbxMin; }
		const Vector3f& bbxMax() const { return mBbxMax; }

		Node* findClosestPoint(const Vector3f& p) const { return nullptr; }
		const Vector3f p(const Node* i) const { return Vector3f::Zero(); };

		size_t sizeInBytes() const { return 0; }

		template<Attribute A>
		//Filters the specified attribute for all vertices from the finest level to the coarsest level.
		void filterToCoarsestLevel() { }

		class PhasesIterator
		{
		public:
			PhasesIterator(std::vector<Node*>::const_iterator it)
				: it(it)
			{ }

			const PhasesIterator& operator++() { ++it; return *this; }
			bool operator!=(PhasesIterator& other) const { return it != other.it; }
			ForEachHelper<std::vector<Node*>::const_iterator> operator*() { return ForEachHelper<std::vector<Node*>::const_iterator>(it, it + 1); }

		private:
			std::vector<Node*>::const_iterator it;
		};

	private:
		const Optimizer* optimizer;

		Vector3f mBbxMin, mBbxMax;
		float coarsestGridSize;

		Vector3f origin;

		typedef std::unordered_map<Index, Node*> LevelType;

		void buildTree(const std::vector<std::pair<Vector3f, Vector3f>>& points, int level, const Vector3f& bbxMin, float gridSize);

		void findExistingChildren(const Index & idx, int level, std::vector<LevelType::iterator>& children);

		void findExistingLeaves(const Index & idx, int level, std::vector<std::pair<int, Index>>& leaves);

		void findTerminalNodes(const Index & current, int currentLevel);

		
		std::vector<LevelType> levels;
		std::vector<Node*> terminals, leaves;

		Index parent(const typename Index& idx) const;
		void children(const Index& idx, std::array<typename Index, 8>& c) const;
	};
}