/*
	hierarchy.cpp: Code to generate unstructured multi-resolution hierarchies
	from meshes or point clouds

	This file is part of the implementation of

		Instant Field-Aligned Meshes
		Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

	All rights reserved. Use of this source code is governed by a
	BSD-style license that can be found in the LICENSE.txt file.
*/

#include <tbb/tbb.h>
#include <random>

#include "HierarchyUnstructured.h"
#include "AttributeConsistency.h"
#include "field.h"
#include "HierarchyOptimizationHelper.h"

#include "Optimizer.h"

using namespace HierarchyUnstructured;

AdjacencyMatrix HierarchyUnstructured::downsample_graph(const AdjacencyMatrix adj, const std::vector<Vector3f> &V,
	const std::vector<Vector3f> &N, const std::vector<float> &A,
	std::vector<Vector3f> &V_p, std::vector<Vector3f> &N_p, std::vector<float> &A_p,
	MatrixXu &to_upper, VectorXu &to_lower,
	bool deterministic)
{
	struct Entry {
		uint32_t i, j;
		float order;
		inline Entry() { };
		inline Entry(uint32_t i, uint32_t j, float order) : i(i), j(j), order(order) { }
		inline bool operator<(const Entry &e) const { return order > e.order; }
	};

	uint32_t nLinks = adj[V.size()] - adj[0];
	Entry *entries = new Entry[nLinks];
	Timer<> timer;
	std::cout << "  Collapsing .. ";
	std::cout.flush();

	tbb::parallel_for(
		tbb::blocked_range<uint32_t>(0u, (uint32_t)V.size(), GRAIN_SIZE),
		[&](const tbb::blocked_range<uint32_t> &range) {
		for (uint32_t i = range.begin(); i != range.end(); ++i) {
			uint32_t nNeighbors = adj[i + 1] - adj[i];
			uint32_t base = adj[i] - adj[0];
			for (uint32_t j = 0; j < nNeighbors; ++j) {
				size_t k = adj[i][j];
				Float dp = N[i].dot(N[k]);
				Float ratio = A[i] > A[k] ? (A[i] / A[k]) : (A[k] / A[i]);
				entries[base + j] = Entry(i, k, dp * ratio);
			}
		}
		//SHOW_PROGRESS_RANGE(range, V.cols(), "Downsampling graph (1/6)");
	}
	);

	//if (progress)
	//	progress("Downsampling graph (2/6)", 0.0f);

	//if (deterministic)
	//    pss::parallel_stable_sort(entries, entries + nLinks, std::less<Entry>());
	//else
	tbb::parallel_sort(entries, entries + nLinks, std::less<Entry>());

	std::vector<bool> mergeFlag(V.size(), false);

	uint32_t nCollapsed = 0;
	for (uint32_t i = 0; i < nLinks; ++i) {
		const Entry &e = entries[i];
		if (mergeFlag[e.i] || mergeFlag[e.j])
			continue;
		mergeFlag[e.i] = mergeFlag[e.j] = true;
		entries[nCollapsed++] = entries[i];
	}
	uint32_t vertexCount = V.size() - nCollapsed;

	/* Allocate memory for coarsened graph */
	V_p.resize(vertexCount);
	N_p.resize(vertexCount);
	A_p.resize(vertexCount);
	to_upper.resize(2, vertexCount);
	to_lower.resize(V.size());

	tbb::parallel_for(
		tbb::blocked_range<uint32_t>(0u, (uint32_t)nCollapsed, GRAIN_SIZE),
		[&](const tbb::blocked_range<uint32_t> &range) {
		for (uint32_t i = range.begin(); i != range.end(); ++i) {
			const Entry &e = entries[i];
			const Float area1 = A[e.i], area2 = A[e.j], surfaceArea = area1 + area2;
			if (surfaceArea > RCPOVERFLOW)
				V_p[i] = (V[e.i] * area1 + V[e.j] * area2) / surfaceArea;
			else
				V_p[i] = (V[e.i] + V[e.j]) * 0.5f;
			Vector3f normal = N[e.i] * area1 + N[e.j] * area2;
			Float norm = normal.norm();
			N_p[i] = norm > RCPOVERFLOW ? Vector3f(normal / norm)
				: Vector3f::UnitX();
			A_p[i] = surfaceArea;
			to_upper.col(i) << e.i, e.j;
			to_lower[e.i] = i; to_lower[e.j] = i;
		}
		//SHOW_PROGRESS_RANGE(range, nCollapsed, "Downsampling graph (3/6)");
	}
	);

	delete[] entries;

	std::atomic<int> offset(nCollapsed);
	tbb::blocked_range<uint32_t> range(0u, (uint32_t)V.size(), GRAIN_SIZE);

	auto copy_uncollapsed = [&](const tbb::blocked_range<uint32_t> &range) {
		for (uint32_t i = range.begin(); i != range.end(); ++i) {
			if (!mergeFlag[i]) {
				uint32_t idx = offset++;
				V_p[idx] = V[i];
				N_p[idx] = N[i];
				A_p[idx] = A[i];
				to_upper.col(idx) << i, INVALID;
				to_lower[i] = idx;
			}
		}
		//SHOW_PROGRESS_RANGE(range, V.cols(), "Downsampling graph (4/6)");
	};

	if (deterministic)
		copy_uncollapsed(range);
	else
		tbb::parallel_for(range, copy_uncollapsed);

	VectorXu neighborhoodSize(V_p.size() + 1);

	tbb::parallel_for(
		tbb::blocked_range<uint32_t>(0u, (uint32_t)V_p.size(), GRAIN_SIZE),
		[&](const tbb::blocked_range<uint32_t> &range) {
		std::vector<size_t> scratch;
		for (uint32_t i = range.begin(); i != range.end(); ++i) {
			scratch.clear();

			for (int j = 0; j < 2; ++j) {
				uint32_t upper = to_upper(j, i);
				if (upper == INVALID)
					continue;
				for (size_t *link = adj[upper]; link != adj[upper + 1]; ++link)
					scratch.push_back(to_lower[*link]);
			}

			std::sort(scratch.begin(), scratch.end());
			uint32_t id = INVALID, size = 0;
			for (const auto &link : scratch) {
				if (id != link && link != i) {
					id = link;
					++size;
				}
			}
			neighborhoodSize[i + 1] = size;
		}
		//SHOW_PROGRESS_RANGE(range, V_p.cols(), "Downsampling graph (5/6)");
	}
	);

	neighborhoodSize[0] = 0;
	for (uint32_t i = 0; i < neighborhoodSize.size() - 1; ++i)
		neighborhoodSize[i + 1] += neighborhoodSize[i];

	uint32_t nLinks_p = neighborhoodSize[neighborhoodSize.size() - 1];
	AdjacencyMatrix adj_p = new size_t*[V_p.size() + 1];
	size_t *links = new size_t[nLinks_p];
	for (uint32_t i = 0; i < neighborhoodSize.size(); ++i)
		adj_p[i] = links + neighborhoodSize[i];

	tbb::parallel_for(
		tbb::blocked_range<uint32_t>(0u, (uint32_t)V_p.size(), GRAIN_SIZE),
		[&](const tbb::blocked_range<uint32_t> &range) {
		std::vector<size_t> scratch;
		for (uint32_t i = range.begin(); i != range.end(); ++i) {
			scratch.clear();

			for (int j = 0; j < 2; ++j) {
				uint32_t upper = to_upper(j, i);
				if (upper == INVALID)
					continue;
				for (size_t *link = adj[upper]; link != adj[upper + 1]; ++link)
					scratch.push_back(to_lower[*link]);
			}
			std::sort(scratch.begin(), scratch.end());
			size_t *dest = adj_p[i];
			uint32_t id = INVALID;
			for (const auto &link : scratch) {
				if (link != i) {
					if (id != link) {
						*dest++ = link;
						id = link;
					}
					//else {
					//	dest[-1].weight += link.weight;
					//}
				}
			}
		}
		//SHOW_PROGRESS_RANGE(range, V_p.cols(), "Downsampling graph (6/6)");
	}
	);
	std::cout << "done. (" << V.size() << " -> " << V_p.size() << " vertices, took "
		<< timeString(timer.value()) << ")" << std::endl;
	return adj_p;
}

Hierarchy::Hierarchy(const Optimizer& optimizer, ExtractedMesh& extractionResult)
	: AbstractHierarchy(optimizer, extractionResult), kdTree(nullptr)
{
	//if (sizeof(Link) != 12)
	//	throw std::runtime_error("Adjacency matrix entries are not packed! Investigate compiler settings.");
	mA.reserve(MAX_DEPTH + 1);
	mV.reserve(MAX_DEPTH + 1);
	mN.reserve(MAX_DEPTH + 1);
	mDirField.reserve(MAX_DEPTH + 1);
	mPosField.reserve(MAX_DEPTH + 1);
	mAdj.reserve(MAX_DEPTH + 1);
	mToFiner.reserve(MAX_DEPTH);
	mToCoarser.reserve(MAX_DEPTH);
}

Hierarchy::~Hierarchy()
{
	for (size_t i = 0; i < mAdj.size(); ++i) {
		delete[] mAdj[i][0];
		delete[] mAdj[i];
	}
	mAdj.clear(); mV.clear(); mDirField.clear();
	mPosField.clear(); mN.clear(); mA.clear();
	mToFiner.clear(); mToCoarser.clear();
	if (kdTree)
		delete kdTree;
}

void Hierarchy::build()
{
	const bool deterministic = false;
	
	std::cout << "Building multiresolution hierarchy .." << std::endl;
	Timer<> timer;
	for (int i = 0; i < MAX_DEPTH; ++i) 
	{
		std::vector<Vector3f> N_p, V_p;
		std::vector<float> A_p;
		MatrixXu toUpper;
		VectorXu toLower;

		AdjacencyMatrix adj_p =
			downsample_graph(mAdj[i], mV[i], mN[i], mA[i], V_p, N_p, A_p,
				toUpper, toLower, deterministic);

		mAdj.push_back(std::move(adj_p));
		mV.push_back(std::move(V_p));
		mN.push_back(std::move(N_p));
		mA.push_back(std::move(A_p));
		mToFiner.push_back(std::move(toUpper));
		mToCoarser.push_back(std::move(toLower));
		if (mV[mV.size() - 1].size() == 1)
			break;
	}
	std::cout << "Hierarchy construction took " << timeString(timer.value()) << "." << std::endl;
}

ForEachHelper<Hierarchy::VertexIterator> Hierarchy::vertices(int level) const
{
	return ForEachHelper<VertexIterator>(VertexIterator(0, level), VertexIterator(mV[level].size(), level));
}

void Hierarchy::addPoints(const Matrix3Xf & V, const Matrix3Xf & N)
{
	//rebuild the entire structure from scratch

	mV.resize(1);
	mN.resize(1);
	mA.resize(1);
	mToFiner.clear(); mToCoarser.clear();

	//copy new values

	size_t oldColumns = mV[0].size();
	size_t newColumns = oldColumns + V.cols();
	mV[0].resize(newColumns);
	mN[0].resize(newColumns);
	mA[0].resize(newColumns);
#pragma omp parallel for
	for (int i = 0; i < V.cols(); ++i)
	{
		mV[0][oldColumns + i] = V.col(i);
		mN[0][oldColumns + i] = N.col(i); //TODO: estimate normals if not present	
		mA[0][oldColumns + i] = 1;
	}

	//Expand bounding box
	bbox.expand(V);

	//build kd-tree
	if (kdTree)
		delete kdTree;
	kdTree = new kdTreeType(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams());
	kdTree->buildIndex();

	//calcualate adjacency
	for (size_t i = 0; i < mAdj.size(); ++i) {
		delete[] mAdj[i][0];
		delete[] mAdj[i];
	}
	mAdj.resize(1);
	mAdj[0] = generate_adjacency_matrix_knn(mV[0], *this, 8);

	build();

	resetSolution();

	optimizeFull();

	PositionsChanged();
	NormalsChanged();
	AdjacencyChanged();
}

void Hierarchy::optimizeFull()
{
	for (int iLevel = mV.size() - 1; iLevel >= 0; --iLevel)
	{
		auto v = vertices(iLevel);
		auto set = prepareForOptimization(v.begin(), v.end(), *this);
		set.optimize(optimizer);

		if (iLevel > 0)
		{
			copyToFinerLevel<DirField>(iLevel);
			copyToFinerLevel<PosField>(iLevel);
		}
	}

	DirFieldChanged();
	PosFieldChanged();
}

void Hierarchy::reset()
{
	throw std::runtime_error("Not implemented");
}

void init_random_tangent(const std::vector<Vector3f> &N, std::vector<Vector3f> &dirField) {
	dirField.resize(N.size());
	tbb::parallel_for(tbb::blocked_range<uint32_t>(0u, (uint32_t)N.size()),
		[&](const tbb::blocked_range<uint32_t> &range) {
		std::mt19937 rng(range.begin());
		std::uniform_real_distribution<float> dist(0, 2 * (float)M_PI);
		for (uint32_t i = range.begin(); i != range.end(); ++i) {
			Vector3f s, t;
			coordinate_system(N[i], s, t);
			float angle = dist(rng);
			dirField[i] = s * std::cos(angle) + t * std::sin(angle);
		}
	}
	);	
}

void init_random_position(const std::vector<Vector3f> &P, const std::vector<Vector3f> &N, std::vector<Vector3f> &posField, Float scale) {
	posField.resize(N.size());
	tbb::parallel_for(tbb::blocked_range<uint32_t>(0u, (uint32_t)N.size()),
		[&](const tbb::blocked_range<uint32_t> &range) {
		std::mt19937 rng(range.begin());
		std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
		for (uint32_t i = range.begin(); i != range.end(); ++i) {
			Vector3f s, t;
			coordinate_system(N[i], s, t);
			float x = dist(rng),
				y = dist(rng);
			posField[i] = P[i] + (s*x + t*y)*scale;
		}
	}
	);
}

void Hierarchy::resetSolution()
{
	std::cout << "Setting to random solution .. ";
	std::cout.flush();
	Timer<> timer;
	if (mDirField.size() != mV.size()) {
		mDirField.resize(mV.size());
		mPosField.resize(mV.size());
	}
	for (size_t i = 0; i < mV.size(); ++i) {
		mDirField[i].resize(mV[i].size());
		mPosField[i].resize(mV[i].size());		
	}
	int coarsestLevel = mV.size() - 1;
	init_random_tangent(mN[coarsestLevel], mDirField[coarsestLevel]);
	init_random_position(mV[coarsestLevel], mN[coarsestLevel], mPosField[coarsestLevel], optimizer.meshSettings().scale);
	//mFrozenO = mFrozenQ = false;
	std::cout << "done. (took " << timeString(timer.value()) << ")" << std::endl;
}

class SpecificAttributeAccess
{
public:
	SpecificAttributeAccess(Hierarchy& h, const Hierarchy::VertexIndex& v)
		: h(h), v(v)
	{ }

	template<Attribute A> typename AttributeTraits<A>::Type attribute();
	template<> typename AttributeTraits<Position>::Type attribute<Position>() { return h.attribute<Position>(v); }
	template<> typename AttributeTraits<Normal>::Type attribute<Normal>() { return h.attribute<Normal>(v); }
	template<> typename AttributeTraits<DirField>::Type attribute<DirField>() { return h.attribute<DirField>(v); }
	template<> typename AttributeTraits<PosField>::Type attribute<PosField>() { return h.attribute<PosField>(v); }
	template<> typename AttributeTraits<Area>::Type attribute<Area>() { return h.attribute<Area>(v); }

private:
	Hierarchy& h;
	Hierarchy::VertexIndex v;
};

template<Attribute A>
void Hierarchy::copyToFinerLevel(int srcLevel)
{
	assert(srcLevel > 0 && srcLevel < mV.size());
	int dstLevel = srcLevel - 1;

	const MatrixXu &toFiner = mToFiner[srcLevel - 1];

	vertices(srcLevel).processInParallel([&](const auto& j)
	{
		for (int k = 0; k<2; ++k) 
		{
			auto dest = toFiner(k, j.idx);
			if (dest == INVALID)
				continue;
			this->attribute<A>(Index(dest, dstLevel)) = this->attribute<A>(j);

			AttributeConsistency<A>::makeConsistent(SpecificAttributeAccess(*this, Index(dest, srcLevel - 1)));
		}
	});
}

template void Hierarchy::copyToFinerLevel<Normal>(int srcLevel);
template void Hierarchy::copyToFinerLevel<DirField>(int srcLevel);
template void Hierarchy::copyToFinerLevel<PosField>(int srcLevel);

template<Attribute A>
void Hierarchy::filterToCoarserLevel(int srcLevel)
{
	assert(srcLevel < mV.size() - 1);

	int dstLevel = srcLevel + 1;

	const MatrixXu &toFiner = mToFiner[srcLevel];

	vertices(dstLevel).processInParallel([&](const auto& j)
	{
		size_t src1 = toFiner(0, j.idx);
		size_t src2 = toFiner(1, j.idx);

		if (src2 == INVALID)
			this->attribute<A>(j) = this->attribute<A>(Index(src1, srcLevel));
		else
		{
			Float a1 = this->attribute<Area>(Index(src1, srcLevel));
			Float a2 = this->attribute<Area>(Index(src2, srcLevel));
			//TODO: This will only work for attributes without symmetry!
			this->attribute<A>(j) = a1 * this->attribute<A>(Index(src1, srcLevel)) + a2 * this->attribute<A>(Index(src2, srcLevel)) / (a1 + a2);
		}
			
		AttributeConsistency<A>::makeConsistent(SpecificAttributeAccess(*this, j));
	});
}

template<Attribute A>
void Hierarchy::filterToCoarsestLevel()
{
	for (int i = 0; i < mV.size() - 1; ++i)
		filterToCoarserLevel<A>(i);
}

template void Hierarchy::filterToCoarsestLevel<Normal>();
//Do not instantiate for symmetrical attributes without adapting copyToCoarserLevel()!

class CallbackDistanceResult
{
public:
	const float radius;
	const std::function<void(size_t)>& callback;

	CallbackDistanceResult(Float radius, const std::function<void(size_t)>& callback)
		: radius(radius), callback(callback)
	{}

	void init() {}
	void clear() { }
	size_t size() { return 0; }
	bool full() const { return true; }

	float worstDist() const { return radius; }

	void addPoint(float distance, size_t index)
	{
		callback(index);
	}
};

void Hierarchy::findNearestPointsRadius(const Vector3f& p, Float radius, const std::function<void(size_t)>& callback) const
{
	nanoflann::SearchParams params(0, 0, false);

	CallbackDistanceResult result(radius * radius, callback);
	kdTree->radiusSearchCustomCallback(p.data(), result, params);
}

void Hierarchy::findNearestPointsKNN(const Vector3f& p, int k, const std::function<void(const size_t&)>& callback, Float maxRadiusSq) const
{
	nanoflann::SearchParams params(0, 0, false);

	if (k > mV[0].size())
		k = mV[0].size();

	std::vector<size_t> indices(k);
	std::vector<float> distances(k);

	kdTree->knnSearch(p.data(), k, indices.data(), distances.data());

	for (int i = 0; i < k; ++i)
		callback(indices.at(i));
}

void Hierarchy::findNearestPointsKNN(const Vector3f & p, int k, std::vector<size_t>& output) const
{
	nanoflann::SearchParams params(0, 0, false);

	if (k > mV[0].size())
		k = mV[0].size();

	std::vector<float> distances(k);
	output.resize(output.size() + k);

	kdTree->knnSearch(p.data(), k, output.data() + output.size() - k, distances.data());
}

size_t Hierarchy::findClosestPoint(const Vector3f & p) const
{
	nanoflann::SearchParams params(0, 0, false);

	if (mV[0].size() == 0)
		throw std::runtime_error("Cannot find the closest point from an empty Scan.");
	
	size_t index;
	float distance;

	kdTree->knnSearch(p.data(), 1, &index, &distance);

	return index;
}

const Vector3f& Hierarchy::p(const size_t& i) const
{
	return mV[0][i];
}

size_t Hierarchy::sizeInBytes() const
{
	size_t size = sizeof(*this);
	
	auto tmpName = tmpnam(nullptr);
	FILE* kdDump = fopen(tmpName, "wb");
	kdTree->saveIndex(kdDump);
	size += ftell(kdDump);
	fclose(kdDump);
	_unlink(tmpName);

	for (int i = 0; i < mV.size(); ++i) 
	{
		size += (mAdj[i][mV[i].size()] - mAdj[i][0]) * sizeof(size_t) + mV[i].size() * sizeof(size_t *);
		size += mV[i].size() * sizeof(Vector3f);
		size += mN[i].size() * sizeof(Vector3f);
		size += mA[i].size() * sizeof(Float);
		size += mDirField[i].size() * sizeof(Vector3f);//::sizeInBytes(mDirField[i]);
		size += mDirField[i].size() * sizeof(Vector3f);//::sizeInBytes(mPosField[i]);
		
	}
	for (int i = 0; i < mV.size() - 1; ++i)
	{
		size += ::sizeInBytes(mToFiner[i]);
		size += ::sizeInBytes(mToCoarser[i]);
	}

	return size;
}


// Must return the number of data points
size_t Hierarchy::kdtree_get_point_count() const { return mV[0].size(); }

// Returns the L2 distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
Float Hierarchy::kdtree_distance(const Float *p1, const size_t idx_p2, size_t size) const
{
	return (mV[0][idx_p2] - Vector3f(p1[0], p1[1], p1[2])).squaredNorm();	
}

// Returns the dim'th component of the idx'th point in the class:
Float Hierarchy::kdtree_get_pt(const size_t idx, int dim) const { return mV[0][idx](dim); }