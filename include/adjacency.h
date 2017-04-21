/*
    aabb.h -- functionality for creating adjacency matrices together with
              uniform or cotangent weights. Also contains data structures
              used to store integer variables.

    This file is part of the implementation of

        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#pragma once

#include "common.h"
#include "INeighborQueryable.h"
#include <vector>

class Scan;

typedef size_t** AdjacencyMatrix;

class NeighborIterator
{
public:
	NeighborIterator(size_t* p)
		: current(p)
	{ }

	const NeighborIterator& operator++() { ++current; return *this; }
	bool operator!=(NeighborIterator& other) const { return current != other.current; }
	size_t operator*() { return *current; }

private:
	size_t* current;
};

extern AdjacencyMatrix generate_adjacency_matrix_uniform(const MatrixXu &F, size_t vertexCount);

extern AdjacencyMatrix generate_adjacency_matrix_knn(const std::vector<Vector3f> &V, const IKNNQueryable<size_t>& queryable, int k);