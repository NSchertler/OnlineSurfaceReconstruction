/*
    aabb.cpp -- functionality for creating adjacency matrices together with
                uniform or cotangent weights. Also contains data structures
                used to store integer variables.

    This file is part of the implementation of

        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <set>
#include <unordered_set>
#include <map>
#include <tbb/tbb.h>

#include "adjacency.h"
#include "common.h"
#include "Scan.h"
#include "IndentationLog.h"

#include <nanoflann.hpp>
//#include <flann/flann.hpp>
//#include <nabo/nabo.h>
//#include <kdTree.h>
//#include "bvh.h"
//#include <unordered_map>

AdjacencyMatrix generate_adjacency_matrix_uniform(
    const MatrixXu &F, size_t vertexCount)
{
    
	VectorXu rowBeginIndices(vertexCount + 1);
    TimedBlock b("Generating adjacency matrix ..");    

	//very inefficient procedure for testing purposes...

	std::vector<std::unordered_set<uint32_t>> neighbors(vertexCount); //stores the neighbors for each vertex

	//tbb::parallel_for(
	//	tbb::blocked_range<uint32_t>(0u, (uint32_t)F.cols(), GRAIN_SIZE),
	//	[&](const tbb::blocked_range<uint32_t> &range)
	{
		//for (uint32_t i = range.begin(); i != range.end(); ++i)
		for (uint32_t i = 0; i != F.cols(); ++i)
		{
			auto& face = F.col(i);
			neighbors.at(face[0]).insert(face[1]);
			neighbors.at(face[0]).insert(face[2]);
			neighbors.at(face[1]).insert(face[0]);
			neighbors.at(face[1]).insert(face[2]);
			neighbors.at(face[2]).insert(face[0]);
			neighbors.at(face[2]).insert(face[1]);					
		}
	}
	//);

	rowBeginIndices[0] = 0;
    for (uint32_t i=0; i<vertexCount; ++i)
		rowBeginIndices[i+1] = rowBeginIndices[i] + neighbors.at(i).size();

    AdjacencyMatrix adj = new size_t*[vertexCount + 1];
    uint32_t nLinks = rowBeginIndices[rowBeginIndices.size()-1];
	size_t *links = new size_t[nLinks];
    for (uint32_t i=0; i<rowBeginIndices.size(); ++i)
        adj[i] = links + rowBeginIndices[i];

	tbb::parallel_for(0u, (uint32_t)vertexCount,
		[&](uint32_t i)
	{
		size_t *ptr = adj[i];
		for (auto neighbor : neighbors.at(i))
			*ptr++ = neighbor;
	}
	);

    return adj;
}
AdjacencyMatrix generate_adjacency_matrix_knn(const std::vector<Vector3f> &V, const IKNNQueryable<size_t>& queryable, int k)
{

	VectorXu rowBeginIndices(V.size() + 1);
	TimedBlock b("Generating adjacency matrix ..");
	
	std::vector<std::vector<size_t>> neighbors(V.size()); //stores the neighbors for each vertex

	tbb::parallel_for(
		0u, (uint32_t)V.size(),
		[&](uint32_t i)
	{
		Vector3f current = V[i];
		
		queryable.findNearestPointsKNN(current, 8, [&](size_t n)
		{
			neighbors[i].push_back(n);
		});
	}
	);	

	rowBeginIndices[0] = 0;
	for (uint32_t i = 0; i<V.size(); ++i)
		rowBeginIndices[i + 1] = rowBeginIndices[i] + neighbors[i].size() - 1;

	AdjacencyMatrix adj = new size_t*[V.size() + 1];
	uint32_t nLinks = rowBeginIndices[rowBeginIndices.size() - 1];
	size_t *links = new size_t[nLinks];
	for (uint32_t i = 0; i<rowBeginIndices.size(); ++i)
		adj[i] = links + rowBeginIndices[i];

	tbb::parallel_for(0u, (uint32_t)V.size(),
		[&](uint32_t i)
	{
		size_t *ptr = adj[i];
		for (auto neighbor : neighbors[i])
		{
			if(neighbor != i)
				*ptr++ = neighbor;
		}
	}
	);

	return adj;
}