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

#include "HierarchyOptimizationHelper.h"

#include <tbb/tbb.h>
#include <random>

#include "Parallelization.h"

using namespace osr;

ColorData osr::generateGraphColoring(int vertexCount, const std::vector<std::vector<size_t>>& neighbors, std::vector<uint8_t>& color)
{
	//color the graph

	const uint8_t INVALID_COLOR = 0xFF;

	Timer<> timer;
	std::cout << "Coloring " << vertexCount << " vertices .. ";

	/* Generate a permutation */
	std::vector<uint32_t> perm(vertexCount);
	std::vector<tbb::spin_mutex> mutex(vertexCount);
#pragma omp parallel for
	for (int i = 0; i < vertexCount; ++i)
		perm[i] = i;
	parallel_shuffle(perm, mutex);

	std::fill(color.begin(), color.end(), INVALID_COLOR);
	color.resize(vertexCount, INVALID_COLOR);
	ColorData colorData = tbb::parallel_reduce
	(
		tbb::blocked_range<uint32_t>(0u, vertexCount),
		ColorData(),
		[&](const tbb::blocked_range<uint32_t> &range, ColorData colorData) -> ColorData
	{
		bool possible_colors[MAX_COLORS];

		for (uint32_t pidx = range.begin(); pidx != range.end(); ++pidx)
		{
			uint32_t i = perm[pidx];

			size_t lastN = 0;
			for (int j = 0; j < neighbors[i].size(); ++j)
			{
				auto n = neighbors[i][j];
				if (lastN <= i && i < n)
					mutex[i].lock();
				lastN = n;
				if (n >= vertexCount)
					break;
				mutex[n].lock(); //neighbors are sorted by index, which avoids deadlocks
			}
			if (lastN <= i)
				mutex[i].lock();

			std::fill(possible_colors, possible_colors + colorData.nColors, true);

			for (auto n : neighbors[i])
			{
				if (n >= vertexCount)
					break;
				uint8_t c = color[n];
				if (c != INVALID_COLOR)
				{
					while (c >= colorData.nColors)
					{
						possible_colors[colorData.nColors] = true;
						colorData.nNodes[colorData.nColors] = 0;
						colorData.nNeighbors[colorData.nColors] = 0;
						colorData.nColors++;
					}
					possible_colors[c] = false;
				}
			}


			uint8_t chosen_color = INVALID_COLOR;
			for (uint8_t j = 0; j < colorData.nColors; ++j)
			{
				if (possible_colors[j])
				{
					chosen_color = j;
					break;
				}
			}
			if (chosen_color == INVALID_COLOR)
			{
				if (colorData.nColors == MAX_COLORS - 1)
					throw std::runtime_error("Ran out of colors during graph coloring! "
						"The input mesh is very likely corrupt.");
				colorData.nNodes[colorData.nColors] = 1;
				colorData.nNeighbors[colorData.nColors] = neighbors[i].size();
				color[i] = colorData.nColors++;
			}
			else
			{
				colorData.nNodes[chosen_color]++;
				colorData.nNeighbors[chosen_color] += neighbors[i].size();
				color[i] = chosen_color;
			}

			mutex[i].unlock();
			for (auto j : neighbors[i])
				if (j < vertexCount)
					mutex[j].unlock();
		}
		return colorData;
	},
		[](ColorData c1, ColorData c2) -> ColorData
	{
		ColorData result;
		result.nColors = std::max(c1.nColors, c2.nColors);
		memset(result.nNodes, 0, sizeof(uint32_t) * result.nColors);
		memset(result.nNeighbors, 0, sizeof(uint32_t) * result.nColors);
		for (uint8_t i = 0; i < c1.nColors; ++i)
		{
			result.nNodes[i] += c1.nNodes[i];
			result.nNeighbors[i] += c1.nNeighbors[i];
		}
		for (uint8_t i = 0; i < c2.nColors; ++i)
		{
			result.nNodes[i] += c2.nNodes[i];
			result.nNeighbors[i] += c2.nNeighbors[i];
		}
		return result;
	}
	);

	std::cout << "done (took " << (int)colorData.nColors << " colors and " << timeString(timer.value()) << ")." << std::endl;

	return colorData;
}