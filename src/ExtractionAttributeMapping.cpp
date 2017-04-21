#include "ExtractionAttributeMapping.h"
#include "ParallelCG.h"
#include <iostream>

#ifdef OPENMP
#include <omp.h>
#endif

// ------------  LaplacianEntry  ------------

void LaplacianEntry::addNeighbor(const ExtractionHelper::Entity * entity, size_t localIndex)
{
	neighbors.push_back({ entity, localIndex });
}

void LaplacianEntry::reset()
{
	neighbors.clear();
}


// ------------  LeastSquaresSystemBuilder  ------------

LeastSquaresSystemBuilder::LeastSquaresSystemBuilder(size_t totalTexels)
	: initialGuess(Eigen::Matrix<float, -1, 4>::Zero(totalTexels, 4))
{ }

void LeastSquaresSystemBuilder::closeGapsInGuess()
{
	//Closes gaps in the initial guess by propagating value

	std::vector<int> zeroGuessRows;
	for (int i = 0; i < initialGuess.rows(); ++i)
		if (initialGuess.row(i).isZero())
			zeroGuessRows.push_back(i);

	int iteration = 0;
	while (!zeroGuessRows.empty() && iteration < 10)
	{
		++iteration;
#pragma omp parallel for
		for (int i = 0; i < zeroGuessRows.size(); ++i)
		{
			auto row = zeroGuessRows[i];
			if (row == -1)
				continue;
			//take the initial guess from the neighbor with the maximum weight in the matrix
			float maxWeight = 0;
			for (Eigen::SparseMatrix<float>::InnerIterator it = rowIterator(row); it; ++it)
			{
				if (it.index() != row && std::abs(it.value()) > maxWeight)
				{
					auto v = initialGuess.row(it.index());
					if (!v.isZero())
					{
						maxWeight = std::abs(it.value());
						initialGuess.row(row) = v;
					}
				}
			}
			if (maxWeight > 0)
				zeroGuessRows[i] = -1;
		}
	}
}

void LeastSquaresSystemBuilder::setGuess(size_t index, const Vector4f& guess)
{
	initialGuess.row(index) = guess;
}

// ------------  HeightFieldLeastSquaresSystemBuilder  ------------

HeightFieldLeastSquaresSystemBuilder::HeightFieldLeastSquaresSystemBuilder(size_t totalTexels)
	: LeastSquaresSystemBuilder(totalTexels), system(totalTexels)
{ 
#ifdef OPENMP
	rowWorkingSet.resize(omp_get_num_procs());
#else
	rowWorkingSet.resize(1);
#endif
}

void HeightFieldLeastSquaresSystemBuilder::addLaplacianEntry(const LaplacianEntry & entry, float weight, uint8_t currentGeneration, const ExtractionHelper::IEntityContainer& container)
{
	auto& row =
#ifdef OPENMP
		rowWorkingSet[omp_get_thread_num()];
#else
		rowWorkingSet[0];
#endif
	row.reset();

	float normalizer = 1.0f / entry.neighbors.size();
	row.addCoefficient(entry.center.entity->indexInTexelVector + entry.center.localIndex, -1.0f);
	for (auto& n : entry.neighbors)
	{
		if (n.entity->generation == currentGeneration)
			row.addCoefficient(n.entity->indexInTexelVector + n.localIndex, normalizer);
		else
			row.addCoefficientWithFixed(normalizer, n.entity->texel(n.localIndex));
	}
	system.addRow(row, weight);
}

void HeightFieldLeastSquaresSystemBuilder::addInterpolationConstraint(const ExtractionHelper::FaceInterpolationInfo * interpolationInfos, size_t count, const Vector4f& value, float weight, uint8_t currentGeneration)
{
	auto& row =
#ifdef OPENMP
		rowWorkingSet[omp_get_thread_num()];
#else
		rowWorkingSet[0];
#endif
	row.reset();

	float maxWeight = -1;
	int maxWeightIdx = -1;

	for (int i = 0; i < count; ++i)
	{
		if (interpolationInfos[i].weight == 0)
			continue;
		if (interpolationInfos[i].entity->generation == currentGeneration)
		{
			auto idx = interpolationInfos[i].entity->indexInTexelVector + interpolationInfos[i].localTexelIndex;
			row.addCoefficient(idx, interpolationInfos[i].weight);
			if (interpolationInfos[i].weight > maxWeight)
			{
				maxWeight = interpolationInfos[i].weight;
				maxWeightIdx = idx;
			}
		}
		else
		{
			row.addCoefficientWithFixed(interpolationInfos[i].weight, interpolationInfos[i].entity->texel(interpolationInfos[i].localTexelIndex));
		}
	}

	row.addToRHS(value);
	system.addRowAtomic(row, weight);

	//Set the initial guess for the texel with the maximum weight to the point attribute
	if (maxWeightIdx >= 0)
		initialGuess.row(maxWeightIdx) = value;
}

void HeightFieldLeastSquaresSystemBuilder::reserve(const Eigen::VectorXi& entriesPerRow)
{
	system.lhs.reserve(entriesPerRow);
}

Eigen::Matrix<float, Eigen::Dynamic, 4> HeightFieldLeastSquaresSystemBuilder::solve()
{
	ParallelCG<LeastSquaresSystem<4>::MatrixType> solver;
	solver.setTolerance(1e-4);

	auto solution = system.solve(solver, initialGuess);
	std::cout << solver.iterations() << " iterations" << std::endl;

	return solution;
}

Eigen::SparseMatrix<float>::InnerIterator HeightFieldLeastSquaresSystemBuilder::rowIterator(size_t row)
{
	return Eigen::SparseMatrix<float>::InnerIterator(system.lhs, row);
}

// ------------  GeometricLeastSquaresSystemBuilder  ------------

GeometricLeastSquaresSystemBuilder::GeometricLeastSquaresSystemBuilder(size_t totalTexels)
	: LeastSquaresSystemBuilder(totalTexels), colorSystem(totalTexels), offsetSystem(totalTexels)
{
#ifdef OPENMP
	colorRowWorkingSet.resize(omp_get_num_procs());
	offsetRowWorkingSet.resize(3 * omp_get_num_procs());
#else
	colorRowWorkingSet.resize(1);
	offsetRowWorkingSet.resize(3);
#endif
}

void GeometricLeastSquaresSystemBuilder::addLaplacianEntry(const LaplacianEntry & entry, float weight, uint8_t currentGeneration, const ExtractionHelper::IEntityContainer& container)
{	
#ifdef OPENMP
	auto& colorRow = colorRowWorkingSet[omp_get_thread_num()];
	auto  offsetRows = &offsetRowWorkingSet[3 * omp_get_thread_num()];
#else
	auto& colorRow = rowWorkingSet[0];
	auto  offsetRows = &offsetRowWorkingSet[3];
#endif
	colorRow.reset();
	for (int i = 0; i < 3; ++i)
		offsetRows[i].reset();

	float normalizer = 1.0f / entry.neighbors.size();
	Vector3f centerP, centerN;
	entry.center.entity->getInterpolatedPositionNormal(container, entry.center.localIndex, centerP, centerN);
	for (int i = 0; i < 3; ++i)
	{
		offsetRows[i].addCoefficient(entry.center.entity->indexInTexelVector + entry.center.localIndex, -centerN(i));
		offsetRows[i].addToRHS(centerP.block<1, 1>(i, 0));
		//offsetRows[i].addCoefficient(entry.center.entity->indexInTexelVector + entry.center.localIndex, 1);
	}
	colorRow.addCoefficient(entry.center.entity->indexInTexelVector + entry.center.localIndex, -1.0f);
	for (auto& n : entry.neighbors)
	{
		Vector3f p, no;
		n.entity->getInterpolatedPositionNormal(container, n.localIndex, p, no);
		for (int i = 0; i < 3; ++i)
			offsetRows[i].addToRHS(-normalizer * p.block<1, 1>(i, 0));
		if (n.entity->generation == currentGeneration)
		{			
			for (int i = 0; i < 3; ++i)
				offsetRows[i].addCoefficient(n.entity->indexInTexelVector + n.localIndex, normalizer * no(i));
				//offsetRows[i].addCoefficient(n.entity->indexInTexelVector + n.localIndex, normalizer);
			colorRow.addCoefficient(n.entity->indexInTexelVector + n.localIndex, normalizer);
		}
		else
		{
			colorRow.addCoefficientWithFixed(normalizer, n.entity->texel(n.localIndex).head<3>());
			for (int i = 0; i < 3; ++i)
				offsetRows[i].addCoefficientWithFixed(normalizer * no(i), n.entity->texel(n.localIndex).tail<1>());
		}
	}
	colorSystem.addRow(colorRow, weight);
	for(int i = 0; i < 3; ++i)
		offsetSystem.addRow(offsetRows[i], weight);
}

void GeometricLeastSquaresSystemBuilder::addInterpolationConstraint(const ExtractionHelper::FaceInterpolationInfo * interpolationInfos, size_t count, const Vector4f & value, float weight, uint8_t currentGeneration)
{
#ifdef OPENMP
	auto& colorRow = colorRowWorkingSet[omp_get_thread_num()];
	auto& offsetRow = offsetRowWorkingSet[3 * omp_get_thread_num()];
#else
	auto& colorRow = rowWorkingSet[0];
	auto  offsetRows = &offsetRowWorkingSet[3];
#endif
	colorRow.reset();
	offsetRow.reset();

	float maxWeight = -1;
	int maxWeightIdx = -1;

	for (int i = 0; i < count; ++i)
	{
		if (interpolationInfos[i].weight == 0)
			continue;
		if (interpolationInfos[i].entity->generation == currentGeneration)
		{
			auto idx = interpolationInfos[i].entity->indexInTexelVector + interpolationInfos[i].localTexelIndex;
			colorRow.addCoefficient(idx, interpolationInfos[i].weight);
			offsetRow.addCoefficient(idx, interpolationInfos[i].weight);			
			if (interpolationInfos[i].weight > maxWeight)
			{
				maxWeight = interpolationInfos[i].weight;
				maxWeightIdx = idx;
			}
		}
		else
		{
			colorRow.addCoefficientWithFixed(interpolationInfos[i].weight, interpolationInfos[i].entity->texel(interpolationInfos[i].localTexelIndex).head<3>());
			offsetRow.addCoefficientWithFixed(interpolationInfos[i].weight, interpolationInfos[i].entity->texel(interpolationInfos[i].localTexelIndex).tail<1>());
		}
	}

	colorRow.addToRHS(value.head<3>());
	offsetRow.addToRHS(value.tail<1>());

	colorSystem.addRow(colorRow, weight);
	offsetSystem.addRow(offsetRow, weight);

	//Set the initial guess for the texel with the maximum weight to the point attribute
	if (maxWeightIdx >= 0)
		initialGuess.row(maxWeightIdx) = value;
}

void GeometricLeastSquaresSystemBuilder::reserve(const Eigen::VectorXi & entriesPerRow)
{
	colorSystem.lhs.reserve(entriesPerRow);
	offsetSystem.lhs.reserve(entriesPerRow);
}

Eigen::Matrix<float, Eigen::Dynamic, 4> GeometricLeastSquaresSystemBuilder::solve()
{
	ParallelCG<LeastSquaresSystem<4>::MatrixType> solver;
	solver.setTolerance(1e-4);	

	Eigen::Matrix<float, Eigen::Dynamic, 4> solution(initialGuess.rows(), 4);

	solver.solveColUpperExclusive = 3;
	colorSystem.solve(solver, initialGuess, solution);
	
	solver.solveColLowerInclusive = 3;
	solver.solveColUpperExclusive = 4;
	offsetSystem.solve(solver, initialGuess, solution);

	return solution;
}

Eigen::SparseMatrix<float>::InnerIterator GeometricLeastSquaresSystemBuilder::rowIterator(size_t row)
{
	return Eigen::SparseMatrix<float>::InnerIterator(colorSystem.lhs, row);
}
