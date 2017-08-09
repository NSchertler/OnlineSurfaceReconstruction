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

#include <Eigen/Sparse>
#include "osr/Parallelization.h"

namespace osr
{
	template <int SolutionColumns>
	class LeastSquaresSystem;

	// Represents a row in the linear system A x = b
	template <int SolutionColumns>
	class LinearSystemRow
	{
	public:
		//Resets the row to zero
		void reset();

		//Adds the summand coefficient * x_index to the left-hand side of the equation.
		void addCoefficient(int index, float coefficient);

		//Adds the summand coefficient * fixed to the left-hand side of the equation.
		void addCoefficientWithFixed(float coefficient, const Eigen::Matrix<float, SolutionColumns, 1>& fixed);

		//Adds value to the right-hand side of the equation.
		void addToRHS(const Eigen::Matrix<float, SolutionColumns, 1>& value);

	private:
		std::vector<std::pair<int, float>> lhsCoefficients;
		Eigen::Matrix<float, SolutionColumns, 1> rhs;

		template <int N>
		friend class LeastSquaresSystem;
	};

	// Represents a sparse linear least squares system as normal equations A^T A x = A^T b.
	// SolutionColumns is the number of columns of x and b.
	template <int SolutionColumns>
	class LeastSquaresSystem
	{
	public:
		typedef Eigen::SparseMatrix<float> MatrixType;

		LeastSquaresSystem();
		LeastSquaresSystem(int numberOfUnknowns);

		// Updates the system in a way that is equivalent to adding another row in A and b in A x = b.
		// The first SkipColumns columns of the row's right-hand side are ignored.
		template <int SkipColumns, int Columns>
		void addRow(const LinearSystemRow<Columns>& row, float weight = 1)
		{
			for (auto& entry1 : row.lhsCoefficients)
			{
				for (auto& entry2 : row.lhsCoefficients)
				{
					lhs.coeffRef(entry1.first, entry2.first) += weight * entry1.second * entry2.second;
				}
				rhs.row(entry1.first) += weight * entry1.second * row.rhs.template block<SolutionColumns, 1>(SkipColumns, 0).transpose();
			}
		}

		// Updates the system in a way that is equivalent to adding another row in A and b in A x = b.
		// row is a list of index/value pairs. solution is the right-hand side of the equation.
		void addRow(const LinearSystemRow<SolutionColumns>& row, float weight = 1);

		// Has the same semantics as addRow but performs all operations atomically. This requires
		// that all entries in the matrix are already present.
		template <int SkipColumns, int Columns>
		void addRowAtomic(const LinearSystemRow<Columns>& row, float weight = 1)
		{
			for (auto& entry1 : row.lhsCoefficients)
			{
				if (entry1.second == 0)
					continue;
				for (auto& entry2 : row.lhsCoefficients)
				{
					if (entry2.second == 0)
						continue;
					float* coeff = &lhs.coeffRef(entry1.first, entry2.first);
					atomicAdd(coeff, weight * entry1.second * entry2.second);
				}
				rhs.row(entry1.first) += weight * entry1.second * row.rhs.template block<SolutionColumns, 1>(SkipColumns, 0).transpose();
			}
		}

		// Has the same semantics as addRow but performs all operations atomically. This requires
		// that all entries in the matrix are already present.
		void addRowAtomic(const LinearSystemRow<SolutionColumns>& row, float weight = 1);

		template <typename EigenSolver>
		Eigen::Matrix<float, Eigen::Dynamic, SolutionColumns> solve(EigenSolver& solver, const Eigen::Matrix<float, Eigen::Dynamic, SolutionColumns>& initialGuess)
		{
			solver.compute(lhs);

			Eigen::Matrix<float, Eigen::Dynamic, SolutionColumns> solution(initialGuess.rows(), SolutionColumns);
			solver.solveWithGuess(rhs, initialGuess, solution);
			return solution;
		}

		// This overload can be used when you want to solve only a subset of available columns of the solution.
		template <int TotalColumnsOfGuessAndSolution, typename EigenSolver>
		void solve(EigenSolver& solver, const Eigen::Matrix<float, Eigen::Dynamic, TotalColumnsOfGuessAndSolution>& initialGuess, Eigen::Matrix<float, Eigen::Dynamic, TotalColumnsOfGuessAndSolution>& solution)
		{
			solver.compute(lhs);

			solver.solveWithGuess(rhs, initialGuess, solution);
		}

		size_t numberOfUnknowns() const { return lhs.rows(); }

	public:
		MatrixType lhs;
		Eigen::Matrix<float, -1, SolutionColumns> rhs;
	};
}