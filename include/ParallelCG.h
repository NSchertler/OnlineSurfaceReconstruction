#pragma once

#include <Eigen/Dense>

// A parallel conjugate gradient solver complying to the Eigen
// Sparse Solver concept.

template <typename Matrix>
class ParallelCG
{
public:
	ParallelCG()
		: maxIterations(-1), m(nullptr), toleranceSq(-1)
	{ }

	//Specifies the column range of the initial guess and the solution that you want to solve
	int solveColLowerInclusive = 0;
	int solveColUpperExclusive = -1;

	void setMaxIterations(int i) { maxIterations = i; }
	void setTolerance(double t) { toleranceSq = t * t; }
	int iterations() const { return _iterations; }

	void compute(const Matrix& m)
	{
		if (maxIterations == -1)
			maxIterations = m.rows();
		if (toleranceSq == -1)
			toleranceSq = 1e-16;
		this->m = &m;

		//Calculate preconditioner
		invDiag.resize(m.rows());
		for (int j = 0; j < m.outerSize(); ++j)
		{
			typename Matrix::InnerIterator it(m, j);
			while (it && it.index() != j)
				++it;
			if (it && it.index() == j && it.value() != 0)
				invDiag(j) = 1.0f / it.value();
			else
				invDiag(j) = 1;
		}
	}

	template <typename RHSType, typename SolutionType>
	void solveWithGuess(const RHSType& rhs, const SolutionType& guess, SolutionType& solution)
	{
		Eigen::Matrix<typename RHSType::Scalar, -1, 1> 
			r(rhs.rows()), 
			d(rhs.rows()), 
			q(rhs.rows()), 
			s(rhs.rows());

		int upperCol = solveColUpperExclusive;
		if (upperCol < 0)
			upperCol = guess.cols();

		assert(upperCol - solveColLowerInclusive == rhs.cols());
		
		_iterations = 0;
		for (int col = solveColLowerInclusive; col < upperCol; ++col)
		{	
#pragma omp parallel for
			for(int i = 0; i < solution.rows(); ++i)
				solution.coeffRef(i, col) = guess.coeff(i, col);
			
			parallelMatrixMultiplyVector(*m, solution, col, r);

			double rhsNormSq = 0;
#pragma omp parallel for reduction( + : rhsNormSq)
			for (int i = 0; i < rhs.rows(); i++)
			{
				r(i) = rhs.coeff(i, col - solveColLowerInclusive) - r(i);
				d(i) = invDiag(i) * r(i);
				rhsNormSq += rhs.coeff(i, col - solveColLowerInclusive) * rhs.coeff(i, col - solveColLowerInclusive);
			}
			double threshold = toleranceSq * rhsNormSq;

			double delta_new = 0;
#pragma omp parallel for reduction( + : delta_new )
			for (int i = 0; i < rhs.rows(); i++)
				delta_new += r(i) * d(i);
			
			if (delta_new < threshold)
			{
				continue;
			}

			int it;
			for (it = 0; it < maxIterations && delta_new > threshold; it++)
			{
				parallelMatrixMultiplyVector(*m, d, 0, q);

				double dDotQ = 0;
#pragma omp parallel for reduction( + : dDotQ )
				for (int i = 0; i < rhs.rows(); i++)
					dDotQ += d(i) * q(i);

				float alpha = (float)(delta_new / dDotQ);

#pragma omp parallel for
				for (int i = 0; i < rhs.rows(); i++)
					solution.coeffRef(i, col) += d(i) * alpha;

				const int RESET_COUNT = 50;
				if ((it % RESET_COUNT) == (RESET_COUNT - 1))
				{
					parallelMatrixMultiplyVector(*m, solution, col, r);

#pragma omp parallel for
					for (int i = 0; i < rhs.rows(); i++)
					{
						r(i) = rhs.coeff(i, col - solveColLowerInclusive) - r(i);
						s(i) = invDiag(i) * r(i);
					}
				}
				else
				{
#pragma omp parallel for
					for (int i = 0; i < rhs.rows(); i++)
					{
						r(i) -= q(i) * alpha;
						s(i) = invDiag(i) * r(i);
					}
				}

				double delta_old = delta_new;
				delta_new = 0;
#pragma omp parallel for reduction( + : delta_new )
				for (int i = 0; i < rhs.rows(); i++)
					delta_new += r(i) * s(i);					

				float beta = (float)(delta_new / delta_old);
#pragma omp parallel for
				for (int i = 0; i < rhs.rows(); i++)
					d(i) = s(i) + d(i) * beta;
			}
			_iterations += it;
		} //for every column
		_iterations /= upperCol - solveColLowerInclusive;
	}

private:

	template <typename RHSType>
	void parallelMatrixMultiplyVector(const Matrix& m, const RHSType& x, int col, Eigen::Matrix<typename RHSType::Scalar, -1, 1>& out) const
	{
#pragma omp parallel for
		for (int row = 0; row < m.rows(); row++)
		{
			double accum = 0;
			for (typename Matrix::InnerIterator it(m, row); it; ++it)
				accum += it.value() * x.coeff(it.index(), col);
			out(row) = (typename RHSType::Scalar)accum;
		}
	}

	Eigen::VectorXf invDiag;

	int maxIterations;
	double toleranceSq;
	const Matrix* m;

	int _iterations;
};
