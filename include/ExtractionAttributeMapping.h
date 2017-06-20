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

#include "ExtractionUtils.h"
#include "LeastSquaresSystem.h"

namespace osr
{
	struct LaplacianEntry
	{
		struct Data
		{
			const ExtractionHelper::Entity* entity;
			size_t localIndex;
		};
		Data center;
		std::vector<Data> neighbors;

		void addNeighbor(const ExtractionHelper::Entity* entity, size_t localIndex);

		void reset();
	};

	class LeastSquaresSystemBuilder
	{
	public:
		LeastSquaresSystemBuilder(size_t totalTexels);
		void closeGapsInGuess();
		void setGuess(size_t index, const Vector4f& guess);

		//This method is not thread-safe. Make sure to call this method only once at a time.
		virtual void addLaplacianEntry(const LaplacianEntry& entry, float weight, uint8_t currentGeneration, const ExtractionHelper::IEntityContainer& container) = 0;

		//Performs an atomic add into the underlying system. Assumes that all relevant entries in the sparse matrix already exist.
		virtual void addInterpolationConstraint(const ExtractionHelper::FaceInterpolationInfo* interpolationInfos, size_t count, const Vector4f& value, float weight, uint8_t currentGeneration) = 0;

		virtual void reserve(const Eigen::VectorXi& entriesPerRow) = 0;

		virtual Eigen::Matrix<float, Eigen::Dynamic, 4> solve() = 0;

	protected:
		Eigen::Matrix<float, Eigen::Dynamic, 4> initialGuess;

		virtual Eigen::SparseMatrix<float>::InnerIterator rowIterator(size_t row) = 0;
	};

	class HeightFieldLeastSquaresSystemBuilder : public LeastSquaresSystemBuilder
	{
	public:
		HeightFieldLeastSquaresSystemBuilder(size_t totalTexels);

		//This method is not thread-safe. Make sure to call this method only once at a time.
		void addLaplacianEntry(const LaplacianEntry& entry, float weight, uint8_t currentGeneration, const ExtractionHelper::IEntityContainer& container);

		//Performs an atomic add into the underlying system. Assumes that all relevant entries in the sparse matrix already exist.
		void addInterpolationConstraint(const ExtractionHelper::FaceInterpolationInfo* interpolationInfos, size_t count, const Vector4f& value, float weight, uint8_t currentGeneration);

		void reserve(const Eigen::VectorXi& entriesPerRow);

		Eigen::Matrix<float, Eigen::Dynamic, 4> solve();

	protected:
		Eigen::SparseMatrix<float>::InnerIterator rowIterator(size_t row);

	private:
		LeastSquaresSystem<4> system;

		std::vector<LinearSystemRow<4>> rowWorkingSet; //these rows are used as temporary data stores (one for each OMP thread)
	};

	class GeometricLeastSquaresSystemBuilder : public LeastSquaresSystemBuilder
	{
	public:
		GeometricLeastSquaresSystemBuilder(size_t totalTexels);

		//This method is not thread-safe. Make sure to call this method only once at a time.
		void addLaplacianEntry(const LaplacianEntry& entry, float weight, uint8_t currentGeneration, const ExtractionHelper::IEntityContainer& container);

		//Performs an atomic add into the underlying system. Assumes that all relevant entries in the sparse matrix already exist.
		void addInterpolationConstraint(const ExtractionHelper::FaceInterpolationInfo* interpolationInfos, size_t count, const Vector4f& value, float weight, uint8_t currentGeneration);

		void reserve(const Eigen::VectorXi& entriesPerRow);

		Eigen::Matrix<float, Eigen::Dynamic, 4> solve();

	protected:
		Eigen::SparseMatrix<float>::InnerIterator rowIterator(size_t row);

	private:
		LeastSquaresSystem<3> colorSystem;
		LeastSquaresSystem<1> offsetSystem;

		std::vector<LinearSystemRow<1>> offsetRowWorkingSet; //these rows are used as temporary data stores (one for each OMP thread)
		std::vector<LinearSystemRow<3>> colorRowWorkingSet; //these rows are used as temporary data stores (one for each OMP thread)
	};
}