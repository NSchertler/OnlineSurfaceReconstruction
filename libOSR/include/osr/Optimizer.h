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

#pragma once

#include "osr/MeshSettings.h"
#include "osr/HierarchyDecl.h"
#include "osr/Attributes.h"

#ifdef OPENMP
#include <omp.h>
#endif

namespace osr
{
	class Optimizer
	{
	public:
		Optimizer(const MeshSettings& _meshSettings);

		//Perform orientation field smoothing
		template<bool WithConstraint, typename TDataStore, typename TPhases>
		void optimizeOrientations(TDataStore& dataStore, const TPhases& phases) const
		{
			const int levelIterations = 6;

			for (int it = 0; it < levelIterations; ++it)
			{
				for (auto phase : phases)
				{
					phase.processInParallel([&](auto& i)
					{
						const Vector3f n_i = dataStore.template attribute<Normal>(i);
						Float weight_sum = 0.0f;
						Vector3f sum = dataStore.template attribute<DirField>(i);
						Vector3f dirCompat1, dirCompat2;
						dataStore.forEachNeighbor(i, [&](const auto& j)
						{
							const Float weight = 1; // link.weight;
							if (weight == 0)
								return;
							const Vector3f n_j = dataStore.template attribute<Normal>(j);
							Vector3f q_j = dataStore.template attribute<DirField>(j);
							_meshSettings.rosy->findCompatible(sum, n_i, q_j, n_j, dirCompat1, dirCompat2);
							sum = dirCompat1 * weight_sum + dirCompat2 * weight;
							sum -= n_i*n_i.dot(sum);
							weight_sum += weight;

							Float norm = sum.norm();
							if (norm > RCPOVERFLOW)
								sum /= norm;
						});

						if (WithConstraint)
						{
							auto& constraint = dataStore.template attribute<DirFieldConstraint>(i);
							float lengthSq = constraint.squaredNorm();
							Vector3f constraintNorm = constraint.normalized();
							if (lengthSq > 0)
							{
								float weight = sqrt(lengthSq);
								_meshSettings.rosy->findCompatible(sum, n_i, constraintNorm, n_i, dirCompat1, dirCompat2);
								sum = dirCompat1 * (1 - weight) + dirCompat2 * weight;
								sum -= n_i*n_i.dot(sum);

								Float norm = sum.norm();
								if (norm > RCPOVERFLOW)
									sum /= norm;
							}
						}

						if (weight_sum > 0)
							dataStore.template attribute<DirField>(i) = sum;
					});
				}
			}
		}

		//Perform position field smoothing
		template<typename TDataStore, typename TPhases>
		void optimizePositions(TDataStore& dataStore, const TPhases& phases) const
		{
			const int levelIterations = 6;

			const Float inv_scale = 1.0f / _meshSettings.scale();

			for (int it = 0; it < levelIterations; ++it)
			{
				for (auto phase : phases)
					phase.processInParallel([&](auto& i)
				{
					const Vector3f n_i = dataStore.template attribute<Normal>(i), v_i = dataStore.template attribute<Position>(i);
					Vector3f q_i = dataStore.template attribute<DirField>(i);

					Vector3f sum = dataStore.template attribute<PosField>(i);
					Float weight_sum = 0.0f;

					dataStore.forEachNeighbor(i, [&](const auto& j)
					{
						const Float weight = 1; // link.weight;
						if (weight == 0)
							return;

						const Vector3f n_j = dataStore.template attribute<Normal>(j), v_j = dataStore.template attribute<Position>(j);
						Vector3f q_j = dataStore.template attribute<DirField>(j), o_j = dataStore.template attribute<PosField>(j);

						Vector3f posCompat1, posCompat2;
						_meshSettings.posy->findCompatible(
							v_i, n_i, q_i, sum, v_j, n_j, q_j, o_j, _meshSettings.scale(), inv_scale, posCompat1, posCompat2);

						sum = posCompat1 * weight_sum + posCompat2 * weight;
						weight_sum += weight;
						if (weight_sum > RCPOVERFLOW)
							sum /= weight_sum;
						sum -= n_i.dot(sum - v_i)*n_i;
					});

					if (weight_sum > 0)
						dataStore.template attribute<PosField>(i) = _meshSettings.posy->positionRound(sum, q_i, n_i, v_i, _meshSettings.scale(), inv_scale);
				});
			}
		}

		template <typename TDataStore, typename Iterator>
		void calculateEnergy(TDataStore& dataStore, Iterator begin, Iterator end, float& energyOrientation, float& energyPosition)
		{
			float inv_scale = 1.0f / _meshSettings.scale();
			double eDir = 0, ePos = 0;
			for (auto it = begin; it != end; ++it)
			{
				auto& i = *it;
				const Vector3f v_i = dataStore.template attribute<Position>(i);
				const Vector3f n_i = dataStore.template attribute<Normal>(i);
				Vector3f q_i = dataStore.template attribute<DirField>(i);
				Vector3f o_i = dataStore.template attribute<PosField>(i);
				dataStore.forEachNeighbor(i, [&](const auto& j)
				{
					const Float weight = 1; // link.weight;
					if (weight == 0)
						return;
					const Vector3f v_j = dataStore.template attribute<Position>(j);
					const Vector3f n_j = dataStore.template attribute<Normal>(j);
					Vector3f q_j = dataStore.template attribute<DirField>(j);
					Vector3f o_j = dataStore.template attribute<PosField>(j);
					Vector3f compatDir1, compatDir2;
					_meshSettings.rosy->findCompatible(q_i, n_i, q_j, n_j, compatDir1, compatDir2);

					Vector3f compatPos1, compatPos2;
					_meshSettings.posy->findCompatible(
						v_i, n_i, q_i, o_i, v_j, n_j, q_j, o_j, _meshSettings.scale(), inv_scale, compatPos1, compatPos2);

					eDir += (compatDir1 - compatDir2).squaredNorm();
					ePos += (compatPos1 - compatPos2).squaredNorm();
				});
			}
			energyOrientation = eDir;
			energyPosition = ePos;
		}

		//Perform normal filtering
		template<typename TDataStore, typename TPhases>
		void optimizeNormals(TDataStore& dataStore, const TPhases& phases) const
		{
			const int iterations = 30;
			const float sigmaNormal = 0.10f;
			const float sigmaDistance = 0.10f;

			for (int it = 0; it < iterations; ++it)
			{
				for (auto phase : phases)
				{
					int rosy = _meshSettings.rosy->rosy();
					phase.processInParallel([&](auto& i)
					{
						const Vector3f& q = dataStore.template attribute<DirField>(i);
						const Vector3f& n = dataStore.template attribute<Normal>(i);
						const Vector3f p = dataStore.template attribute<Position>(i);

						// -- Bilateral filtering --
						float sumWeight = 1;
						Vector3f mean = n;

						dataStore.forEachNeighbor(i, [&](const auto& j)
						{
							Vector3f& pn = dataStore.template attribute<Position>(j);
							Vector3f& nn = dataStore.template attribute<Normal>(j);

							float distanceSq = (p - pn).squaredNorm() / (_meshSettings.scale() * _meshSettings.scale());
							float weightDistance = exp(-distanceSq / (2 * sigmaDistance * sigmaDistance));

							float normalDeviation = nn.dot(n);
							normalDeviation = acos(std::min<float>(1, normalDeviation)) * 2 / 3.1415926f; //transform to steeper angle-space
							//Skip the bilateral weight if simple Laplacian smoothing is desired.
							float weight = weightDistance;// *exp((-normalDeviation * normalDeviation) / (2 * sigmaNormal * sigmaNormal));

							sumWeight += weight;
							mean += weight / sumWeight * (nn - mean);

						});
						Vector3f smoothedNormal = mean.normalized();

						Vector3f axis = n.cross(smoothedNormal);
						float angle = acos(std::max(-1.0f, std::min(1.0f, n.dot(smoothedNormal))));
						dataStore.template attribute<Normal>(i) = smoothedNormal;
					});
				}
			}
		}

		const MeshSettings& meshSettings() const { return _meshSettings; }

	protected:
		const MeshSettings& _meshSettings;
	};
}