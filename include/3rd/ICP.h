///////////////////////////////////////////////////////////////////////////////
///   "Sparse Iterative Closest Point"
///   by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///   Copyright (C) 2013  LGG, EPFL
///   -- modified version --
///////////////////////////////////////////////////////////////////////////////
///   1) This file contains different implementations of the ICP algorithm.
///   2) This code requires EIGEN and NANOFLANN.
///   3) If OPENMP is activated some part of the code will be parallelized.
///   4) This code is for now designed for 3D registration
///   5) Two main input types are Eigen::Matrix3Xd or Eigen::Map<Eigen::Matrix3Xd>
///////////////////////////////////////////////////////////////////////////////
///   namespace nanoflann: NANOFLANN KD-tree adaptor for EIGEN
///   namespace RigidMotionEstimator: functions to compute the rigid motion
///   namespace SICP: sparse ICP implementation
///   namespace ICP: reweighted ICP implementation
///////////////////////////////////////////////////////////////////////////////

//Note for Online Surface Reconstruction:
//The original implementation has been adapted slightly to conform to OSR data structures

#ifndef ICP_H
#define ICP_H
#include <Eigen/Dense>
#include <vector>

#include "osr/INeighborQueryable.h"
#include <nsessentials/util/IndentationLog.h>
#include "osr/common.h"

/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace RigidMotionEstimator {

    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3f point_to_point(const Eigen::MatrixBase<Derived1>& X,
                                   const Eigen::MatrixBase<Derived2>& Y,
                                   const Eigen::MatrixBase<Derived3>& w,
								   const Eigen::Affine3f& initialTransform)
	{
		/// Normalize weight vector
        Eigen::VectorXd w_normalized = w.template cast<double>()/(double)w.sum();
        /// De-mean
        Eigen::Vector3d X_mean, Y_mean;
        for(int i=0; i<3; ++i)
		{
            X_mean(i) = (X.row(i).template cast<double>().array()*w_normalized.transpose().array()).sum();
            Y_mean(i) = (Y.row(i).template cast<double>().array()*w_normalized.transpose().array()).sum();
        }
		X_mean = initialTransform.cast<double>() * X_mean;

        /// Compute transformation
        Eigen::Affine3d transformation = initialTransform.cast<double>();
		transformation.pretranslate(-X_mean.cast<double>());
		Eigen::Matrix3d sigma; sigma.setConstant(0.0);
		double error = 0;
		for (int i = 0; i < X.cols(); ++i)
		{
			sigma += (initialTransform.template cast<double>() * X.col(i).template cast<double>() - X_mean) * (double)w_normalized.coeff(i) * (Y.col(i).template cast<double>() - Y_mean).transpose();
			error += (initialTransform * X.col(i) - Y.col(i)).squaredNorm() * w_normalized.coeff(i);
		}
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Eigen::Affine3d rotation;
		rotation.setIdentity();
        if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
            Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
            rotation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
        } else {
            rotation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
        }

		transformation = rotation * transformation;
		transformation.pretranslate(Y_mean);

		error = 0;
		for (int i = 0; i < X.cols(); ++i)
		{
			error += (transformation.cast<float>() * X.col(i) - Y.col(i)).squaredNorm() * w_normalized.coeff(i);
		}

        /// Return transform
        return transformation.cast<float>();
    }

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    template <typename Derived1, typename Derived2>
    inline Eigen::Affine3f point_to_point(const Eigen::MatrixBase<Derived1>& X,
                                          const Eigen::MatrixBase<Derived2>& Y,
		const Eigen::Affine3f& initialTransform)
	{
        return point_to_point(X, Y, Eigen::VectorXf::Ones(X.cols()), initialTransform);
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
    Eigen::Affine3f point_to_plane(const Eigen::MatrixBase<Derived1>& X,
                                   const Eigen::MatrixBase<Derived2>& Y,
                                   const Eigen::MatrixBase<Derived3>& N,
                                   const Eigen::MatrixBase<Derived4>& w,
                                   const Eigen::MatrixBase<Derived5>& u,
								   const Eigen::Affine3f& initialTransform) {
        typedef Eigen::Matrix<double, 6, 6> Matrix66;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Block<Matrix66, 3, 3> Block33;
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w.template cast<double>()/(double)w.sum();
        /// De-mean
        Eigen::Vector3d X_mean;
        for(int i=0; i<3; ++i)
            X_mean(i) = (X.row(i).template cast<double>().array()*w_normalized.transpose().array()).sum();
		X_mean = initialTransform.cast<double>() * X_mean;

		Eigen::Affine3d transformation = initialTransform.cast<double>();
		transformation.pretranslate(-X_mean);

        /// Prepare LHS and RHS
        Matrix66 LHS = Matrix66::Zero();
        Vector6 RHS = Vector6::Zero();
        Block33 TL = LHS.topLeftCorner<3,3>();
        Block33 TR = LHS.topRightCorner<3,3>();
        Block33 BR = LHS.bottomRightCorner<3,3>();
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
        #pragma omp parallel
        {
            #pragma omp for
            for(int i=0; i<X.cols(); i++) {
                C.col(i) = ((initialTransform * X.col(i)).template cast<double>() - X_mean).cross(N.col(i).template cast<double>());
            }
            #pragma omp sections nowait
            {
                #pragma omp section
                for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
                #pragma omp section
                for(int i=0; i<X.cols(); i++) TR += (C.col(i) * N.col(i).template cast<double>().transpose())*w(i);
                #pragma omp section
                for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i).template cast<double>(), w(i));
                #pragma omp section
                for(int i=0; i<C.cols(); i++) {
                    double dist_to_plane = -(((initialTransform * X.col(i)).template cast<double>() - Y.col(i).template cast<double>()).dot(N.col(i).template cast<double>()) - u(i)) * w(i);
                    RHS.head<3>() += C.col(i)*dist_to_plane;
                    RHS.tail<3>() += N.col(i).template cast<double>() * dist_to_plane;
                }
            }
        }
        LHS = LHS.selfadjointView<Eigen::Upper>();
        /// Compute transformation
        Eigen::LDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ()) * transformation;
        transformation.pretranslate(RHS.tail<3>());
		transformation.pretranslate(X_mean);

		return transformation.cast<float>();
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    inline Eigen::Affine3f point_to_plane(const Eigen::MatrixBase<Derived1>& X,
                                          const Eigen::MatrixBase<Derived2>& Yp,
                                          const Eigen::MatrixBase<Derived3>& Yn,
                                          const Eigen::MatrixBase<Derived4>& w,
										  const Eigen::Affine3f& initialTransform) {
        return point_to_plane(X, Yp, Yn, w, Eigen::VectorXf::Zero(X.cols()), initialTransform);
    }
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP {
    struct Parameters {
        bool use_penalty = false; /// if use_penalty then penalty method else ADMM or ALM (see max_inner)
        float p = 1.0f;           /// p norm
        float mu = 10.0f;         /// penalty weight
        float alpha = 1.2f;       /// penalty increase factor
        float max_mu = 1e5f;      /// max penalty
        int max_icp = 100;        /// max ICP iteration
        int max_outer = 100;      /// max outer iteration
        int max_inner = 1;        /// max inner iteration. If max_inner=1 then ADMM else ALM
        float stop = 1e-5f;       /// stopping criteria
		const Eigen::Affine3f* initialTransform = nullptr;
        bool print_icpn = false;  /// (debug) print ICP iteration
    };
    /// Shrinkage operator (Automatic loop unrolling using template)
    template<unsigned int I>
    inline float shrinkage(float mu, float n, float p, float s) {
        return shrinkage<I-1>(mu, n, p, 1.0f - (p/mu)*std::pow(n, p-2.0f)*std::pow(s, p-1.0f));
    }
    template<>
    inline float shrinkage<0>(float, float, float, float s) {return s;}
    /// 3D Shrinkage for point-to-point
    template<unsigned int I>
    inline void shrink(Eigen::Matrix3Xf& dirField, float mu, float p) {
        float Ba = std::pow((2.0f/mu)*(1.0f-p), 1.0f/(2.0f-p));
        float ha = Ba + (p/mu)*std::pow(Ba, p-1.0f);
        #pragma omp parallel for
        for(int i=0; i<dirField.cols(); ++i) {
            float n = dirField.col(i).norm();
            float w = 0.0;
            if(n > ha) w = shrinkage<I>(mu, n, p, (Ba/n + 1.0f)/2.0f);
            dirField.col(i) *= w;
        }
    }
    /// 1D Shrinkage for point-to-plane
    template<unsigned int I>
    inline void shrink(Eigen::VectorXf& y, float mu, float p) {
        float Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
        float ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
        #pragma omp parallel for
        for(int i=0; i<y.rows(); ++i) {
            float n = std::abs(y(i));
            float s = 0.0;
            if(n > ha) s = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
            y(i) *= s;
        }
    }
    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters

	template <typename Index>
	inline Eigen::Affine3f point_to_point(const osr::Matrix3Xf& X, const osr::Matrix3Xf& N,
		const osr::IPointQueryable<Index>& Y,
		Parameters par = Parameters())
	{
		/// Buffers
		Eigen::Matrix3Xf dirField = Eigen::Matrix3Xf::Zero(3, X.cols());
		Eigen::Matrix3Xf Z = Eigen::Matrix3Xf::Zero(3, X.cols());
		Eigen::Matrix3Xf C = Eigen::Matrix3Xf::Zero(3, X.cols());
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		Eigen::Matrix3Xf Xo1 = X;
		Eigen::Matrix3Xf Xo2 = X;
		Eigen::VectorXf w = Eigen::VectorXf::Ones(X.cols());
		/// ICP
		for (int icp = 0; icp<par.max_icp; ++icp) {
			if (par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;
			/// Find closest point
			{
				nse::util::TimedBlock b("Finding correspondences ..");
#pragma omp parallel for
				for (int i = 0; i < X.cols(); ++i)
				{
					Eigen::Vector3f px = transform * X.col(i);
					auto idx = Y.findClosestCompatiblePoint(px, transform.linear() * N.col(i));
					if (Y.isIndexValid(idx))
					{
						dirField.col(i) = Y.neighborP(idx);
						auto d = dirField.col(i) - px;
						auto ld = d.squaredNorm();
						if (ld == 0)
							w(i) = 1;
						else
							w(i) = 1; //std::abs((d / sqrt(ld)).dot(Y.n(idx)));// exp(-(dirField.col(i) - px).squaredNorm() / 10.0f);//1.0f / ((dirField.col(i) - X.V().col(i)).squaredNorm() + 0.001f);
					}
					else
					{
						dirField.col(i).setZero();
						w(i) = 0;
					}
				}
			}
			//Compute rotation and translation
			double mu = par.mu;
			for (int outer = 0; outer<par.max_outer; ++outer) {
				double dual = 0.0;
				for (int inner = 0; inner<par.max_inner; ++inner) {
					//Z update (shrinkage)
					Z = transform * X - dirField + C / mu;
					shrink<3>(Z, mu, par.p);
					// Rotation and translation update
					Eigen::Matrix3Xf U = dirField + Z - C / mu;
					transform = RigidMotionEstimator::point_to_point(X, U, w, transform);
					// Stopping criteria
					dual = (transform * X - Xo1).colwise().norm().maxCoeff();
					Xo1 = transform * X;
					if (dual < par.stop) break;
				}
				// C update (lagrange multipliers)
				Eigen::Matrix3Xf P = transform * X - dirField - Z;
				if (!par.use_penalty) C.noalias() += mu*P;
				// mu update (penalty)
				if (mu < par.max_mu) mu *= par.alpha;
				// Stopping criteria
				double primal = P.colwise().norm().maxCoeff();
				if (primal < par.stop && dual < par.stop) break;
			}

			//transform = RigidMotionEstimator::point_to_point(X.V(), dirField, w, transform);

			/// Stopping criteria
			double stop = (transform * X - Xo2).colwise().norm().maxCoeff();
			Xo2 = transform * X;
			if (stop < par.stop) break;
		}
		return transform;
	}

    /// Sparse ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
	template <typename Index>
	Eigen::Affine3f point_to_plane(const osr::Matrix3Xf& X, const osr::Matrix3Xf& N,
		const osr::IPointQueryable<Index>& Y,
		Parameters par = Parameters())
	{
        /// Buffers
        Eigen::Matrix3Xf Qp = Eigen::Matrix3Xf::Zero(3, X.cols());
        Eigen::Matrix3Xf Qn = Eigen::Matrix3Xf::Zero(3, X.cols());
        Eigen::VectorXf Z = Eigen::VectorXf::Zero(X.cols());
        Eigen::VectorXf C = Eigen::VectorXf::Zero(X.cols());
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        Eigen::Matrix3Xf Xo1 = X;
        Eigen::Matrix3Xf Xo2 = X;
		Eigen::VectorXf w = Eigen::VectorXf::Ones(X.cols());

        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;

            /// Find closest point
			{
				nse::util::TimedBlock b("Finding correspondences ..");
				float sumWeight = 0;
#pragma omp parallel for reduction(+ : sumWeight)
				for (int i = 0; i < X.cols(); ++i)
				{
					Eigen::Vector3f px = transform * X.col(i);
					auto idx = Y.findClosestCompatiblePoint(px, transform.linear() * N.col(i));
					if (Y.isIndexValid(idx))
					{
						Qp.col(i) = Y.neighborP(idx);
						Qn.col(i) = Y.neighborN(idx);
						w(i) = 1;// exp(-(dirField.col(i) - px).squaredNorm() / 10.0f);//1.0f / ((dirField.col(i) - X.V().col(i)).squaredNorm() + 0.001f);
																				  //std::cout << w(i) << std::endl;
						sumWeight += w(i);
					}
					else
					{
						Qp.col(i).setZero();
						Qn.col(i) = Eigen::Vector3f::UnitX();
						w(i) = 0;
					}
				}
				if (sumWeight < 3)
					return transform;
			}
            /// Compute rotation and translation
            float mu = par.mu;
            for(int outer=0; outer<par.max_outer; ++outer) {
                float dual = 0.0;
                for(int inner=0; inner<par.max_inner; ++inner) {
                    /// Z update (shrinkage)
					Eigen::Matrix3Xf transformedQn = transform.linear() * Qn;
					Eigen::Matrix3Xf transformedX = transform * X;
                    Z = (transformedQn.array()*(transformedX - Qp).array()).colwise().sum().transpose()+C.array()/mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::VectorXf U = Z-C/mu;
                    transform = RigidMotionEstimator::point_to_plane(X, Qp, Qn, w, U, transform);
                    /// Stopping criteria
                    dual = (transform * X - Xo1).colwise().norm().maxCoeff();
                    Xo1 = transform * X;
                    if(dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::VectorXf P = ((transform.linear() * Qn).array()*(transform * X - Qp).array()).colwise().sum().transpose()-Z.array();
                if(!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if(mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                float primal = P.array().abs().maxCoeff();
                if(primal < par.stop && dual < par.stop) break;
            }
            /// Stopping criteria
            float stop = (transform * X - Xo2).colwise().norm().maxCoeff();
            Xo2 = transform * X;
            if(stop < par.stop) break;
        }
		return transform;
    }
}

#endif
