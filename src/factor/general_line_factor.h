//
// Created by xubo on 21-2-16.
//

#ifndef SRC_GENERAL_LINE_FACTOR_H
#define SRC_GENERAL_LINE_FACTOR_H

#include "utils/Twist.h"
#include "utils/line_geometry.h"

#include <ceres/autodiff_cost_function.h>

namespace general_line {
    class GeneralLineErrorOneCameraTerm {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GeneralLineErrorOneCameraTerm(const Eigen::Vector4d &obs_t_,
                                      const Eigen::Vector3d &x_dir,
                                      const Eigen::Matrix3d &R_c_l_)
                : obs_t(obs_t_), x_dir_(x_dir), R_c_l(R_c_l_) {}

        template<typename T>
        // theta, rho: line parameter
        // phi: mahata angle
        bool operator()(const T *const line_ptr, const T *const T_w_i_h_ptr,
                        const T *const T_i_c_ptr, T *residual_ptr) const {

            using Vector3T = Eigen::Matrix<T, 3, 1>;
            using Vector2T = Eigen::Matrix<T, 2, 1>;
            using Matrix33T = Eigen::Matrix<T, 3, 3>;
            using Vector6T = Eigen::Matrix<T, 6, 1>;
            using QuaternionT = Eigen::Quaternion<T>;
            using TransformT = Twist<T>;

            // line parameter
            T rho = line_ptr[0];
            T theta = line_ptr[1];

            Eigen::Map<const QuaternionT> q_w_i_h(T_w_i_h_ptr + 3);
            Eigen::Map<const Vector3T> t_w_i_h(T_w_i_h_ptr);

            Eigen::Map<const QuaternionT> q_w_i_t(T_w_i_h_ptr + 3);
            Eigen::Map<const Vector3T> t_w_i_t(T_w_i_h_ptr);

            Eigen::Map<const QuaternionT> q_i_c(T_i_c_ptr + 3);
            Eigen::Map<const Vector3T> t_i_c(T_i_c_ptr);

            TransformT T_i_c(q_i_c, t_i_c);

            TransformT T_w_i_h(q_w_i_h, t_w_i_h);
            TransformT T_w_i_t(q_w_i_t, t_w_i_t);

            TransformT T_w_c_h = T_w_i_h * T_i_c;
            TransformT T_w_c_t = T_w_i_t * T_i_c;

            TransformT T_c_w_h = T_w_c_h.inverse();
            TransformT T_c_w_t = T_w_c_t.inverse();

            TransformT T_t_c_h_c = T_w_c_t.inverse() * T_w_c_h;
            Matrix33T R_t_c_h_c = T_t_c_h_c.rotationMatrix();


            Vector3T line_c;
            line_c << ceres::cos(theta), ceres::sin(theta), T(0);
            line_c = R_c_l.cast<T>() * line_c;


            Vector3T origin_t =  R_t_c_h_c * x_dir_.cast<T>() + rho*T_t_c_h_c.pos;
            Vector3T line_c_t = R_t_c_h_c * line_c;
            Vector3T n_c_t = line_c_t.cross(origin_t);

            T l_norm = n_c_t(0) * n_c_t(0) + n_c_t(1) * n_c_t(1);

            T l_sqrtnorm = ceres::sqrt(l_norm);

            T e1 = T(obs_t(0)) * n_c_t(0) + T(obs_t(1)) * n_c_t(1) + n_c_t(2);
            T e2 = T(obs_t(2)) * n_c_t(0) + T(obs_t(3)) * n_c_t(1) + n_c_t(2);

            Eigen::Map<Vector2T> residual(residual_ptr);

            residual(0) = e1 / l_sqrtnorm;
            residual(1) = e2 / l_sqrtnorm;

            residual =
                    general_line::GeneralLineErrorOneCameraTerm::sqrtInfo_.cast<T>() *
                    residual;

            return true;
        }

        static ceres::CostFunction *create(const Eigen::Vector4d &obs_t_,
                                           const Eigen::Vector3d &x_dir,
                                           const Eigen::Matrix3d &R_c_l) {
            return (new ceres::AutoDiffCostFunction<GeneralLineErrorOneCameraTerm, 2, 2,
                    7, 7>(
                    new GeneralLineErrorOneCameraTerm(obs_t_, x_dir, R_c_l)));
        }

    public:
        static Eigen::Matrix2d
                sqrtInfo_; // the inverse square root of the measurement covariance matrix
        Eigen::Vector4d obs_t;
        Eigen::Vector3d x_dir_;
        Eigen::Matrix3d R_c_l;
    };

    class GeneralLineErrorTwoCameraTerm {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GeneralLineErrorTwoCameraTerm(const Eigen::Vector4d &obs_t_,
                                      const Eigen::Vector3d &x_dir,
                                      const Eigen::Matrix3d &R_c_l_)
                : obs_t(obs_t_), x_dir_(x_dir), R_c_l(R_c_l_) {}

        template<typename T>
        // theta, rho: line parameter
        // phi: mahata angle
        bool operator()(const T *const line_ptr, const T *const T_w_i_h_ptr,
                        const T *const T_w_i_t_ptr, const T *const T_i_c_ptr,
                        T *residual_ptr) const {

            using Vector3T = Eigen::Matrix<T, 3, 1>;
            using Vector2T = Eigen::Matrix<T, 2, 1>;
            using Vector6T = Eigen::Matrix<T, 6, 1>;
            using Matrix33T = Eigen::Matrix<T, 3, 3>;
            using QuaternionT = Eigen::Quaternion<T>;
            using TransformT = Twist<T>;

            // line parameter
            T rho = line_ptr[0];
            T theta = line_ptr[1];

            Eigen::Map<const QuaternionT> q_w_i_h(T_w_i_h_ptr + 3);
            Eigen::Map<const Vector3T> t_w_i_h(T_w_i_h_ptr);

            Eigen::Map<const QuaternionT> q_w_i_t(T_w_i_t_ptr + 3);
            Eigen::Map<const Vector3T> t_w_i_t(T_w_i_t_ptr);

            Eigen::Map<const QuaternionT> q_i_c(T_i_c_ptr + 3);
            Eigen::Map<const Vector3T> t_i_c(T_i_c_ptr);

            TransformT T_i_c(q_i_c, t_i_c);

            TransformT T_w_i_h(q_w_i_h, t_w_i_h);
            TransformT T_w_i_t(q_w_i_t, t_w_i_t);

            TransformT T_w_c_h = T_w_i_h * T_i_c;
            TransformT T_w_c_t = T_w_i_t * T_i_c;

            TransformT T_c_w_h = T_w_c_h.inverse();
            TransformT T_c_w_t = T_w_c_t.inverse();

            TransformT T_t_c_h_c = T_w_c_t.inverse() * T_w_c_h;
            Matrix33T R_t_c_h_c = T_t_c_h_c.rotationMatrix();


            Vector3T line_c;
            line_c << ceres::cos(theta), ceres::sin(theta), T(0);
            line_c = R_c_l.cast<T>() * line_c;


            Vector3T origin_t =  R_t_c_h_c * x_dir_.cast<T>() + rho*T_t_c_h_c.pos;
            Vector3T line_c_t = R_t_c_h_c * line_c;
            Vector3T n_c_t = line_c_t.cross(origin_t);

            T l_norm = n_c_t(0) * n_c_t(0) + n_c_t(1) * n_c_t(1);
            T l_sqrtnorm = ceres::sqrt(l_norm);

            T e1 = T(obs_t(0)) * n_c_t(0) + T(obs_t(1)) * n_c_t(1) + n_c_t(2);
            T e2 = T(obs_t(2)) * n_c_t(0) + T(obs_t(3)) * n_c_t(1) + n_c_t(2);

            Eigen::Map<Vector2T> residual(residual_ptr);
            residual(0) = e1 / l_sqrtnorm;
            residual(1) = e2 / l_sqrtnorm;

            residual =
                    general_line::GeneralLineErrorTwoCameraTerm::sqrtInfo_.cast<T>() *
                    residual;
            return true;
        }

        static ceres::CostFunction *create(const Eigen::Vector4d &obs_t_,
                                           const Eigen::Vector3d &x_dir,
                                           const Eigen::Matrix3d &R_c_l) {

            return (new ceres::AutoDiffCostFunction<GeneralLineErrorTwoCameraTerm, 2, 2,
                    7, 7, 7>(
                    new GeneralLineErrorTwoCameraTerm(obs_t_, x_dir, R_c_l)));
        }

    public:
        static Eigen::Matrix2d
                sqrtInfo_; // the inverse square root of the measurement covariance matrix
        Eigen::Vector4d obs_t;
        Eigen::Vector3d x_dir_;
        Eigen::Matrix3d R_c_l;
    };
} // namespace general_line

#endif // SRC_GENERAL_LINE_FACTOR_H
