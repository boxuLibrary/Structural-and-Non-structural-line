#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utils/Twist.h"
#include "utils/line_geometry.h"

class lineProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 4> {
public:
    lineProjectionFactor(const Eigen::Vector4d &_pts_i);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

///////////////////////////////line in camera frame///////////////////////////////////////////
class lineProjectionFactor_incamera : public ceres::SizedCostFunction<2, 7, 7, 7, 4> {
public:
    lineProjectionFactor_incamera(const Eigen::Vector4d &_pts_i);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class lineProjectionFactor_instartframe : public ceres::SizedCostFunction<2, 4> {
public:
    lineProjectionFactor_instartframe(const Eigen::Vector4d &_pts_i);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};


class PluckerLineErrorOneCameraTerm {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PluckerLineErrorOneCameraTerm(const Eigen::Vector4d &_pts_i)
            : obs_i(_pts_i) {}

    template<typename T>
    // theta, rho: line parameter
    // phi: mahata angle
    bool operator()(const T *const line_ptr, const T *const T_w_i_t_ptr,
                    const T *const T_i_c_ptr, T *residual_ptr) const {

        using Vector3T = Eigen::Matrix<T, 3, 1>;
        using Vector4T = Eigen::Matrix<T, 4, 1>;
        using Vector2T = Eigen::Matrix<T, 2, 1>;
        using Matrix33T = Eigen::Matrix<T, 3, 3>;
        using Vector6T = Eigen::Matrix<T, 6, 1>;
        using QuaternionT = Eigen::Quaternion<T>;
        using TransformT = Twist<T>;

        // line parameter

        Eigen::Map<const QuaternionT> q_w_i_t(T_w_i_t_ptr + 3);
        Eigen::Map<const Vector3T> t_w_i_t(T_w_i_t_ptr);

        Eigen::Map<const QuaternionT> q_i_c(T_i_c_ptr + 3);
        Eigen::Map<const Vector3T> t_i_c(T_i_c_ptr);

        TransformT T_i_c(q_i_c, t_i_c);

        TransformT T_w_i_t(q_w_i_t, t_w_i_t);

        TransformT T_w_c_t = T_w_i_t * T_i_c;

        Vector4T line_orth;
        line_orth << line_ptr[0], line_ptr[1], line_ptr[2], line_ptr[3];


        Vector6T line_w = orth_to_plk(line_orth);

        Vector6T line_c = plk_from_pose(line_w, T_w_c_t.rotationMatrix(), T_w_c_t.pos);

        Vector3T n_c = line_c.head(3);

        T l_norm = n_c(0) * n_c(0) + n_c(1) * n_c(1);

        T l_sqrtnorm = ceres::sqrt(l_norm);

        T e1 = T(obs_i(0)) * n_c(0) + T(obs_i(1)) * n_c(1) + n_c(2);
        T e2 = T(obs_i(2)) * n_c(0) + T(obs_i(3)) * n_c(1) + n_c(2);

        Eigen::Map<Vector2T> residual(residual_ptr);

        residual(0) = e1 / l_sqrtnorm;
        residual(1) = e2 / l_sqrtnorm;

        residual = PluckerLineErrorOneCameraTerm::sqrtInfo_.cast<T>() *
                   residual;
        return true;
    }

    static ceres::CostFunction *create(const Eigen::Vector4d &obs_t_) {
        return (new ceres::AutoDiffCostFunction<PluckerLineErrorOneCameraTerm, 2, 4,
                7, 7>(
                new PluckerLineErrorOneCameraTerm(obs_t_)));
    }


public:
    static Eigen::Matrix2d
            sqrtInfo_; // the inverse square root of the measurement covariance matrix
    Eigen::Vector4d obs_i;
};






class PluckerLineErrorTwoCameraTerm {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PluckerLineErrorTwoCameraTerm(const Eigen::Vector4d &_pts_i)
            : obs_i(_pts_i) {}

    template<typename T>
    // theta, rho: line parameter
    // phi: mahata angle
    bool operator()(const T *const line_ptr, const T *const T_w_i_h_ptr, const T *const T_w_i_t_ptr,
                    const T *const T_i_c_ptr, T *residual_ptr) const {

        using Vector3T = Eigen::Matrix<T, 3, 1>;
        using Vector4T = Eigen::Matrix<T, 4, 1>;
        using Vector2T = Eigen::Matrix<T, 2, 1>;
        using Matrix33T = Eigen::Matrix<T, 3, 3>;
        using Vector6T = Eigen::Matrix<T, 6, 1>;
        using QuaternionT = Eigen::Quaternion<T>;
        using TransformT = Twist<T>;

        // line parameter

        Eigen::Map<const QuaternionT> q_w_i_h(T_w_i_h_ptr + 3);
        Eigen::Map<const Vector3T> t_w_i_h(T_w_i_h_ptr);

        Eigen::Map<const QuaternionT> q_w_i_t(T_w_i_t_ptr + 3);
        Eigen::Map<const Vector3T> t_w_i_t(T_w_i_t_ptr);

        Eigen::Map<const QuaternionT> q_i_c(T_i_c_ptr + 3);
        Eigen::Map<const Vector3T> t_i_c(T_i_c_ptr);

        TransformT T_i_c(q_i_c, t_i_c);

        TransformT T_w_i_t(q_w_i_t, t_w_i_t);

        TransformT T_w_i_h(q_w_i_h, t_w_i_h);

        TransformT T_w_c_t = T_w_i_t * T_i_c;
        TransformT T_w_c_h = T_w_i_h * T_i_c;

        Vector4T line_orth_h;
        line_orth_h << line_ptr[0], line_ptr[1], line_ptr[2], line_ptr[3];

        Vector6T line_h = orth_to_plk(line_orth_h);

        Vector6T line_w = plk_to_pose(line_h, T_w_c_h.rotationMatrix(), T_w_c_h.pos);


        Vector6T line_c = plk_from_pose(line_w, T_w_c_t.rotationMatrix(), T_w_c_t.pos);

        Vector3T n_c = line_c.head(3);

        T l_norm = n_c(0) * n_c(0) + n_c(1) * n_c(1);

        T l_sqrtnorm = ceres::sqrt(l_norm);

        T e1 = T(obs_i(0)) * n_c(0) + T(obs_i(1)) * n_c(1) + n_c(2);
        T e2 = T(obs_i(2)) * n_c(0) + T(obs_i(3)) * n_c(1) + n_c(2);

        Eigen::Map<Vector2T> residual(residual_ptr);

        residual(0) = e1 / l_sqrtnorm;
        residual(1) = e2 / l_sqrtnorm;

        residual = PluckerLineErrorTwoCameraTerm::sqrtInfo_.cast<T>() *
                   residual;
        return true;
    }

    static ceres::CostFunction *create(const Eigen::Vector4d &obs_t_) {
        return (new ceres::AutoDiffCostFunction<PluckerLineErrorTwoCameraTerm, 2, 4,
                7, 7, 7>(
                new PluckerLineErrorTwoCameraTerm(obs_t_)));
    }


public:
    static Eigen::Matrix2d
            sqrtInfo_; // the inverse square root of the measurement covariance matrix
    Eigen::Vector4d obs_i;
};








