#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "utils/utility.h"

class PoseLocalParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

    virtual bool ComputeJacobian(const double *x, double *jacobian) const;

    virtual int GlobalSize() const { return 7; };

    virtual int LocalSize() const { return 6; };
public:
    static void Global2LocalJacobian(const double *x, Eigen::Matrix<double, 7, 6, Eigen::RowMajor> *jacobian);

    static void Local2GlobalJacobian(const double *x, Eigen::Matrix<double, 6, 7, Eigen::RowMajor> *jacobian);
};
