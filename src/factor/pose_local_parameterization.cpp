#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta,
                                     double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq =
            Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseLocalParameterization::ComputeJacobian(const double *x,
                                                double *jacobian) const {
    //    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    //    j.topRows<6>().setIdentity();
    //    j.bottomRows<1>().setZero();

    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    Eigen::Map<const Eigen::Quaterniond> q(x + 3);
    j.setZero();
    j.topLeftCorner<3, 3>().setIdentity();
    j.block<4, 3>(3, 3) = Utility::Qleft_LocalParameter(q);
    return true;
}

void PoseLocalParameterization::Global2LocalJacobian(
        const double *x, Eigen::Matrix<double, 7, 6, Eigen::RowMajor> *jacobian) {
    Eigen::Map<const Eigen::Quaterniond> q(x + 3);
    jacobian->setZero();
    jacobian->topLeftCorner<3, 3>().setIdentity();
    jacobian->block<4, 3>(3, 3) = Utility::Qleft_LocalParameter(q);
}

void PoseLocalParameterization::Local2GlobalJacobian(
        const double *x, Eigen::Matrix<double, 6, 7, Eigen::RowMajor> *jacobian) {
    Eigen::Map<const Eigen::Quaterniond> q(x + 3);
    jacobian->setZero();
    jacobian->topLeftCorner<3, 3>().setIdentity();
    jacobian->block<3, 4>(3, 3) = 2 * Utility::Qleft(q.inverse()).topRows(3);
}
