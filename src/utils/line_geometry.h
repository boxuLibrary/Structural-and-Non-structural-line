//
// Created by hyj on 17-12-8.
//

#ifndef VINS_ESTIMATOR_LINE_GEOMETRY_H
#define VINS_ESTIMATOR_LINE_GEOMETRY_H
#include <eigen3/Eigen/Dense>
#include <ceres/jet.h>
using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 6, 6> Matrix6d;

Vector4d line_to_orth(Vector6d line);
Vector6d orth_to_line(Vector4d orth);
Vector4d plk_to_orth(Vector6d plk);
Vector6d orth_to_plk(Vector4d orth);

Vector4d pi_from_ppp(Vector3d x1, Vector3d x2, Vector3d x3);
Vector6d pipi_plk(Vector4d pi1, Vector4d pi2);
Vector3d plucker_origin(Vector3d n, Vector3d v);
Matrix3d skew_symmetric(Vector3d v);

Vector3d poit_from_pose(Eigen::Matrix3d Rcw, Eigen::Vector3d tcw,
                        Vector3d pt_c);
Vector3d point_to_pose(Eigen::Matrix3d Rcw, Eigen::Vector3d tcw, Vector3d pt_w);
Vector6d line_to_pose(Vector6d line_w, Eigen::Matrix3d Rcw,
                      Eigen::Vector3d tcw);
Vector6d line_from_pose(Vector6d line_c, Eigen::Matrix3d Rcw,
                        Eigen::Vector3d tcw);

Vector6d plk_to_pose(Vector6d plk_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw);
Vector6d plk_from_pose(Vector6d plk_c, Eigen::Matrix3d Rcw,
                       Eigen::Vector3d tcw);

// new add
Eigen::Vector3d pluckerOrigin(const Vector6d &plk);



template <typename T>
Eigen::Matrix<T, 3, 3> skew_symmetric(Eigen::Matrix<T, 3, 1> v) {
  Eigen::Matrix<T, 3, 3> S;
  S << T(0), -v(2), v(1), v(2), T(0), -v(0), -v(1), v(0), T(0);
  return S;
}
template <typename T>
Eigen::Matrix<T, 6, 1> plk_to_pose(Eigen::Matrix<T, 6, 1> plk_w,
                                   Eigen::Matrix<T, 3, 3> Rcw,
                                   Eigen::Matrix<T, 3, 1> tcw) {

  Eigen::Matrix<T, 3, 1> nw = plk_w.head(3);
  Eigen::Matrix<T, 3, 1> vw = plk_w.tail(3);

  Eigen::Matrix<T, 3, 1> nc = Rcw * nw + skew_symmetric(tcw) * Rcw * vw;
  Eigen::Matrix<T, 3, 1> vc = Rcw * vw;

  Eigen::Matrix<T, 6, 1> plk_c;
  plk_c.head(3) = nc;
  plk_c.tail(3) = vc;
  return plk_c;
}

template <typename T>
Eigen::Matrix<T, 6, 1> plk_from_pose(Eigen::Matrix<T, 6, 1> plk_c,
                                     Eigen::Matrix<T, 3, 3> Rcw,
                                     Eigen::Matrix<T, 3, 1> tcw) {

  Eigen::Matrix<T, 3, 3> Rwc = Rcw.transpose();
  Eigen::Matrix<T, 3, 1> twc = -Rwc * tcw;
  return plk_to_pose(plk_c, Rwc, twc);
}

inline Eigen::Vector3d pluckerOrigin(const Vector6d &plk) {

  const Eigen::Vector3d &n = plk.head<3>();
  const Eigen::Vector3d &v = plk.tail<3>();
  Eigen::Vector3d f = n.cross(v);
  double g = v.squaredNorm();

  Eigen::Vector3d p0 = 1. / g * f;

  return p0;
}

template <typename T>
Eigen::Matrix<T, 6, 1> orth_to_plk( Eigen::Matrix<T, 4, 1> orth) {

    Eigen::Matrix<T, 6, 1> plk;

    Eigen::Matrix<T, 3, 1> theta = orth.head(3);
    T phi = orth[3];

    T s1 = ceres::sin(theta[0]);
    T c1 = ceres::cos(theta[0]);
    T s2 = ceres::sin(theta[1]);
    T c2 = ceres::cos(theta[1]);
    T s3 = ceres::sin(theta[2]);
    T c3 = ceres::cos(theta[2]);

    Matrix<T, 3, 3> R;
    R <<
            c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3,
            c2 * s3, s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3,
            -s2, s1 * c2, c1 * c2;

    T w1 = ceres::cos(phi);
    T w2 = ceres::sin(phi);
    T d = w1 / w2;      // 原点到直线的距离

    Matrix<T, 3, 1> u1 = R.col(0);
    Matrix<T, 3, 1> u2 = R.col(1);

    Matrix<T, 3, 1> n = w1 * u1;
    Matrix<T, 3, 1> v = w2 * u2;

    plk.head(3) = n;
    plk.tail(3) = v;

    return plk;
}

inline Vector6d TrimLine(const Eigen::Vector3d &spt, const Eigen::Vector3d &ept,
                         const Vector6d &plk) {

  Eigen::Vector3d p11 = spt / spt.z();
  Eigen::Vector3d p21 = ept / ept.z();
  Eigen::Vector2d ln = (p11.cross(p21)).head(2);
  ln = ln / ln.norm();

  Eigen::Vector3d p12 = Eigen::Vector3d(p11(0) + ln(0), p11(1) + ln(1),
                                        1.0); // 直线垂直方向上移动一个单位
  Eigen::Vector3d p22 = Eigen::Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
  Eigen::Vector3d cam = Eigen::Vector3d(0, 0, 0);

  Eigen::Vector4d pi1 = pi_from_ppp(cam, p11, p12);
  Eigen::Vector4d pi2 = pi_from_ppp(cam, p21, p22);

  Eigen::Matrix4d Lc;
  Eigen::Vector3d nc, vc;

  nc = plk.head(3);
  vc = plk.tail(3);

  Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

  Eigen::Vector4d e1 = Lc * pi1;
  Eigen::Vector4d e2 = Lc * pi2;
  e1 = e1 / e1(3);
  e2 = e2 / e2(3);

  Eigen::Vector3d pts_1(e1(0), e1(1), e1(2));
  Eigen::Vector3d pts_2(e2(0), e2(1), e2(2));

  Vector6d res;
  res.head(3) = pts_1;
  res.tail(3) = pts_2;
  return res;
}

#endif // VINS_ESTIMATOR_LINE_GEOMETRY_H
