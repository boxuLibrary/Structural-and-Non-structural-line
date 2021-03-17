#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

class LineOrthParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

    virtual bool ComputeJacobian(const double *x, double *jacobian) const;

    virtual int GlobalSize() const { return 4; };

    virtual int LocalSize() const { return 4; };
};

class PluckerLineLocalParameterization {
public:
    template<typename T>
    bool operator()(const T *x, const T *delta, T *xPlusDelta) const {

        using Vector3T = Eigen::Matrix<T, 3, 1>;
        using Matrix3T = Eigen::Matrix<T, 3, 3>;
        using Matrix2T = Eigen::Matrix<T, 2, 2>;

        // 逆深度可以直接加减
        // theta --> U,  phi --> W
        Eigen::Map<const Vector3T> theta(x);
        T phi = *(x + 3);
        //Vector3d theta = orth.head(3);
        //double phi = orth[3];
        T s1 = ceres::sin(theta[0]);
        T c1 = ceres::cos(theta[0]);
        T s2 = ceres::sin(theta[1]);
        T c2 = ceres::cos(theta[1]);
        T s3 = ceres::sin(theta[2]);
        T c3 = ceres::cos(theta[2]);
        Matrix3T R;
        R <<
          c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3,
                c2 * s3, s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3,
                -s2, s1 * c2, c1 * c2;
        T w1 = ceres::cos(phi);
        T w2 = ceres::sin(phi);

        // update
        Eigen::Map<const Vector3T> _delta_theta(delta);
        T _delta_phi = *(delta + 3);
        Matrix3T Rz;
        Rz << ceres::cos(_delta_theta(2)), -ceres::sin(_delta_theta(2)), T(0),
                ceres::sin(_delta_theta(2)), ceres::cos(_delta_theta(2)), T(0),
                T(0), T(0), T(1);

        Matrix3T Ry;
        Ry << ceres::cos(_delta_theta(1)), T(0.), ceres::sin(_delta_theta(1)),
                T(0.), T(1.), T(0.),
                -ceres::sin(_delta_theta(1)), T(0.), ceres::cos(_delta_theta(1));

        Matrix3T Rx;
        Rx << T(1.), T(0.), T(0.),
                T(0.), ceres::cos(_delta_theta(0)), -ceres::sin(_delta_theta(0)),
                T(0.), ceres::sin(_delta_theta(0)), ceres::cos(_delta_theta(0));
        R = R * Rx * Ry * Rz;

        Matrix2T W;
        W << w1, -w2, w2, w1;
        Matrix2T delta_W;
        delta_W << cos(_delta_phi), -sin(_delta_phi), sin(_delta_phi), cos(_delta_phi);
        W = W * delta_W;

        // U' -- > theta'. W' --> phi'
        Eigen::Map<Vector3T> theta_pluse(xPlusDelta);                             // double 指针 转为eigen数组
        T *phi_plus(xPlusDelta + 3);

        Vector3T u1 = R.col(0);
        Vector3T u2 = R.col(1);
        Vector3T u3 = R.col(2);
        theta_pluse[0] = atan2(u2(2), u3(2));
        theta_pluse[1] = asin(-u1(2));
        theta_pluse[2] = atan2(u1(1), u1(0));

        *xPlusDelta = ceres::asin(W(1, 0));


        return true;
    }

    static ceres::LocalParameterization *create() {
        return (new ceres::AutoDiffLocalParameterization<PluckerLineLocalParameterization, 4, 4>);
    }
};