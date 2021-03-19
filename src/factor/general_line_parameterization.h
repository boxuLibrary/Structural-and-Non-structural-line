//
// Created by ubuntu on 2020/9/9.
//

#ifndef STRUCTLINELOCALPARAMETERIZATION_H
#define STRUCTLINELOCALPARAMETERIZATION_H

#include <ceres/ceres.h>
#include "normalizeAngle.h"

namespace general_line {
// 定义的误差函数这里是
// define a local parameterization for upadting the angle to be constrained in [-pi, pi)
    class GeneralLineLocalParameterization {
    public:
        template<typename T>
        bool operator()(const T *theta, const T *delta, T *thetaPlusDelta) const {

            // 逆深度可以直接加减

            thetaPlusDelta[0] = theta[0] + delta[0];
            thetaPlusDelta[1] = normalizeAngle(theta[1] + T(1.0) * delta[1]);

            return true;
        }

        static ceres::LocalParameterization *create() {
            return (new ceres::AutoDiffLocalParameterization<GeneralLineLocalParameterization, 2, 2>);
        }
    };
} // namespace struct_vio
#endif // ANGLELOCALPARAMETERIZATION_H
