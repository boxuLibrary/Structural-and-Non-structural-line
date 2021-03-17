//
// Created by ubuntu on 2020/9/9.
//

#ifndef NORMALIZEANGLE_H
#define NORMALIZEANGLE_H

#pragma once
#include <ceres/ceres.h>

namespace general_line {
// normalize the angle in radius between [-pi, pi)
template <typename T> inline T normalizeAngle(const T &angle) {
  // use ceres::floor because it is specialized for double and Jet types
  T twoPi{2.0 * M_PI};
  return angle - twoPi * ceres::floor((angle + T(M_PI)) / twoPi);
}
}

#endif // NORMALIZEANGLE_H
