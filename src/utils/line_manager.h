//
// Created by xubo on 21-3-17.
//

#ifndef SIMULATION_LINE_LINE_MANAGER_H
#define SIMULATION_LINE_LINE_MANAGER_H

#include "utils/EigenTypes.h"
#include "utils/Twist.h"
#include "utils/line_geometry.h"
#include "utils/tic_toc.h"

#include "factor/general_line_parameterization.h"
#include "factor/general_line_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/plucker_line_parameterization.h"
#include "factor/plucker_projection_factor.h"

#include <fstream>
#include <iostream>
#include <random>

struct LineLandmark {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool is_triangulation{false};
    bool boptimize_success{false};
    int id;

    std::array<double, 2> para_data{};
    Eigen::Vector2d para;
    Eigen::Vector3d x_dir;
    Eigen::Vector3d y_dir;
    Eigen::Vector3d z_dir;

    Eigen::Matrix3d R_c_l;

    Eigen::Vec6d plk_w;
    Eigen::Vec6d plk_h;
    Eigen::Vec3d spt_w;
    Eigen::Vec3d ept_w;

    // groudtruth
    Eigen::Vec6d plk_w_gt;
    Eigen::Vector3d spt_w_gt;
    Eigen::Vector3d ept_w_gt;

    // plucker optimazation
    Eigen::Vec4d line_orth_c;
};

struct line_obs {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector4d obs;
    int landmark_id;
};

void read_camera_pose(
        const std::string &path, Eigen::aligned_vector<Transformd> &poses,
        std::vector<std::array<double, 7>> &camera_pose_parameters);

void read_line_obs(
        const std::string &path,
        Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map);

void add_noise_obs(Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map, double noise);

void read_linelandmark(
        const std::string &path,
        Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks);

void triangulate_line(
        Eigen::aligned_unordered_map<int, LineLandmark>& line_landmarks,
        Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
        Eigen::aligned_vector<Transformd> &poses, bool plucker_optimize);

void optimize_line_without_plucker(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                   Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                   std::vector<std::array<double, 7>> &camera_pose_parameters, std::array<double, 7> &T_i_c_paramter);

void optimize_line_without_plucker2(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                   Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                   std::vector<std::array<double, 7>> &camera_pose_parameters, std::array<double, 7> &T_i_c_paramter);

void optimize_line_with_plucker(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                   Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                   std::vector<std::array<double, 7>> &camera_pose_parameters, std::array<double, 7> &T_i_c_paramter);

void optimize_line_with_plucker2(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                std::vector<std::array<double, 7>> &camera_pose_parameters, std::array<double, 7> &T_i_c_paramter);

void updatelineplk(
        Eigen::aligned_unordered_map<int, LineLandmark>& line_landmarks,
        Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
        Eigen::aligned_vector<Transformd> &poses, bool plucker_optimize);

void savefeatures( Eigen::aligned_unordered_map<int, LineLandmark>& line_landmarks,  bool plucker_optimize);



#endif //SIMULATION_LINE_LINE_MANAGER_H
