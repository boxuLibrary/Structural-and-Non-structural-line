//
// Created by ubuntu on 2021/3/6.
//

#include "utils/line_manager.h"

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <fstream>
#include <iostream>
#include <set>


bool plucker_optimize = 0; // PLUCKER_OPTIMIZE    LINE_OPTIMIZE

using namespace std;


int main(int argc, char **argv) {

    general_line::GeneralLineErrorOneCameraTerm::sqrtInfo_ =
            460 / 3.2 * Matrix2d::Identity();
    general_line::GeneralLineErrorTwoCameraTerm::sqrtInfo_ =
            460 / 3.2 * Matrix2d::Identity();

    lineProjectionFactor::sqrt_info = 460 / 3.2 * Matrix2d::Identity();

    PluckerLineErrorOneCameraTerm::sqrtInfo_ = 460 / 3.2 * Matrix2d::Identity();


    std::string pose_path =
            "../bin/cam_pose_tum.txt";

    //  "/home/ubuntu/Downloads/vio_data_simulation/bin/keyframe/all_lines_599.txt"
    std::string line_obs_path =
            "../bin/keyframe/all_lines_";

    std::string line_landmarks_path =
            "../bin/house_model/house.txt";

    std::string line_landmarks_path_noise =
            "../bin/house_model/house_noise.txt";

    // step1: read camera poses
    Eigen::aligned_vector<Transformd> camera_poses;
    std::vector<std::array<double, 7>> camera_pose_parameters;
    std::array<double, 7> T_i_c_paramter{0, 0, 0, 0, 0, 0, 1};
    read_camera_pose(pose_path, camera_poses, camera_pose_parameters);

    // step2: read line landmak groudtruth
    Eigen::aligned_unordered_map<int, LineLandmark> line_landmarks;
    read_linelandmark(line_landmarks_path, line_landmarks);

    // step3: read line observation
    Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> line_obs_map;
    read_line_obs(line_obs_path, line_obs_map);

    std::map<int, std::set<int>> line2cam_map;

    // step4: triangulation line

    cout << "line_landmark.size(): " << line_landmarks.size() << endl;

    triangulate_line(line_landmarks,line_obs_map, camera_poses, plucker_optimize);

    if(plucker_optimize)
    {
        optimize_line_with_plucker(line_landmarks, line_obs_map, camera_pose_parameters, T_i_c_paramter);

    } else
    {
        optimize_line_without_plucker(line_landmarks, line_obs_map, camera_pose_parameters, T_i_c_paramter);
    }

    updatelineplk(line_landmarks, line_obs_map, camera_poses, plucker_optimize);
    savefeatures(line_landmarks, plucker_optimize);

    return 0;
}