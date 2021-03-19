//
// Created by xubo on 21-3-17.
//
#include "line_manager.h"


void read_camera_pose(
        const std::string &path, Eigen::aligned_vector<Transformd> &poses,
        std::vector<std::array<double, 7>> &camera_pose_parameters) {

    poses.clear();
    camera_pose_parameters.clear();

    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line[0] == '#')
            continue;

        std::stringstream ss(line);

        char tmp;
        double timestamp;
        Eigen::Quaterniond q;
        Eigen::Vector3d pos;

        ss >> timestamp >> pos[0] >> pos[1] >> pos[2] >> q.x() >> q.y() >> q.z() >>
           q.w();

        std::array<double, 7> parameter{pos[0], pos[1], pos[2], q.x(),
                                        q.y(), q.z(), q.w()};

        //    std::cout << std::fixed << pos.transpose() << " " <<
        //    q.coeffs().transpose()
        //              << std::endl;
        poses.push_back(Transformd(q, pos));
        camera_pose_parameters.push_back(parameter);
    }

    f.close();
}

void read_line_obs(
        const std::string &path,
        Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map) {
    line_obs_map.clear();

    for (int i = 0; i < 600; i++) {
        // std::cout << "==============================" << std::endl;
        // std::cout << "camera id: " << i << std::endl;
        std::string obs_path = path + std::to_string(i) + ".txt";
        std::ifstream f(obs_path);
        std::string line;
        while (std::getline(f, line)) {
            if (line[0] == '#')
                continue;

            std::stringstream ss(line);

            line_obs observation;

            ss >> observation.obs[0] >> observation.obs[1] >> observation.obs[2] >>
               observation.obs[3] >> observation.landmark_id;

            line_obs_map[observation.landmark_id][i] = observation;
//            std::cout << std::fixed << observation.obs.transpose() << " "
//                      << observation.landmark_id << std::endl;
        }

        f.close();
    }
}

void add_noise_obs(Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map, double noise) {


    for (auto &landmark: line_obs_map) {

        for (auto &obs:landmark.second) {

            std::random_device rd_sp;
            std::default_random_engine generator_sp(rd_sp());
            std::normal_distribution<double> noise_sp(0.0, noise);
            Eigen::Vector2d noise_line_sp(noise_sp(generator_sp), noise_sp(generator_sp));

            std::random_device rd_ep;
            std::default_random_engine generator_ep(rd_ep());
            std::normal_distribution<double> noise_ep(0.0, noise);
            Eigen::Vector2d noise_line_ep(noise_ep(generator_ep), noise_ep(generator_ep));


            noise_line_sp = noise_line_sp / 460;
            noise_line_ep = noise_line_ep / 460;

            obs.second.obs[0] = obs.second.obs[0] + noise_line_sp[0];
            obs.second.obs[1] = obs.second.obs[1] + noise_line_sp[1];
            obs.second.obs[2] = obs.second.obs[2] + noise_line_ep[0];
            obs.second.obs[3] = obs.second.obs[3] + noise_line_ep[1];

        }

    }


}


void read_linelandmark(
        const std::string &path,
        Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks) {
    line_landmarks.clear();

    std::ifstream f(path);

    std::string line;
    int count = 0;
    while (std::getline(f, line)) {
        if (line[0] == '#')
            continue;

        std::stringstream ss(line);


        LineLandmark landmark;

        ss >> landmark.spt_w_gt[0] >> landmark.spt_w_gt[1] >>
           landmark.spt_w_gt[2] >> landmark.ept_w_gt[0] >> landmark.ept_w_gt[1] >>
           landmark.ept_w_gt[2];


        landmark.id = count;
        count++;

        Eigen::Vec3d nc, vc;
        nc = landmark.spt_w_gt.cross(landmark.ept_w_gt);
        vc = landmark.ept_w_gt - landmark.spt_w_gt;

        landmark.plk_w_gt.head<3>() = nc;
        landmark.plk_w_gt.tail<3>() = vc;

        line_landmarks[landmark.id] = landmark;
    }

    f.close();
}

void triangulate_line(
        Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
        Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
        Eigen::aligned_vector<Transformd> &poses, bool plucker_optimize) {

    for (auto &landmark : line_landmarks) {
        int landmark_id = landmark.second.id;

        if (line_obs_map.find(landmark_id) == line_obs_map.end() or
            line_obs_map.at(landmark_id).size() < 2) {
            continue;
        }

        int start_frame_id = line_obs_map.at(landmark_id).begin()->first;
        Transformd T_w_s = poses.at(start_frame_id);

        Eigen::Vector4d obs_line =
                line_obs_map.at(landmark_id).at(start_frame_id).obs;


        Eigen::Vector3d spt{obs_line(0), obs_line(1), 1};
        Eigen::Vector3d ept{obs_line(2), obs_line(3), 1};

        Eigen::Vector4d plane_para = pi_from_ppp(spt, ept, Eigen::Vector3d(0, 0, 0));
        Eigen::Vector3d plane_norm = plane_para.head(3);
        plane_norm.normalize();

        Eigen::Vector3d mpt = (spt + ept) / 2;
        // Eigen::Vector3d mpt = spt;

        Eigen::Vector3d y_dir = mpt.normalized();
        Eigen::Vector3d x_dir = y_dir.cross(plane_norm);
        x_dir.norm();

        landmark.second.x_dir = y_dir;
        landmark.second.y_dir = x_dir;
        landmark.second.z_dir = plane_norm;

        landmark.second.R_c_l.col(0) = x_dir;
        landmark.second.R_c_l.col(1) = y_dir;
        landmark.second.R_c_l.col(2) = plane_norm;

        // Eigen::Vector3d origin = y_dir * 5;

        Eigen::Vector3d obs_dir = ept - spt;

        obs_dir.normalize();

        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obsi, obsj; // obs from two frame are used to do triangulation

        // plane pi from ith obs in ith camera frame
        Eigen::Vector4d pii;
        Eigen::Vector3d ni; // normal vector of plane

        double min_cos_theta = 1;
        for (const auto &obs : line_obs_map.at(landmark_id)) // 遍历所有的观测， 注意
        {

            int target_frame_id = obs.first;
            if (target_frame_id == start_frame_id) // 第一个观测是start frame 上
            {

                obsi = obs.second.obs;
                Eigen::Vector3d p1(obs.second.obs[0], obs.second.obs[1], 1);
                Eigen::Vector3d p2(obs.second.obs[2], obs.second.obs[3], 1);
                pii = pi_from_ppp(p1, p2, Vector3d(0, 0, 0));
                ni = pii.head(3);
                ni.normalize();
                continue;
            }

            Transformd T_w_t = poses.at(target_frame_id);
            Transformd T_s_t = T_w_s.inverse() * T_w_t;
            // 非start frame(其他帧)上的观测

            Eigen::Vector3d t = T_s_t.pos;              // tij
            Eigen::Matrix3d R = T_s_t.rotationMatrix(); // Rij

            // plane pi from jth obs in ith camera frame
            Vector3d p3(obs.second.obs[0], obs.second.obs[1], 1);
            Vector3d p4(obs.second.obs[2], obs.second.obs[3], 1);
            p3 = R * p3 + t;
            p4 = R * p4 + t;
            Vector4d pij = pi_from_ppp(p3, p4, t);
            Eigen::Vector3d nj = pij.head(3);
            nj.normalize();

            double cos_theta = ni.dot(nj);
            if (cos_theta < min_cos_theta) {
                min_cos_theta = cos_theta;
                tij = t;
                Rij = R;
                obsj = obs.second.obs;
            }
        }


        if (min_cos_theta > 0.998) {
            continue;
            char tmp;
            std::cout << "parital is not enough" << std::endl;
            std::cin >> tmp;
        }


        // plane pi from jth obs in ith camera frame
        Vector3d p3(obsj(0), obsj(1), 1);
        Vector3d p4(obsj(2), obsj(3), 1);
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4, tij);

        Vector6d plk = pipi_plk(pii, pij);
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);

        landmark.second.plk_h = plk; // plk in camera frame
        landmark.second.is_triangulation = true;

        //  used to debug
        Vector3d pc, nc, vc;
        nc = landmark.second.plk_h.head(3);
        vc = landmark.second.plk_h.tail(3);

        Eigen::Vec6d plk_w;

        Vector6d line_w = plk_to_pose(landmark.second.plk_h, T_w_s.rotationMatrix(),
                                      T_w_s.pos); // transfrom to world frame

        std::cout << "triangulate dir: "
                  << line_w.tail<3>().normalized().transpose()
                  << " gt dir: " << landmark.second.plk_w_gt.tail<3>().normalized().transpose()
                  << std::endl;


        if (!plucker_optimize) {

            Eigen::Vector3d vc_l = landmark.second.R_c_l.transpose() * vc;
            double cos_thet = std::atan2(vc_l.y(), vc_l.x());

            Eigen::Vector3d line_origin = pluckerOrigin(line_w);
            double distance = landmark.second.x_dir.dot(line_origin);

            // landmark.para.x() = 1.0 / distance;
            landmark.second.para.x() = 0.2;

            //            landmark.second.para.y() = acos(cos_theta);
            landmark.second.para.y() = cos_thet;

        } else {

            // landmark.second.line_orth = plk_to_orth(line_w);

            // todo In order to equality, we use the same initial value to triangulate the line
            Eigen::Vector3d vc_l = landmark.second.R_c_l.transpose() * vc;
            double cos_thet = std::atan2(vc_l.y(), vc_l.x());
            double d = 1 / 0.2;


            Eigen::Vector3d origin_c = landmark.second.x_dir * d;

            Eigen::Vector3d line_c;
            line_c << cos(cos_thet), sin(cos_thet), 0;
            line_c = landmark.second.R_c_l * line_c;


            Eigen::Matrix<double, 6, 1> plk_c_h;
            plk_c_h.head(3) = origin_c.cross(line_c);
            plk_c_h.tail(3) = line_c;

            //landmark.second.line_orth_c = plk_to_orth(landmark.second.plk_h);
            landmark.second.line_orth_c = plk_to_orth(plk_c_h);
        }


    }

}


void updatelineplk(
        Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
        Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
        Eigen::aligned_vector<Transformd> &poses, bool plucker_optimize) {

    for (auto &landmark : line_landmarks) {
        int landmark_id = landmark.second.id;
        int host_frame_id = line_obs_map.at(landmark_id).begin()->first;


        if (!plucker_optimize) {
            double d = 1 / landmark.second.para[0];
            double theta = landmark.second.para[1];

            Eigen::Vector3d origin = landmark.second.x_dir * d;

            Eigen::Vector3d line_c;
            line_c << cos(theta), sin(theta), 0;

            line_c = landmark.second.R_c_l * line_c;


            Eigen::Matrix<double, 6, 1> plk_c_h;
            plk_c_h.head(3) = origin.cross(line_c);
            plk_c_h.tail(3) = line_c;

            landmark.second.plk_h = plk_c_h;

            Transformd T_w_s = poses.at(host_frame_id);

            Eigen::Vec6d line_w = plk_to_pose(landmark.second.plk_h, T_w_s.rotationMatrix(),
                                              T_w_s.pos); // transfrom to world frame
            landmark.second.plk_w = line_w; // transfrom to camera frame
        } else {


            Vector4d line_orth_c = landmark.second.line_orth_c;
            Eigen::Vec6d line_c = orth_to_plk(line_orth_c);
            Transformd T_w_s = poses.at(host_frame_id);
            Vector6d line_w = plk_to_pose(line_c, T_w_s.rotationMatrix(), T_w_s.pos); // transfrom to world frame


//            Vector4d line_orth_w = landmark.second.line_orth;
//            Eigen::Vec6d line_w = orth_to_plk(line_orth_w);
//            Transformd T_w_s = poses.at(host_frame_id);
//            Vector6d line_c = plk_from_pose(line_w, T_w_s.rotationMatrix(), T_w_s.pos);

            landmark.second.plk_h = line_c;
            landmark.second.plk_w = line_w;

        }


        Eigen::Vector3d line_origin = pluckerOrigin(landmark.second.plk_w);

        landmark.second.spt_w = line_origin;
        landmark.second.ept_w = line_origin;

        double d1 = 0;
        double d2 = 0;

        auto &line_obs = line_obs_map.at(landmark_id);

        for (const auto &obs : line_obs) {
            int target_id = obs.first;
            Transformd T_w_t = poses.at(target_id);
            Transformd T_t_w = T_w_t.inverse();

            Vector6d plk_t = plk_to_pose(landmark.second.plk_w, T_t_w.rotationMatrix(), T_t_w.pos);
            Eigen::Vec3d p11{obs.second.obs[0], obs.second.obs[1], 1.0};
            Eigen::Vec3d p21{obs.second.obs[2], obs.second.obs[3], 1.0};

            Vector6d cpt = TrimLine(p11, p21, plk_t);

            Eigen::Vector3d cpt1 = cpt.head<3>();
            Eigen::Vector3d cpt2 = cpt.tail<3>();

            if (cpt1(2) < 0 || cpt2(2) < 0) {
                break;
            }

            if ((cpt1 - cpt2).norm() > 8) {
                break;
            }

            landmark.second.boptimize_success = true;

            Eigen::Vector3d wpt1 = T_w_t * cpt1;
            Eigen::Vector3d wpt2 = T_w_t * cpt2;


            // line project distance

            Eigen::Vector3d line_dir;
            if (landmark.second.plk_w[5] < 0) {
                line_dir = -landmark.second.plk_w.tail<3>();
            } else {
                line_dir = landmark.second.plk_w.tail<3>();
            }

            double d1_tmp = (wpt1 - line_origin).dot(line_dir);
            double d2_tmp = (wpt2 - line_origin).dot(line_dir);

            if (d2_tmp < d1_tmp) {

                std::swap(d1_tmp, d2_tmp);
                Eigen::Vector3d tmp = wpt1;
                wpt1 = wpt2;
                wpt2 = tmp;
            }

            if (target_id == host_frame_id) {
                landmark.second.spt_w = wpt1;
                landmark.second.ept_w = wpt2;
                d1 = d1_tmp;
                d2 = d2_tmp;
            }

            if (d1 == d2) {
                d1 = d1_tmp;
                d2 = d2_tmp;
                continue;
            }

            if (d1_tmp >= d1) {
                d1 = d1_tmp;
                landmark.second.spt_w = wpt1;
            }

            if (d2_tmp <= d2) {
                d2 = d2_tmp;
                landmark.second.ept_w = wpt2;
            }
        }

    }
}

void optimize_line_without_plucker(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                   Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                   std::vector<std::array<double, 7>> &camera_pose_parameters,
                                   std::array<double, 7> &T_i_c_paramter) {
    double total_time = 0;
    int num_optimaize = 0;

    double total_iteration = 0;

    for (auto &line_landmark : line_landmarks) {

        if (not line_landmark.second.is_triangulation) continue;

        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::CauchyLoss(1.0);

        // record host frame id
        auto landmark_id = line_landmark.first;
        int host_frame_id = line_obs_map.at(landmark_id).begin()->first;
        auto &host_pose_param = camera_pose_parameters.at(host_frame_id);
        // add externsic parameter
        ceres::LocalParameterization *local_parameterization =
                new PoseLocalParameterization();
        problem.AddParameterBlock(T_i_c_paramter.data(), 7,
                                  local_parameterization); // p,q
        problem.SetParameterBlockConstant(T_i_c_paramter.data());

        const auto &obs = line_obs_map.at(landmark_id);
        // loop over camera
        for (const auto &target_frame_obs : obs) {
            int target_frame_id = target_frame_obs.first;
            ceres::LocalParameterization *local_parameterization =
                    new PoseLocalParameterization();
            problem.AddParameterBlock(
                    camera_pose_parameters.at(target_frame_id).data(), 7,
                    local_parameterization); // p,q
            problem.SetParameterBlockConstant(
                    camera_pose_parameters.at(target_frame_id).data());
        }

        // add line parameter
        ceres::LocalParameterization *local_parameterization_line =
                general_line::GeneralLineLocalParameterization::create();
        problem.AddParameterBlock(line_landmark.second.para.data(), 2,
                                  local_parameterization_line); // p,q

        // add factor, notice host and target factor

        for (const auto &target_frame_obs : obs) {
            int target_frame_id = target_frame_obs.first;
            auto &target_pose_param = camera_pose_parameters.at(target_frame_id);
            // one camera factor
            if (host_frame_id == target_frame_id) {


                continue;
//                ceres::CostFunction *factor_host =
//                        general_line::GeneralLineErrorOneCameraTerm::create(
//                                target_frame_obs.second.obs, line_landmark.second.x_dir,
//                                line_landmark.second.R_c_l);
//
//                problem.AddResidualBlock(
//                        factor_host, loss_function, line_landmark.second.para.data(),
//                        target_pose_param.data(), T_i_c_paramter.data());


            } else {
                // two camera factor
                ceres::CostFunction *factor_target =
                        general_line::GeneralLineErrorTwoCameraTerm::create(
                                target_frame_obs.second.obs, line_landmark.second.x_dir,
                                line_landmark.second.R_c_l);

                problem.AddResidualBlock(
                        factor_target, loss_function, line_landmark.second.para.data(),
                        host_pose_param.data(), target_pose_param.data(),
                        T_i_c_paramter.data());
            }
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 6;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        TicToc t_line_optimize;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        total_time += t_line_optimize.toc();
        num_optimaize++;
        total_iteration += summary.iterations.size();

        std::cout << summary.BriefReport() << "\n";

//        std::cout << "optimize time is: " << t_line_optimize.toc() << std::endl;
//        std::cout << "add factor is: " << line_obs_num << std::endl;
//        std::cout << "-------------------------------------------" << std::endl;

    }

    double average_time = total_time / num_optimaize;
    double average_iteration = total_iteration / num_optimaize;

    std::cout << "average optimization of 2 line is: " << average_time * 1e-3 << "s" << std::endl;
    std::cout << "average iteration of 2 line is: " << average_iteration << std::endl;
}


void optimize_line_without_plucker2(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                    Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                    std::vector<std::array<double, 7>> &camera_pose_parameters,
                                    std::array<double, 7> &T_i_c_paramter) {
    double total_time = 0;
    double total_iteration = 0;

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    // add externsic parameter
    ceres::LocalParameterization *local_parameterization =
            new PoseLocalParameterization();
    problem.AddParameterBlock(T_i_c_paramter.data(), 7,
                              local_parameterization); // p,q
    problem.SetParameterBlockConstant(T_i_c_paramter.data());


    for (int i = 0; i < camera_pose_parameters.size(); i++) {

        ceres::LocalParameterization *local_parameterization =
                new PoseLocalParameterization();
        problem.AddParameterBlock(
                camera_pose_parameters.at(i).data(), 7,
                local_parameterization); // p,q
        problem.SetParameterBlockConstant(
                camera_pose_parameters.at(i).data());

    }


    for (auto &line_landmark : line_landmarks) {

        if (not line_landmark.second.is_triangulation) continue;

        // add line parameter
        ceres::LocalParameterization *local_parameterization_line =
                general_line::GeneralLineLocalParameterization::create();
        problem.AddParameterBlock(line_landmark.second.para.data(), 2,
                                  local_parameterization_line); // p,q

        // record host frame id
        auto landmark_id = line_landmark.first;
        int host_frame_id = line_obs_map.at(landmark_id).begin()->first;
        auto &host_pose_param = camera_pose_parameters.at(host_frame_id);

        const auto &obs = line_obs_map.at(landmark_id);

        // add factor, notice host and target factor

        for (const auto &target_frame_obs : obs) {

            int target_frame_id = target_frame_obs.first;
            auto &target_pose_param = camera_pose_parameters.at(target_frame_id);
            // one camera factor
            if (host_frame_id == target_frame_id) {

                continue;

            } else {
                // two camera factor
                ceres::CostFunction *factor_target =
                        general_line::GeneralLineErrorTwoCameraTerm::create(
                                target_frame_obs.second.obs, line_landmark.second.x_dir,
                                line_landmark.second.R_c_l);

                problem.AddResidualBlock(
                        factor_target, loss_function, line_landmark.second.para.data(),
                        host_pose_param.data(), target_pose_param.data(),
                        T_i_c_paramter.data());
            }
        }
    }


    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    TicToc t_line_optimize;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    total_time += t_line_optimize.toc();

    total_iteration += summary.iterations.size();

    std::cout << summary.BriefReport() << "\n";


    std::cout << "average optimization of 2 line is: " << total_time * 1e-3 << "s" << std::endl;
    std::cout << "average iteration of 2 line is: " << total_iteration << std::endl;
}

void optimize_line_with_plucker(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                std::vector<std::array<double, 7>> &camera_pose_parameters,
                                std::array<double, 7> &T_i_c_paramter) {
    double total_time = 0;
    int num_optimaize = 0;
    double total_iteration = 0;

    for (auto &line_landmark : line_landmarks) {


        if (not line_landmark.second.is_triangulation) continue;

        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::CauchyLoss(1.0);
        //    loss_function = nullptr;

        // record host frame id
        auto landmark_id = line_landmark.first;
        int host_frame_id = line_obs_map.at(landmark_id).begin()->first;
        auto &host_pose_param = camera_pose_parameters.at(host_frame_id);
        // add externsic parameter

        ceres::LocalParameterization *local_parameterization =
                new PoseLocalParameterization();
        problem.AddParameterBlock(T_i_c_paramter.data(), 7,
                                  local_parameterization); // p,q
        problem.SetParameterBlockConstant(T_i_c_paramter.data());

        const auto &obs = line_obs_map.at(landmark_id);
        // loop over camera
        for (const auto &target_frame_obs : obs) {

            int target_frame_id = target_frame_obs.first;
            ceres::LocalParameterization *local_parameterization =
                    new PoseLocalParameterization();
            problem.AddParameterBlock(
                    camera_pose_parameters.at(target_frame_id).data(), 7,
                    local_parameterization); // p,q

            problem.SetParameterBlockConstant(
                    camera_pose_parameters.at(target_frame_id).data());
        }

        // add line parameter

//        ceres::LocalParameterization *local_parameterization_line =
//                new LineOrthParameterization();
//        problem.AddParameterBlock(line_landmark.second.line_orth.data(), 4,
//                                  local_parameterization_line); // p,q

        ceres::LocalParameterization *local_parameterization_line =
                plucker::LineOrthParameterization::create();
        problem.AddParameterBlock(line_landmark.second.line_orth_c.data(), 4,
                                  local_parameterization_line); // p,q


        // add factor, notice host and target factor
        for (const auto &target_frame_obs : obs) {
            int target_frame_id = target_frame_obs.first;
            auto &target_pose_param = camera_pose_parameters.at(target_frame_id);
            // one camera factor
            if (host_frame_id == target_frame_id) {

                continue;
            }

            {
                // auto diff for plucker representation
                ceres::CostFunction *f =
                        PluckerLineErrorTwoCameraTerm::create(
                                target_frame_obs.second.obs);

                problem.AddResidualBlock(
                        f, loss_function, line_landmark.second.line_orth_c.data(), host_pose_param.data(),
                        target_pose_param.data(),
                        T_i_c_paramter.data());

//                ceres::CostFunction *f1 = new lineProjectionFactor(target_frame_obs.second.obs);
//                problem.AddResidualBlock(f1, loss_function, target_pose_param.data(), T_i_c_paramter.data(),
//                                         line_landmark.second.line_orth.data());


            }
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 10;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        TicToc t_plucker;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        total_time += t_plucker.toc();
        num_optimaize++;
        total_iteration += summary.iterations.size();

        std::cout << summary.BriefReport() << "\n";
    }

    double average_time = total_time / num_optimaize;
    double average_iteration = total_iteration / num_optimaize;

    std::cout << "average optimization of plucker is: " << average_time * 1e-3 << " s" << std::endl;
    std::cout << "average iteration of plucker is: " << average_iteration << std::endl;

}

void optimize_line_with_plucker2(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks,
                                 Eigen::aligned_map<int, Eigen::aligned_map<int, line_obs>> &line_obs_map,
                                 std::vector<std::array<double, 7>> &camera_pose_parameters,
                                 std::array<double, 7> &T_i_c_paramter) {
    double total_time = 0;
    double total_iteration = 0;

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    ceres::LocalParameterization *local_parameterization =
            new PoseLocalParameterization();
    problem.AddParameterBlock(T_i_c_paramter.data(), 7,
                              local_parameterization); // p,q
    problem.SetParameterBlockConstant(T_i_c_paramter.data());


    for (int i = 0; i < camera_pose_parameters.size(); i++) {

        ceres::LocalParameterization *local_parameterization =
                new PoseLocalParameterization();
        problem.AddParameterBlock(
                camera_pose_parameters.at(i).data(), 7,
                local_parameterization); // p,q
        problem.SetParameterBlockConstant(
                camera_pose_parameters.at(i).data());

    }


    for (auto &line_landmark : line_landmarks) {


        if (not line_landmark.second.is_triangulation) continue;

        // record host frame id
        auto landmark_id = line_landmark.first;
        int host_frame_id = line_obs_map.at(landmark_id).begin()->first;
        auto &host_pose_param = camera_pose_parameters.at(host_frame_id);


        const auto &obs = line_obs_map.at(landmark_id);


        ceres::LocalParameterization *local_parameterization_line =
                plucker::LineOrthParameterization::create();
        problem.AddParameterBlock(line_landmark.second.line_orth_c.data(), 4,
                                  local_parameterization_line); // p,q


        // add factor, notice host and target factor
        for (const auto &target_frame_obs : obs) {
            int target_frame_id = target_frame_obs.first;
            auto &target_pose_param = camera_pose_parameters.at(target_frame_id);
            // one camera factor
            if (host_frame_id == target_frame_id) {

                continue;
            }

            {
                // auto diff for plucker representation
                ceres::CostFunction *f =
                        PluckerLineErrorTwoCameraTerm::create(
                                target_frame_obs.second.obs);

                problem.AddResidualBlock(
                        f, loss_function, line_landmark.second.line_orth_c.data(), host_pose_param.data(),
                        target_pose_param.data(),
                        T_i_c_paramter.data());


            }
        }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    TicToc t_plucker;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    total_time += t_plucker.toc();

    total_iteration += summary.iterations.size();

    std::cout << summary.BriefReport() << "\n";


    double average_time = total_time;
    double average_iteration = total_iteration;

    std::cout << "average optimization of plucker is: " << average_time * 1e-3 << " s" << std::endl;
    std::cout << "average iteration of plucker is: " << average_iteration << std::endl;

}

void savefeatures(Eigen::aligned_unordered_map<int, LineLandmark> &line_landmarks, bool plucker_optimize) {

    double total_err = 0;
    int total_line_num = 0;

    // todo We intersect the line with the plane XY to get the intersection point, the evaluate the accuracy
    Eigen::Vector4d plane(0, 0, 1, 0);

    for (const auto &landmark : line_landmarks) {
        if (!plucker_optimize) {

            if (landmark.second.boptimize_success) {
                std::ofstream foutC("../bin/res/res_ours.txt");
                foutC.setf(std::ios::fixed, std::ios::floatfield);
                foutC.precision(5);
                foutC << landmark.second.spt_w.x() << " "
                      << landmark.second.spt_w.y() << " "
                      << landmark.second.spt_w.z() << " "
                      << landmark.second.ept_w.x() << " "
                      << landmark.second.ept_w.y() << " "
                      << landmark.second.ept_w.z() << " " << std::endl;
                foutC.close();

                total_line_num++;


                Eigen::Vector3d sp_err = landmark.second.spt_w_gt - landmark.second.spt_w;
                Eigen::Vector3d ep_err = landmark.second.ept_w_gt - landmark.second.ept_w;
                total_err = total_err + (sp_err.norm() + ep_err.norm()) / 2;


            }


        } else {

            if (landmark.second.boptimize_success) {

                std::ofstream foutC("../bin/res/res_plucker.txt");
                foutC.setf(std::ios::fixed, std::ios::floatfield);
                foutC.precision(5);
                foutC << landmark.second.spt_w.x() << " "
                      << landmark.second.spt_w.y() << " "
                      << landmark.second.spt_w.z() << " "
                      << landmark.second.ept_w.x() << " "
                      << landmark.second.ept_w.y() << " "
                      << landmark.second.ept_w.z() << " " << std::endl;
                foutC.close();


                total_line_num++;


                Eigen::Vector3d sp_err = landmark.second.spt_w_gt - landmark.second.spt_w;
                Eigen::Vector3d ep_err = landmark.second.ept_w_gt - landmark.second.ept_w;
                total_err = total_err + (sp_err.norm() + ep_err.norm()) / 2;

            }

        }
    }

    double aver_err = total_err / total_line_num;
    std::cout << "average err is: " << aver_err * 100 << " cm" << std::endl;

}
