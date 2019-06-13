#include <iostream>
#include "image_undistort/stereo_dense.h"

#include <yaml-cpp/yaml.h>

void loadCameraParamsFromYamlNode(const YAML::Node& node, cv::Size& res,
                                  Eigen::Matrix<double, 4, 4>& T,
                                  Eigen::Matrix<double, 3, 3>& K,
                                  std::vector<double>& D,
                                  image_undistort::DistortionModel& distortion_model) {
    const std::string distortion_model_string = node["distortion_model"].as<std::string>();
    if (distortion_model_string == "fov") {
        distortion_model = image_undistort::DistortionModel::FOV;
    }
    YAML::Node d = node["distortion_coeffs"];
    D.push_back(d[0].as<double>());
    YAML::Node resolution = node["resolution"];
    res = cv::Size(resolution[0].as<int>(), resolution[1].as<int>());

    YAML::Node intrinsics = node["intrinsics"];
    K << intrinsics[0].as<double>(), 0, intrinsics[2].as<double>(),
            0, intrinsics[0].as<double>(1), intrinsics[3].as<double>(),
                    0,0,1;
    if (node["T_cn_cnm1"].IsDefined()) {
        for (int i = 0; i < 4; i++ ) {
            for (int j = 0; j < 4; j++) {
                T(i,j) = node["T_cn_cnm1"][i][j].as<double>();
            }
        }

    } else {
        T.setIdentity();
    }

}
int main (int argc, char** argv) {
    std::string fin = "/home/pang/data/dataset/segway_outdoor/cui_stereo_calib/camchain-cam_stereo.yaml";
    YAML::Node yamlConfig = YAML::LoadFile(fin);
    YAML::Node node0  = yamlConfig["cam0"];
    YAML::Node node1  = yamlConfig["cam1"];

    image_undistort::CameraParametersPair left_camera_param_pair;
    image_undistort::CameraParametersPair right_camera_param_pair;


   cv::Size resolution0, resolution1;
   Eigen::Matrix<double, 4, 4> T0, T1;
   Eigen::Matrix<double, 3, 3> K0, K1;
   std::vector<double> D0, D1;
    image_undistort::DistortionModel distortion_model0, distortion_model1;
    loadCameraParamsFromYamlNode(node0, resolution0, T0, K0, D0, distortion_model0);
    loadCameraParamsFromYamlNode(node1, resolution1, T1, K1, D1, distortion_model1);



    return 0;
}