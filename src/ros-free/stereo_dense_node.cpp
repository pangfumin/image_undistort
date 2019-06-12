#include <iostream>
#include "image_undistort/stereo_dense.h"

#include <yaml-cpp/yaml.h>

//const cv::Size& resolution,
//const Eigen::Matrix<double, 4, 4>& T,
//const Eigen::Matrix<double, 3, 3>& K,
//const std::vector<double>& D,
//const DistortionModel& distortion_model

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



//    for (std::size_t i=0;i<primes.size();i++) {
//        std::cout << primes[i].as<float>() << "\n";
//    }

}
int main (int argc, char** argv) {
    std::string fin = "/media/pang/Plus/segway_outdoor/cui_stereo_calib/camchain-cam_stereo.yaml";       //yaml文件所在的路径
    YAML::Node yamlConfig = YAML::LoadFile(fin);


    const std::string username = yamlConfig["cam0"]["camera_model"].as<std::string>();
    std::cout << "username: " << username << std::endl;
    YAML::Node node  = yamlConfig["cam0"];
    std::cout << node["distortion_coeffs"].Type() << std::endl;
//    std::vector<float> = node["distortion_coeffs"].as<std::vector<float>>() ;

    image_undistort::CameraParametersPair left_camera_param_pair;
    image_undistort::CameraParametersPair right_camera_param_pair;
//    left_camera_param_pair.setInputCameraParameters()


   cv::Size resolution;
   Eigen::Matrix<double, 4, 4> T;
   Eigen::Matrix<double, 3, 3> K;
   std::vector<double> D;
    image_undistort::DistortionModel distortion_model;
    loadCameraParamsFromYamlNode(node, resolution, T, K, D, distortion_model);



    return 0;
}