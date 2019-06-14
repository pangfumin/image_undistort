#include <iostream>
#include "image_undistort/stereo_dense.h"
#include "file-system-tools.h"

#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    std::cout << T0 << std::endl;
    std::cout << T1 << std::endl;
    left_camera_param_pair.setInputCameraParameters(resolution0, T0, K0, D0, distortion_model0);
    right_camera_param_pair.setInputCameraParameters(resolution1, T1, K1, D1, distortion_model1);




    const std::string image0_path = "/home/pang/data/dataset/segway_outdoor/cui_stereo_calib/newCamPics/Camera_1R_recorder";
    const std::string image1_path = "/home/pang/data/dataset/segway_outdoor/cui_stereo_calib/newCamPics/Camera_1R_recorder";

    std::vector<std::string> left_image_filenames;
    std::vector<std::string> right_image_filenames;
    /// image0
    common::getAllFilesInFolder(image0_path, &left_image_filenames);
    std::cout<<"image0_list: " << left_image_filenames.size() << std::endl;

    std::sort(left_image_filenames.begin(),left_image_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    /// image1
    common::getAllFilesInFolder(image1_path, &right_image_filenames);
    std::cout<<"image1_list: " << right_image_filenames.size() << std::endl;
    std::sort(right_image_filenames.begin(),right_image_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });


    cv::Size output_resolution  = left_camera_param_pair.getInputPtr()->resolution()/2;
    Eigen::Matrix3d output_K;
    output_K << K0(0,0)/2, 0, output_resolution.width/2,
            0, K0(1,1)/2,  output_resolution.height/2,
            0,0,1;

    image_undistort::StereoDense stereoDense(left_camera_param_pair, right_camera_param_pair,
                                             output_resolution, output_K);


    for (int i=0; i < right_image_filenames.size(); i++ ) {
        std::cout << right_image_filenames.at(i) << std::endl;
        cv::Mat left_image = cv::imread(left_image_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat right_image = cv::imread(right_image_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);


        cv::Mat left_undistorted ;
        cv::Mat right_undistorted;

        cv::Mat left_rect ;
        cv::Mat right_rect;

        stereoDense.undistortStereo(left_image, right_image, left_undistorted, right_undistorted);
        stereoDense.rectifyStereo(left_undistorted, right_undistorted, left_rect, right_rect);


        cv::Mat merge = 0.5 * left_rect + 0.5 *right_rect;

//        cv::imshow("left", left_image);
        cv::imshow("left_undistorted", left_undistorted);
        cv::imshow("left_rect", left_rect);
//        cv::imshow("right", right_image);
        cv::imshow("right_undistorted", right_undistorted);
        cv::imshow("right_rect", right_rect);
        cv::imshow("merge", merge);
        cv::waitKey(30000);
    }




    return 0;
}