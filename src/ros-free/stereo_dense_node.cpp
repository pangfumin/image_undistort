#include <iostream>
#include <fstream>
#include "image_undistort/stereo_dense.h"
#include "file-system-tools.h"

#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

void loadCameraParamsFromYamlNode(const YAML::Node& node, cv::Size& res,
                                  Eigen::Matrix<double, 4, 4>& T,
                                  Eigen::Matrix<double, 3, 3>& K,
                                  std::vector<double>& D,
                                  image_undistort::DistortionModel& distortion_model) {
    YAML::Node d = node["distortion_coeffs"];
    const std::string distortion_model_string = node["distortion_model"].as<std::string>();
    if (distortion_model_string == "fov") {
        distortion_model = image_undistort::DistortionModel::FOV;
        D.push_back(d[0].as<double>());
    } else if (distortion_model_string == "radtan"){
        distortion_model = image_undistort::DistortionModel::RADTAN;
        D.push_back(d[0].as<double>());
        D.push_back(d[1].as<double>());
        D.push_back(d[2].as<double>());
        D.push_back(d[3].as<double>());
    }

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

cv::Mat drawStereoRectify(const cv::Mat left, const cv::Mat right) {
    std::vector<cv::Mat> channels;
    cv::Mat outimg(left.rows, 2*left.cols, CV_8UC3);
    auto colJump = left.cols;
    cv::Mat left_rgb = outimg(cv::Rect(0, 0, left.cols, left.rows));
    cv::Mat right_rgb = outimg(cv::Rect( colJump, 0, left.cols, left.rows));

    cv::cvtColor(left, left_rgb, CV_GRAY2BGR);
    cv::cvtColor(right, right_rgb, CV_GRAY2BGR);

    int num_line = 10;
    int row_interval =  left.rows /num_line;
    for (int i=0; i < num_line; i++) {
        cv::Point start(0, i * row_interval );
        cv::Point end(2*left.cols, i * row_interval );
        cv::line(outimg, start, end, cv::Scalar(255,0,0), 2);
    }

    return outimg;

}
int main (int argc, char** argv) {
    std::string fin = "/home/pang/data/dataset/segway_outdoor/cui_stereo_calib/camchain.yaml";

    std::string rectify_out_file = "/home/pang/data/dataset/segway_outdoor/cui_stereo_calib/rectified_camera_parameters.txt";

    const std::string image0_path = "/home/pang/data/dataset/segway_outdoor/2019-06-05-16-45-28_forMap/image0_syn";
    const std::string image1_path = "/home/pang/data/dataset/segway_outdoor/2019-06-05-16-45-28_forMap/image1_syn";

    std::string save_folder = "/home/pang/data/dataset/segway_outdoor/2019-06-05-16-45-28_forMap";
    auto image0_rect_folder = save_folder+ "/image0_syn_rect";
    auto image1_rect_folder = save_folder+ "/image1_syn_rect";

    std::ofstream rectify_out_ofs(rectify_out_file);

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

    if (!common::pathExists(image0_rect_folder)) {
        common::createPath(image0_rect_folder);
    }
    if (!common::pathExists(image1_rect_folder)) {
        common::createPath(image1_rect_folder);
    }


    cv::Size output_resolution  = left_camera_param_pair.getInputPtr()->resolution();
    Eigen::Matrix3d output_K;
    output_K << K0(0,0), 0, output_resolution.width/2,
            0, K0(1,1),  output_resolution.height/2,
            0,0,1;

    image_undistort::StereoDense stereoDense(left_camera_param_pair, right_camera_param_pair,
                                             output_resolution, output_K);


    cv::Mat P0 = stereoDense.getP0();
    cv::Mat P1 = stereoDense.getP1();
    Eigen::Matrix<double ,3,4> eigenP0, eigenP1;
    cv::cv2eigen(P0, eigenP0);
    cv::cv2eigen(P1, eigenP1);
    rectify_out_ofs << output_resolution.height << " " << output_resolution.width << std::endl;
    rectify_out_ofs << eigenP0 << std::endl;
    rectify_out_ofs << eigenP1 << std::endl;

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

//        cv::Mat disparity, disparity8;
//        stereoDense.calcDisparityImage(left_rect, right_rect, disparity);
//        disparity.convertTo(disparity, CV_32F, 1.0/16);
//        disparity.convertTo(disparity8, CV_8UC1);

        std::string path, file;
        common::splitPathAndFilename(left_image_filenames.at(i), &path, &file);
        std::string left_save_rect_file = image0_rect_folder + "/" + file;
        std::string right_save_rect_file = image1_rect_folder + "/" + file;

//
        cv::imwrite(left_save_rect_file, left_rect);
        cv::imwrite(right_save_rect_file, right_rect);


        cv::Mat merge = drawStereoRectify(left_rect, right_rect);
////        cv::imshow("left", left_image);
//        cv::imshow("left_undistorted", left_undistorted);
//        cv::imshow("left_rect", left_rect);
////        cv::imshow("right", right_image);
//        cv::imshow("right_undistorted", right_undistorted);
//        cv::imshow("right_rect", right_rect);


        cv::Size resize(merge.cols/2, merge.rows/2);
        cv::Mat dst;
        cv::resize(merge, dst, resize);
        cv::imshow("merge", dst);
//        cv::imshow("disparity8", disparity8);
        cv::waitKey(10);
    }




    return 0;
}