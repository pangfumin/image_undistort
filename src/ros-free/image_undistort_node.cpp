#include <iostream>
#include <fstream>
#include "image_undistort/ros-free/stereo_dense.h"
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
            0, intrinsics[1].as<double>(), intrinsics[3].as<double>(),
                    0,0,1;
    std::cout << "K:\n" << K << std::endl;
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
    std::string fin = "/home/pang/data/dataset/segway_outdoor/2019-04-26-12-56-23/2019-04-26-12-56-23.yaml";


    const std::string dataset = "/home/pang/data/dataset/segway_outdoor/2019-04-26-12-56-23";
    const std::string image0_folder = dataset + "/image_0";
    const std::string image1_folder = dataset + "/image_1";
    const std::string image2_folder = dataset + "/image_2";
    const std::string image3_folder = dataset + "/image_3";

    std::string save_folder = dataset;
    auto image0_undistort_folder = save_folder+ "/image0_undistort";
    auto image1_undistort_folder = save_folder+ "/image1_undistort";
    auto image2_undistort_folder = save_folder+ "/image2_undistort";
    auto image3_undistort_folder = save_folder+ "/image3_undistort";

    std::string undistort_parameter_file = save_folder + "/undistort_parameter.txt";
    std::ofstream undistort_parameter_ofs(undistort_parameter_file);

    if (!common::pathExists(image0_undistort_folder)) {
        common::createPath(image0_undistort_folder);
    }

    if (!common::pathExists(image1_undistort_folder)) {
        common::createPath(image1_undistort_folder);
    }

    if (!common::pathExists(image2_undistort_folder)) {
        common::createPath(image2_undistort_folder);
    }

    if (!common::pathExists(image3_undistort_folder)) {
        common::createPath(image3_undistort_folder);
    }


    YAML::Node yamlConfig = YAML::LoadFile(fin);
    YAML::Node node0  = yamlConfig["cam0"];
    YAML::Node node1  = yamlConfig["cam1"];
    YAML::Node node2  = yamlConfig["cam2"];
    YAML::Node node3  = yamlConfig["cam3"];

    image_undistort::CameraParametersPair camera0_param_pair;
    image_undistort::CameraParametersPair camera1_param_pair;
    image_undistort::CameraParametersPair camera2_param_pair;
    image_undistort::CameraParametersPair camera3_param_pair;


   cv::Size resolution0, resolution1;
   Eigen::Matrix<double, 4, 4> T0, T1, T2, T3;
   Eigen::Matrix<double, 3, 3> K0, K1, K2, K3;
   std::vector<double> D0, D1, D2, D3;
    image_undistort::DistortionModel distortion_model0, distortion_model1, distortion_model2, distortion_model3 ;
    loadCameraParamsFromYamlNode(node0, resolution0, T0, K0, D0, distortion_model0);
    loadCameraParamsFromYamlNode(node1, resolution0, T1, K1, D1, distortion_model1);
    loadCameraParamsFromYamlNode(node2, resolution0, T2, K2, D2, distortion_model2);
    loadCameraParamsFromYamlNode(node3, resolution0, T3, K3, D3, distortion_model3);

    camera0_param_pair.setInputCameraParameters(resolution0, T0, K0, D0, distortion_model0);
    camera1_param_pair.setInputCameraParameters(resolution1, T1, K1, D1, distortion_model1);
    camera2_param_pair.setInputCameraParameters(resolution0, T2, K2, D2, distortion_model2);
    camera3_param_pair.setInputCameraParameters(resolution1, T3, K3, D3, distortion_model3);

    std::vector<std::string> image0_filenames;
    std::vector<std::string> image1_filenames;
    std::vector<std::string> image2_filenames;
    std::vector<std::string> image3_filenames;
    std::vector<std::string> lidar_filenames;
    /// image0
    common::getAllFilesInFolder(image0_folder, &image0_filenames);
    std::cout<<"image0_list: " << image0_filenames.size() << std::endl;
    std::sort(image0_filenames.begin(),image0_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    /// image1
    common::getAllFilesInFolder(image1_folder, &image1_filenames);
    std::cout<<"image1_list: " << image1_filenames.size() << std::endl;
    std::sort(image1_filenames.begin(),image1_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });


    /// image2
    common::getAllFilesInFolder(image2_folder, &image2_filenames);
    std::cout<<"image2_list: " << image2_filenames.size() << std::endl;
    std::sort(image2_filenames.begin(),image2_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    /// image3
    common::getAllFilesInFolder(image3_folder, &image3_filenames);
    std::cout<<"image3_list: " << image3_filenames.size() << std::endl;
    std::sort(image3_filenames.begin(),image3_filenames.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });




    cv::Size output_resolution  = camera0_param_pair.getInputPtr()->resolution();
    Eigen::Matrix3d output_K;
    output_K << K0(0,0), 0, output_resolution.width/2,
            0, K0(1,1),  output_resolution.height/2,
            0,0,1;

    undistort_parameter_ofs << K0(0,0) << " " << K0(1,1)
                        << " " <<  output_resolution.width/2 << " " << output_resolution.height/2 << std::endl;

    std::cout  << K0(0,0) << " " << K0(1,1)
                            << " " <<  output_resolution.width/2 << " " << output_resolution.height/2 << std::endl;


    camera0_param_pair.setOutputCameraParameters(output_resolution,camera0_param_pair.getInputPtr()->T(),output_K);
    camera1_param_pair.setOutputCameraParameters(output_resolution,camera0_param_pair.getInputPtr()->T(),output_K);
    camera2_param_pair.setOutputCameraParameters(output_resolution,camera0_param_pair.getInputPtr()->T(),output_K);
    camera3_param_pair.setOutputCameraParameters(output_resolution,camera0_param_pair.getInputPtr()->T(),output_K);

    std::shared_ptr<image_undistort::Undistorter> undistorter0
            = std::make_shared<image_undistort::Undistorter>(camera0_param_pair);
    std::shared_ptr<image_undistort::Undistorter> undistorter1
            = std::make_shared<image_undistort::Undistorter>(camera1_param_pair);
    std::shared_ptr<image_undistort::Undistorter> undistorter2
            = std::make_shared<image_undistort::Undistorter>(camera2_param_pair);
    std::shared_ptr<image_undistort::Undistorter> undistorter3
            = std::make_shared<image_undistort::Undistorter>(camera3_param_pair);

    for (int i=0; i < image0_filenames.size(); i++ ) {
        cv::Mat image0 = cv::imread(image0_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat image1 = cv::imread(image1_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat image2 = cv::imread(image2_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat image3 = cv::imread(image3_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);

//        cv::imshow("image0", image0);
//        cv::imshow("image1", image1);
//        cv::imshow("image2", image2);
//        cv::imshow("image3", image3);

        cv::Mat image0_undistort, image1_undistort, image2_undistort, image3_undistort;
        undistorter0->undistortImage(image0, &image0_undistort);
        undistorter0->undistortImage(image1, &image1_undistort);
        undistorter0->undistortImage(image2, &image2_undistort);
        undistorter0->undistortImage(image3, &image3_undistort);

//        cv::imshow("image0_undistort", image0_undistort);
//        cv::imshow("image1_undistort", image1_undistort);
//        cv::imshow("image2_undistort", image2_undistort);
//        cv::imshow("image3_undistort", image3_undistort);

        cv::waitKey(1);



        std::string path, file;
        common::splitPathAndFilename(image0_filenames.at(i), &path, &file);
        std::string save_undistort_file0 = image0_undistort_folder + "/" + file;
        std::string save_undistort_file1 = image1_undistort_folder + "/" + file;
        std::string save_undistort_file2 = image2_undistort_folder + "/" + file;
        std::string save_undistort_file3 = image3_undistort_folder + "/" + file;

////
        cv::imwrite(save_undistort_file0, image0_undistort);
        cv::imwrite(save_undistort_file1, image1_undistort);
        cv::imwrite(save_undistort_file2, image2_undistort);
        cv::imwrite(save_undistort_file3, image3_undistort);


//
        std::cout << i << "/" << image0_filenames.size() << std::endl;
//        cv::Size resize(merge.cols/2, merge.rows/2);
//        cv::Mat dst;
//        cv::resize(merge, dst, resize);
//        cv::imshow("merge", dst);
////        cv::imshow("disparity8", disparity8);
//        cv::waitKey(10);
    }




    return 0;
}