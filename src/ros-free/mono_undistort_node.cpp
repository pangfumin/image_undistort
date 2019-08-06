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
    std::cout << "intrinsic: " << intrinsics.size() << std::endl;
    if (node["camera_model"].as<std::string>() == "ds") {
        distortion_model = image_undistort::DistortionModel::DOUBLESPHERE;
        D.push_back(intrinsics[0].as<double>());
        D.push_back(intrinsics[1].as<double>());

        K << intrinsics[2].as<double>(), 0, intrinsics[4].as<double>(),
                0, intrinsics[3].as<double>(), intrinsics[5].as<double>(),
                0,0,1;

    } else {
        K << intrinsics[0].as<double>(), 0, intrinsics[2].as<double>(),
                0, intrinsics[1].as<double>(), intrinsics[3].as<double>(),
                0,0,1;
    }

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
    std::string fin = "/media/pang/Elements/dataset/2019-07-30-14-49-34/camchain_ds.yaml";


    const std::string dataset = "/media/pang/Elements/dataset/2019-07-30-14-49-34";
    const std::string image0_folder = dataset + "/front";

    std::string save_folder = dataset;
    auto image0_undistort_folder = save_folder+ "/image0_undistort";

    std::string undistort_parameter_file = save_folder + "/undistort_parameter.txt";
    std::ofstream undistort_parameter_ofs(undistort_parameter_file);

    if (!common::pathExists(image0_undistort_folder)) {
        common::createPath(image0_undistort_folder);
    }




    YAML::Node yamlConfig = YAML::LoadFile(fin);
    YAML::Node node0  = yamlConfig["cam0"];
    std::cout << node0.size() << std::endl;


    image_undistort::CameraParametersPair camera0_param_pair;



   cv::Size resolution0, resolution1;
   Eigen::Matrix<double, 4, 4> T0, T1, T2, T3;
   Eigen::Matrix<double, 3, 3> K0, K1, K2, K3;
   std::vector<double> D0, D1, D2, D3;
    image_undistort::DistortionModel distortion_model0, distortion_model1, distortion_model2, distortion_model3 ;
    loadCameraParamsFromYamlNode(node0, resolution0, T0, K0, D0, distortion_model0);


    camera0_param_pair.setInputCameraParameters(resolution0, T0, K0, D0, distortion_model0);


    std::vector<std::string> image0_filenames;

    /// image0
    common::getAllFilesInFolder(image0_folder, &image0_filenames);
    std::cout<<"image0_list: " << image0_filenames.size() << std::endl;
    std::sort(image0_filenames.begin(),image0_filenames.end(), [](std::string a, std::string b) {
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

    std::shared_ptr<image_undistort::Undistorter> undistorter0
            = std::make_shared<image_undistort::Undistorter>(camera0_param_pair);


    for (int i=0; i < image0_filenames.size(); i++ ) {
        cv::Mat image0 = cv::imread(image0_filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE);


        cv::imshow("image0", image0);
//        cv::imshow("image1", image1);
//        cv::imshow("image2", image2);
//        cv::imshow("image3", image3);

        cv::Mat image0_undistort, image1_undistort, image2_undistort, image3_undistort;
        undistorter0->undistortImage(image0, &image0_undistort);


        cv::imshow("image0_undistort", image0_undistort);
//        cv::imshow("image1_undistort", image1_undistort);
//        cv::imshow("image2_undistort", image2_undistort);
//        cv::imshow("image3_undistort", image3_undistort);

        cv::waitKey(10);


//
        std::string path, file;
        common::splitPathAndFilename(image0_filenames.at(i), &path, &file);
        std::string save_undistort_file0 = image0_undistort_folder + "/" + file;

        cv::imwrite(save_undistort_file0, image0_undistort);

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