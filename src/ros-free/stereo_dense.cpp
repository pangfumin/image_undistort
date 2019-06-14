#include "image_undistort/stereo_dense.h"
#include "image_undistort/camera_parameters.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace  image_undistort {
    StereoDense::StereoDense(const CameraParametersPair& left_camera_param_pair,
                             const CameraParametersPair& right_camera_param_pair,
                             const cv::Size& output_resolution,
                             const Eigen::Matrix3d& output_K):
            left_camera_param_pair_(left_camera_param_pair),
            right_camera_param_pair_(right_camera_param_pair),
            output_resolution_(output_resolution),
            output_K_(output_K){


        auto T0 = left_camera_param_pair_.getInputPtr()->T();
        left_camera_param_pair_.setOutputCameraParameters(output_resolution_, T0, output_K_);
        right_camera_param_pair_.setOutputCameraParameters(output_resolution_, T0, output_K_);

        left_undistorter_ = std::make_shared<Undistorter>(left_camera_param_pair_);
        right_undistorter_ = std::make_shared<Undistorter>(right_camera_param_pair_);
        stereo_camera_parameters_ptr_ = std::make_shared<StereoCameraParameters>();


//        stereo_camera_parameters_ptr_->setInputCameraParameters(
//                left_camera_param_pair_.getInputPtr()->resolution(),
//                left_camera_param_pair_.getInputPtr()->T(),
//                left_camera_param_pair_.getInputPtr()->K(),
//                left_camera_param_pair_.getInputPtr()->D(),
//                left_camera_param_pair_.getInputPtr()->distortionModel(),
//                CameraSide::FIRST);
//        std::cout << "set first" << std::endl;
//
//        stereo_camera_parameters_ptr_->setInputCameraParameters(
//                right_camera_param_pair_.getInputPtr()->resolution(),
//                right_camera_param_pair_.getInputPtr()->T(),
//                right_camera_param_pair_.getInputPtr()->K(),
//                right_camera_param_pair_.getInputPtr()->D(),
//                right_camera_param_pair_.getInputPtr()->distortionModel(),
//                CameraSide::SECOND);
//        std::cout << "set second" << std::endl;
//
//
//        std::cout << left_camera_param_pair_.getInputPtr()->T().transpose() << std::endl;
//        std::cout << right_camera_param_pair_.getInputPtr()->T().transpose() << std::endl;

        calculateStereoRectify();

        std::cout << P0_ << std::endl;
        std::cout << P1_ << std::endl;
        std::cout << R0_ << std::endl;
        std::cout << R1_ << std::endl;



    }



    cv::Mat StereoDense::undistort(Undistorter* undistorter, const cv::Mat& image) {
        cv::Mat undistorted;
        undistorter->undistortImage(image, &undistorted);
        return undistorted;
    }

    void StereoDense::calculateStereoRectify() {

        cv::Mat K0, K1;
        cv::eigen2cv(output_K_, K0);
        cv::eigen2cv(output_K_, K1);

        cv::Mat R0, R1, P0, P1, Q;
        cv::Mat R, t;
        Eigen::Matrix3d R_cam1_cam0 = right_camera_param_pair_.getInputPtr()->T().topLeftCorner(3,3);
        Eigen::Vector3d t_cam1_cam0 = right_camera_param_pair_.getInputPtr()->T().topRightCorner(3,1);
        cv::eigen2cv(R_cam1_cam0, R);
        cv::eigen2cv(t_cam1_cam0, t);

        cv::stereoRectify(K0, cv::Mat(), K1, cv::Mat(), output_resolution_, R, t, R0_, R1_, P0_, P1_, Q_);

        cv::initUndistortRectifyMap(K0,cv::Mat(),R0_,P0_,output_resolution_,CV_32F,M1l_,M2l_);
        cv::initUndistortRectifyMap(K1,cv::Mat(),R1_,P1_,output_resolution_,CV_32F,M1r_,M2r_);

    }

    void StereoDense::undistortStereo(const cv::Mat& image0, const cv::Mat& image1,
                         cv::Mat& image_undistort0, cv::Mat& image_undistort1) {

        image_undistort0 = undistort(left_undistorter_.get(), image0);
        image_undistort1 = undistort(right_undistorter_.get(), image1);
    }



}