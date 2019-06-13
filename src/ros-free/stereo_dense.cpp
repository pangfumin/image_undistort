#include "image_undistort/stereo_dense.h"

namespace  image_undistort {
    StereoDense::StereoDense(const CameraParametersPair& left_camera_param_pair,
                             const CameraParametersPair& right_camera_param_pair):
            left_undistorter_(left_camera_param_pair),
            right_undistorter_(right_camera_param_pair),
            left_camera_param_pair_(left_camera_param_pair),
            right_camera_param_pair_(right_camera_param_pair){

    }

    cv::Mat StereoDense::undistort(const cv::Mat& image) {
        cv::Mat undistorted;
        left_undistorter_.undistortImage(image, &undistorted);
        return undistorted;
    }

}