#ifndef  STEREO_DENSE_H
#define  STEREO_DENSE_H

#include "image_undistort/undistorter.h"
namespace  image_undistort {
    class StereoDense {
    public:
        StereoDense(const CameraParametersPair& left_camera_param_pair,
                    const CameraParametersPair& right_camera_param_pair);

        cv::Mat undistort(const cv::Mat& image);

    private:
        CameraParametersPair left_camera_param_pair_;
        CameraParametersPair right_camera_param_pair_;

        Undistorter left_undistorter_;
        Undistorter right_undistorter_;


        // cache
        cv::Mat left_undistorted_image_;
        cv::Mat right_undistorted_image_;
    };
}
#endif