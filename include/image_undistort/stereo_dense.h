#ifndef  STEREO_DENSE_H
#define  STEREO_DENSE_H

#include "image_undistort/undistorter.h"
namespace  image_undistort{
    class StereoCameraParameters;
    class StereoDense {
    public:
        StereoDense(const CameraParametersPair& left_camera_param_pair,
                    const CameraParametersPair& right_camera_param_pair,
                    const cv::Size& output_resolution,
                    const Eigen::Matrix3d& output_K);

        cv::Mat undistort(Undistorter* undistorter, const cv::Mat& image);

        void calculateStereoRectify();

        void undistortStereo(const cv::Mat& image0, const cv::Mat& image1,
                cv::Mat& image_undistort0, cv::Mat& image_undistort1);

        void rectifyStereo(const cv::Mat& image_undistort0, const cv::Mat& image_undistort1,
                cv::Mat& image0_rect, cv::Mat& image1_rect);

    private:
        CameraParametersPair left_camera_param_pair_;
        CameraParametersPair right_camera_param_pair_;

        std::shared_ptr<Undistorter> left_undistorter_;
        std::shared_ptr<Undistorter> right_undistorter_;

        std::shared_ptr<StereoCameraParameters> stereo_camera_parameters_ptr_;


        //
        cv::Size output_resolution_;
        Eigen::Matrix3d output_K_;
        cv::Mat R0_, R1_, P0_, P1_, Q_; // stereo rectify result
        cv::Mat M1l_,M2l_,M1r_,M2r_;


        // cache
        cv::Mat left_undistorted_image_;
        cv::Mat right_undistorted_image_;
    };
}
#endif