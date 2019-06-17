#ifndef  STEREO_DENSE_H
#define  STEREO_DENSE_H

#include "image_undistort/undistorter.h"

constexpr int kPreFilterCap = 31;
constexpr int kSADWindowSize = 11;
constexpr int kMinDisparity = 0;
constexpr int kNumDisparities = 64;
constexpr int kUniquenessRatio = 0;
constexpr int kSpeckleRange = 3;
constexpr int kSpeckleWindowSize = 500;

constexpr int kP1 = 120;
constexpr int kP2 = 240;
constexpr int kDisp12MaxDiff = -1;

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

        static void fillDisparityFromSide(const cv::Mat& input_disparity,
                                                const cv::Mat& valid, const bool& from_left,
                                                cv::Mat* filled_disparity);
        void bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                                    cv::Mat* disparity_filled,
                                                    cv::Mat* input_valid) const;

        void calcDisparityImage(
                const cv::Mat& left_image, const cv::Mat& right_image,
                cv::Mat& disparity) const;

        cv::Mat getP0() const {return P0_; }
        cv::Mat getP1() const {return P1_; }

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

        // depth parameter
        int pre_filter_cap_;
        int sad_window_size_;
        int min_disparity_;
        int num_disparities_;
        int uniqueness_ratio_;
        int speckle_range_;
        int speckle_window_size_;

        int p1_;
        int p2_;
        int disp_12_max_diff_;

        // cache
        cv::Mat left_undistorted_image_;
        cv::Mat right_undistorted_image_;
    };
}
#endif