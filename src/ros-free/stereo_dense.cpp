#include "image_undistort/ros-free/stereo_dense.h"
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

        calculateStereoRectify();


        pre_filter_cap_ = kPreFilterCap;
        sad_window_size_ = kSADWindowSize;
        min_disparity_ = kMinDisparity;
        num_disparities_ = kNumDisparities;
        uniqueness_ratio_ = kUniquenessRatio;
        speckle_range_ = kSpeckleRange;
        speckle_window_size_ = kSpeckleWindowSize;

        p1_ = kP1;
        p2_ = kP2;
        disp_12_max_diff_ = kDisp12MaxDiff;

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
        Eigen::Matrix4d T_cam1_cam0 = right_camera_param_pair_.getInputPtr()->T();
        Eigen::Matrix3d R_cam1_cam0 = T_cam1_cam0.topLeftCorner(3,3);
        Eigen::Vector3d t_cam1_cam0 = T_cam1_cam0.topRightCorner(3,1);

        cv::eigen2cv(R_cam1_cam0, R);
        cv::eigen2cv(t_cam1_cam0, t);

        cv::stereoRectify(K0, cv::Mat(), K1, cv::Mat(), output_resolution_, R, t, R0_, R1_, P0_, P1_, Q_);

        cv::initUndistortRectifyMap(K0,cv::Mat(),R0_,P0_,output_resolution_,CV_32F,M1l_,M2l_);
        cv::initUndistortRectifyMap(K1,cv::Mat(),R1_,P1_,output_resolution_,CV_32F,M1r_,M2r_);

    }

    void StereoDense::rectifyStereo(const cv::Mat& image_undistort0, const cv::Mat& image_undistort1,
                       cv::Mat& image0_rect, cv::Mat& image1_rect) {
        cv::remap(image_undistort0,image0_rect,M1l_,M2l_,cv::INTER_LINEAR);
        cv::remap(image_undistort1,image1_rect,M1r_,M2r_,cv::INTER_LINEAR);
    }


    void StereoDense::undistortStereo(const cv::Mat& image0, const cv::Mat& image1,
                         cv::Mat& image_undistort0, cv::Mat& image_undistort1) {
        image_undistort0 = undistort(left_undistorter_.get(), image0);
        image_undistort1 = undistort(right_undistorter_.get(), image1);
    }


// simply replaces invalid disparity values with a valid value found by scanning
// horizontally (note: if disparity values are already valid or if no valid
// value can be found int_max is inserted)
    void StereoDense::fillDisparityFromSide(const cv::Mat& input_disparity,
                                      const cv::Mat& valid, const bool& from_left,
                                      cv::Mat* filled_disparity) {
        *filled_disparity =
                cv::Mat(input_disparity.rows, input_disparity.cols, CV_16S);

        for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
            bool prev_valid = false;
            int16_t prev_value;

            for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
                size_t x_scan;
                if (from_left) {
                    x_scan = x_pixels;
                } else {
                    x_scan = (input_disparity.cols - x_pixels - 1);
                }

                if (valid.at<uint8_t>(y_pixels, x_scan)) {
                    prev_valid = true;
                    prev_value = input_disparity.at<int16_t>(y_pixels, x_scan);
                    filled_disparity->at<int16_t>(y_pixels, x_scan) =
                            std::numeric_limits<int16_t>::max();
                } else if (prev_valid) {
                    filled_disparity->at<int16_t>(y_pixels, x_scan) = prev_value;
                } else {
                    filled_disparity->at<int16_t>(y_pixels, x_scan) =
                            std::numeric_limits<int16_t>::max();
                }
            }
        }
    }

    void StereoDense::bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                          cv::Mat* disparity_filled,
                                          cv::Mat* input_valid) const {
        // mark valid pixels
        *input_valid = cv::Mat(input_disparity.rows, input_disparity.cols, CV_8U);

        int side_bound = sad_window_size_ / 2;

        for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
            for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
                // the last check is because the sky has a bad habit of having a disparity
                // at just less than the max disparity
                if ((x_pixels < side_bound + min_disparity_ + num_disparities_) ||
                    (y_pixels < side_bound) ||
                    (x_pixels > (input_disparity.cols - side_bound)) ||
                    (y_pixels > (input_disparity.rows - side_bound)) ||
                    (input_disparity.at<int16_t>(y_pixels, x_pixels) < 0) ||
                    (input_disparity.at<int16_t>(y_pixels, x_pixels) >=
                     (min_disparity_ + num_disparities_ - 1) * 16)) {
                    input_valid->at<uint8_t>(y_pixels, x_pixels) = 0;
                } else {
                    input_valid->at<uint8_t>(y_pixels, x_pixels) = 1;
                }
            }
        }

        // erode by size of SAD window, this prevents issues with background pixels
        // being given the same depth as neighboring objects in the foreground.
        cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(sad_window_size_, sad_window_size_));
        cv::erode(*input_valid, *input_valid, kernel);

        // take a guess for the depth of the invalid pixels by scanning along the row
        // and giving them the same value as the closest horizontal point.
        cv::Mat disparity_filled_left, disparity_filled_right;
        fillDisparityFromSide(input_disparity, *input_valid, true,
                              &disparity_filled_left);
        fillDisparityFromSide(input_disparity, *input_valid, false,
                              &disparity_filled_right);

        // take the most conservative disparity of the two
        *disparity_filled = cv::max(disparity_filled_left, disparity_filled_right);

        // 0 disparity is valid but cannot have a depth associated with it, because of
        // this we take these points and replace them with a disparity of 1.
        for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
            for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
                if (input_disparity.at<int16_t>(y_pixels, x_pixels) == 0) {
                    disparity_filled->at<int16_t>(y_pixels, x_pixels) = 1;
                }
            }
        }
    }


    void StereoDense::calcDisparityImage(
            const cv::Mat& left_image,
            const cv::Mat& right_image,
           cv::Mat& disparity) const {

        auto mode = cv::StereoSGBM::MODE_SGBM;


        cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
                min_disparity_, num_disparities_, sad_window_size_, p1_, p2_,
                disp_12_max_diff_, pre_filter_cap_, uniqueness_ratio_,
                speckle_window_size_, speckle_range_, mode);

        left_matcher->compute(left_image,right_image,
                              disparity);

    }

}