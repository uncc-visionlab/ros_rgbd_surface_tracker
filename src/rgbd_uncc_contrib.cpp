/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */

#include <ros_rgbd_surface_tracker/rgbd_uncc_contrib.hpp>

namespace cv {
    namespace rgbd {

        void RgbdSurfaceTracker::segmentDepth() {
            std::cout << "segmentDepth()" << std::endl;
            cv::Mat img_I = rawRGB.getMat(cv::ACCESS_READ);
            cv::Mat img_Z = rawDepth.getMat(cv::ACCESS_READ);
            cv::Mat img_L = cv::Mat::zeros(img_I.rows, img_I.cols, CV_8U);

            float cx = cameraMatrix.at<float>(0, 2);
            float cy = cameraMatrix.at<float>(1, 2);
            float fx = cameraMatrix.at<float>(0, 0);
            float fy = cameraMatrix.at<float>(1, 1);
            //cv::RGBDImageProcessor rgbd_proc(img_I, img_Z, cx, cy, fx);

            //int tile_width = 100, tile_height = 100;
            //int x0 = floor(0.5 * (rgbd_proc.width - tile_width));
            //int y0 = floor(0.5 * (rgbd_proc.height - tile_height));
            //int num_samples = 1000;

            //std::vector<cv::Point3f> data;
            //data.reserve(num_samples);

            //bool got_all_samples = rgbd_proc.getTileData_Uniform(
            //        x0, y0, tile_width, tile_height, data, num_samples);
        }
    } /* namespace rgbd */
} /* namespace cv */
