/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <ros_rgbd_surface_tracker/rgbd_uncc_contrib>

namespace cv {
    namespace rgbd {

//        void RgbdSurfaceTracker::segmentDepth() {
//            // cv::Mat access flags are cv::ACCESS_READ, cv::ACCESS_WRITE, cv::ACCESS_RW, cv::ACCESS_FAST
//            cv::UMat frame = cv_rgbimg_ptr->image.getUMat(cv::ACCESS_READ);
//            cv::UMat depth_frame;
//            cv::UMat frame_vis = frame.clone(); // refresh visualization frame
//            cv::Ptr<cv::UMat> descriptors_frame(new cv::UMat);
//            cv::UMat mask;
//
//
//            cv::Mat img_I = frame.getMat(cv::ACCESS_READ);
//            cv::Mat img_Z = depth_frame.getMat(cv::ACCESS_READ);
//            cv::Mat img_L = cv::Mat::zeros(img_I.rows, img_I.cols, CV_8U);
//
//            //            float cx = model_.cx();
//            //            float cy = model_.cy();
//            //            float fx = model_.fx();
//            //            int BLOCKSIZE = 50;
//            //            cv::RGBDImageProcessor rgbd_proc(img_I, img_Z, cx, cy, fx);
//            //
//            //            int tile_width = 100, tile_height = 100;
//            //            int x0 = floor(0.5 * (rgbd_proc.width - tile_width));
//            //            int y0 = floor(0.5 * (rgbd_proc.height - tile_height));
//            //            int num_samples = 1000;
//            //
//            //            std::vector<cv::Point3f> data;
//            //            data.reserve(num_samples);
//            //
//            //            bool got_all_samples = rgbd_proc.getTileData_Uniform(
//            //                    x0, y0, tile_width, tile_height, data, num_samples);
//        }
    } /* namespace rgbd */
} /* namespace cv */
