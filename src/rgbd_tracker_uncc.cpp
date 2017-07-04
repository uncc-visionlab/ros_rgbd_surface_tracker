/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */
#include <opencv2/rgbd.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

namespace cv {
    namespace rgbd {

        void RgbdSurfaceTracker::callback(cv::Mat& _ocv_rgbframe, cv::Mat& _ocv_depthframe_float,
                cv::Mat& _rgb_distortionCoeffs, cv::Mat& _rgb_cameraMatrix) {
            float cx = _rgb_cameraMatrix.at<float>(0, 2);
            float cy = _rgb_cameraMatrix.at<float>(1, 2);
            float fx = _rgb_cameraMatrix.at<float>(0, 0);
            float fy = _rgb_cameraMatrix.at<float>(1, 1);
            cv::rgbd::RgbdImage rgbd_img(_ocv_rgbframe, _ocv_depthframe_float, cx, cy, fx);
            segmentDepth(rgbd_img);
        }

        void RgbdSurfaceTracker::segmentDepth(cv::rgbd::RgbdImage& rgbd_img) {
            const int sobelSize = 3;
            const double sobelScale = 1. / 8.;
            int normalWinSize = 5;
            int normalMethod = RgbdNormals::RGBD_NORMALS_METHOD_FALS;
            cv::Ptr<RgbdNormals> normalsComputer;
            normalsComputer = makePtr<RgbdNormals>(rgbd_img.getDepth().rows,
                    rgbd_img.getDepth().cols,
                    rgbd_img.getDepth().depth(),
                    rgbd_img.getCameraMatrix(),
                    normalWinSize,
                    normalMethod);
            rgbd_img.setNormalsComputer(normalsComputer);
            rgbd_img.computeNormals();
            iterativeAlignment(rgbd_img);
        }
    } /* namespace rgbd */
} /* namespace cv */
