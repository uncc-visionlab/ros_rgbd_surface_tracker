/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */
#include <opencv2/rgbd.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

extern int supermain(int, char **, PlaneVisualizationData& vis_data);

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

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif

            rgbd_img.computeNormals();
            //iterativeAlignment(rgbd_img);

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      
            PlaneVisualizationData* vis_data = this->getPlaneVisualizationData();
            supermain(0, nullptr, *vis_data);
            std::cout << "num tris from marching cubes: " << vis_data->triangles.size() << std::endl;
            
        }
    } /* namespace rgbd */
} /* namespace cv */
