/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rgbd_uncc_contrib.h
 * Author: arwillis
 *
 * Created on June 30, 2017, 10:01 AM
 */

#ifndef RGBD_UNCC_CONTRIB_HPP
#define RGBD_UNCC_CONTRIB_HPP

#ifdef __cplusplus

#include <limits>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <boost/shared_ptr.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>

#define PROFILE_CALLGRIND false

#ifdef PROFILE_CALLGRIND
#include <valgrind/callgrind.h>
#endif

namespace cv {
    namespace rgbd {

        class RgbdSurfaceTracker {
        public:
            typedef boost::shared_ptr<RgbdSurfaceTracker> Ptr;

            RgbdSurfaceTracker() {
            }

            virtual ~RgbdSurfaceTracker() {
            };

            void segmentDepth(cv::rgbd::RgbdImage& rgbd_img);

            void callback(cv::Mat& _ocv_rgbframe, cv::Mat& _ocv_depthframe_float,
                    cv::Mat& _rgb_distortionCoeffs, cv::Mat& _rgb_cameraMatrix);
            
            PlaneVisualizationData *getPlaneVisualizationData() {
                return &vis_data;
            }
            
            void iterativeAlignment(cv::rgbd::RgbdImage& rgbd_img);
        private:
            // -------------------------
            // Disabling default copy constructor and default
            // assignment operator.
            // -------------------------
            RgbdSurfaceTracker(const RgbdSurfaceTracker& ref);
            RgbdSurfaceTracker& operator=(const RgbdSurfaceTracker& ref);
            PlaneVisualizationData vis_data;
        };
    } /* namespace rgbd */
} /* namespace cv */

#endif /* __cplusplus */
#endif /* RGBD_UNCC_CONTRIB_H */

