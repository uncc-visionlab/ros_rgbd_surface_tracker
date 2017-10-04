/* 
 * File:   rgbd_uncc_contrib.h
 * Author: arwillis
 *
 * Created on June 30, 2017, 10:01 AM
 */

#ifndef RGBD_TRACKER_HPP
#define RGBD_TRACKER_HPP

#ifdef __cplusplus

#include <opencv2/core.hpp>

#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>

class RgbdCameraTracker {
public:
    typedef boost::shared_ptr<RgbdCameraTracker> Ptr;


    virtual void callback(cv::Mat& _ocv_rgbframe, cv::Mat& _ocv_depthframe_float,
            cv::Mat& _rgb_distortionCoeffs, cv::Mat& _rgb_cameraMatrix) = 0;

    virtual Pose getPose() = 0;
    
    virtual Pose getDeltaPose() = 0;

}; /* class RgbdCameraTracker */

#endif /* __cplusplus */
#endif /* RGBD_UNCC_CONTRIB_H */

