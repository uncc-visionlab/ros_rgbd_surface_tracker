/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ros_rgbd_surface_tracker.h
 * Author: arwillis
 *
 * Created on June 30, 2017, 10:27 AM
 */

#ifndef ROS_RGBD_SURFACE_TRACKER_H
#define ROS_RGBD_SURFACE_TRACKER_H

#ifdef __cplusplus

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>

// TF includes
#include <tf/tf.h>
//#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros_rgbd_surface_tracker/rgbd_uncc_contrib.hpp>

class ROS_RgbdSurfaceTracker {
public:
    typedef boost::shared_ptr<ROS_RgbdSurfaceTracker> Ptr;

    ROS_RgbdSurfaceTracker() :
    nodeptr(new ros::NodeHandle),
    nh("~"), it(nh) {

        nh.param<std::string>("map_frame", map_frame_id_str, "optitrack");
        nh.param<std::string>("optical_parent", parent_frame_id_str, "optitrack");
        nh.param<std::string>("optical_frame", rgbd_frame_id_str, "rgbd_frame");

        image_pub = it.advertise("result", 1);
    };

    virtual ~ROS_RgbdSurfaceTracker() {
    };

    void initializeSubscribersAndPublishers();

    void rgbdImageCallback(const sensor_msgs::ImageConstPtr& depth_msg,
            const sensor_msgs::ImageConstPtr& rgb_msg_in,
            const sensor_msgs::CameraInfoConstPtr& info_msg);

    void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg,
            const sensor_msgs::CameraInfoConstPtr& info_msg);

    void createDepthImageFloat(cv::UMat depth_frame);

    void cameraInfoToCVMats(const sensor_msgs::CameraInfoConstPtr &cam_info,
            bool useRectifiedParameters, cv::Mat &cameraMatrix,
            cv::Mat &distortionCoeffs, cv::Size &imSize);


private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    ROS_RgbdSurfaceTracker(const ROS_RgbdSurfaceTracker& yRef);
    ROS_RgbdSurfaceTracker& operator=(const ROS_RgbdSurfaceTracker& yRef);

    ros::NodeHandle nh;
    ros::NodeHandlePtr nodeptr;
    ros::Time frame_time;
    std::string frame_id_str;
    cv_bridge::CvImageConstPtr cv_rgbimg_ptr;
    cv_bridge::CvImageConstPtr cv_depthimg_ptr;
    image_geometry::PinholeCameraModel model_;
    std::string depth_encoding;
    int depth_row_step;

    std::string map_frame_id_str; // frame id for parent of the RGBD sensor
    std::string parent_frame_id_str; // frame id for parent of the RGBD sensor
    std::string rgbd_frame_id_str; // frame id for RGBD sensor      

    // subscribers to RGBD sensor data
    image_transport::SubscriberFilter sub_depthImage;
    image_transport::SubscriberFilter sub_rgbImage;
    ros::Subscriber sub_trigger_ptcloud_pub;

    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgbCameraInfo;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depthCameraInfo;

    ros::Publisher pubPoseWCovariance;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;

    cv::rgbd::RgbdSurfaceTracker rgbdSurfTracker;
};

#endif /* __cplusplus */
#endif /* ROS_RGBD_SURFACE_TRACKER_H */

