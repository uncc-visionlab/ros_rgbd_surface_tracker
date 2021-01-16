#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

// ROS includes
#include <message_filters/subscriber.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

// TF includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// subscriber for depth images
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Includes for this Library
#include <ros_rgbd_surface_tracker/ros_rgbd_surface_tracker.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

//typedef float Scalar;
typedef double Scalar;

namespace stdpatch {
#define NUMIDCHARS 3

    template < typename T > std::string to_string(const T& n) {
        std::ostringstream stm;
        stm << std::setw(NUMIDCHARS) << std::setfill('0') << n;
        return stm.str();
    }
}

namespace Eigen {

    void toString(std::string name, Eigen::MatrixXf mat) {

        static std::string sep = "\n----------------------------------------\n";
        static int StreamPrecision = 4;
        static Eigen::IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        std::cout << sep << name << " = " << mat.format(OctaveFmt) << ";" << sep;
    }
}

void ROS_RgbdSurfaceTracker::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg,
        const sensor_msgs::CameraInfoConstPtr& info_msg) {
    static tf::TransformListener listener;
    static int frame_id = 0;
    ROS_INFO("Heard depth image.");
    frame_time = depth_msg->header.stamp;
    std::string keyframe_frameid_str("frame_");
    keyframe_frameid_str.append(stdpatch::to_string(frame_id++));
    depth_encoding = depth_msg->encoding;
    depth_row_step = depth_msg->step;
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        cv_depthimg_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        cv_depthimg_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    //model_.fromCameraInfo(info_msg);
    frame_id_str = keyframe_frameid_str;

    cv::Mat distortionCoeffs(5, 1, CV_32F), cameraMatrix(3, 3, CV_32F);
    cv::Size imSize;
    cameraInfoToCVMats(info_msg, false, cameraMatrix, distortionCoeffs, imSize);

    cv::Mat _ocv_depthframe_float(cv_depthimg_ptr->image.size(), CV_32F);
    createDepthImageFloat(_ocv_depthframe_float);
    static cv::Mat _ocv_rgbframe = cv::Mat(_ocv_depthframe_float.size(), CV_8UC3, cv::Scalar(127, 127, 127));
    rgbdCameraTrackerPtr->callback(_ocv_rgbframe, _ocv_depthframe_float, distortionCoeffs, cameraMatrix);

    cv::Mat rgb_result = _ocv_rgbframe.clone();
    publishResults(depth_msg, rgb_result);
}

tf::Transform convertPoseToTFTransform(const Pose& pose) {
    cv::Mat rotation = pose.getRotation_Mat();
    cv::Vec3f translation;
    pose.getTranslation(translation);
    Eigen::Map<Eigen::Matrix3f> eigenRot((float *) rotation.data);
    //std::cout << "rotation = " << rotation << std::endl;
    eigenRot.transposeInPlace();
    //std::cout << "eigenRot = " << eigenRot << std::endl;
    Eigen::Quaternionf quat(eigenRot);
    return tf::Transform(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()),
            tf::Vector3(translation[0], translation[1], translation[2]));
}

void ROS_RgbdSurfaceTracker::publishResults(const sensor_msgs::ImageConstPtr& depth_msg, cv::Mat& rgb_result) {

    //    rgbdSurfTracker.updateSurfaces(rgbd_img_ptr, rgb_result);

    //    PlaneVisualizationData* vis_data = rgbdSurfTracker.getPlaneVisualizationData();
    //    plane_vis.clearMarkerList();
    //    plane_vis.publishTriangleMesh(*vis_data, depth_msg->header.stamp);

    Pose pose = rgbdCameraTrackerPtr->getPose();
    tf::Transform pose_transformTF = convertPoseToTFTransform(pose);
    tf::StampedTransform pose_stampedTF(pose_transformTF, frame_time,
            parent_frame_id_str, rgbd_frame_id_str);
    tfbroadcaster.sendTransform(pose_stampedTF);

    if (image_pub.getNumSubscribers() > 0) {
        //show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = rgbd_frame_id_str;
        out_msg.header.stamp = frame_time;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = rgb_result;
        image_pub.publish(out_msg.toImageMsg());
    }

    if (pubPose_w_cov.getNumSubscribers() > 0 && true) {
        geometry_msgs::TransformStamped pose_stampedMSG;
        tf::transformStampedTFToMsg(pose_stampedTF, pose_stampedMSG);
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = rgbd_frame_id_str;
        pose_msg.header.stamp = frame_time;
        pose_msg.pose.pose.orientation = pose_stampedMSG.transform.rotation;
        pose_msg.pose.pose.position.x = pose_stampedMSG.transform.translation.x;
        pose_msg.pose.pose.position.y = pose_stampedMSG.transform.translation.y;
        pose_msg.pose.pose.position.z = pose_stampedMSG.transform.translation.z;
        for (int offset = 0; offset < 36; offset++) {
            pose_msg.pose.covariance[offset] = 0.0f; //cov[offset];
        }
        pubPose_w_cov.publish(pose_msg);
    }

    if (pubOdom_w_cov.getNumSubscribers() > 0) {
        Pose delta_pose = rgbdCameraTrackerPtr->getDeltaPose();
        tf::Transform delta_pose_transformTF = convertPoseToTFTransform(delta_pose);
        tf::StampedTransform delta_pose_stampedTF(delta_pose_transformTF, frame_time,
                parent_frame_id_str, rgbd_frame_id_str);
        geometry_msgs::TransformStamped delta_pose_stampedMSG;
        tf::transformStampedTFToMsg(delta_pose_stampedTF, delta_pose_stampedMSG);
        geometry_msgs::PoseWithCovarianceStamped delta_pose_msg;
        delta_pose_msg.header.frame_id = rgbd_frame_id_str;
        delta_pose_msg.header.stamp = frame_time;
        delta_pose_msg.pose.pose.orientation = delta_pose_stampedMSG.transform.rotation;
        delta_pose_msg.pose.pose.position.x = delta_pose_stampedMSG.transform.translation.x;
        delta_pose_msg.pose.pose.position.y = delta_pose_stampedMSG.transform.translation.y;
        delta_pose_msg.pose.pose.position.z = delta_pose_stampedMSG.transform.translation.z;
        for (int offset = 0; offset < 36; offset++) {
            delta_pose_msg.pose.covariance[offset] = 0.0f; //cov[offset];
        }
        pubOdom_w_cov.publish(delta_pose_msg);
    }
}

void ROS_RgbdSurfaceTracker::rgbdImageCallback(const sensor_msgs::ImageConstPtr& depth_msg,
        const sensor_msgs::ImageConstPtr& rgb_msg_in,
        const sensor_msgs::CameraInfoConstPtr & info_msg) {
    static int frame_id = 0;
    //ROS_DEBUG("Heard rgbd image.");
    frame_time = depth_msg->header.stamp;
    std::string keyframe_frameid_str("frame_");
    keyframe_frameid_str.append(stdpatch::to_string(frame_id++));
    depth_encoding = depth_msg->encoding;
    depth_row_step = depth_msg->step;
    cv_rgbimg_ptr = cv_bridge::toCvShare(rgb_msg_in, sensor_msgs::image_encodings::BGR8);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        cv_depthimg_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        cv_depthimg_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    if (depth_msg->width != rgb_msg_in->width || depth_msg->height != rgb_msg_in->height) {
        ROS_ERROR("Depth and RGB image dimensions don't match depth=( %dx%d ) rgb=( %dx%d )!",
                depth_msg->width, depth_msg->height, rgb_msg_in->width, rgb_msg_in->height);
    }
    //model_.fromCameraInfo(info_msg);
    frame_id_str = keyframe_frameid_str;

    cv::Mat distortionCoeffs(5, 1, CV_32F), cameraMatrix(3, 3, CV_32F);
    cv::Size imSize;
    cameraInfoToCVMats(info_msg, false, cameraMatrix, distortionCoeffs, imSize);

    cv::Mat _ocv_depthframe_float(cv_depthimg_ptr->image.size(), CV_32F);
    createDepthImageFloat(_ocv_depthframe_float);

    cv::Mat _ocv_rgbframe = cv_rgbimg_ptr->image.clone();
    rgbdCameraTrackerPtr->callback(_ocv_rgbframe, _ocv_depthframe_float, distortionCoeffs, cameraMatrix);

    cv::Mat rgb_result = _ocv_rgbframe.clone();
    publishResults(depth_msg, rgb_result);
}

void ROS_RgbdSurfaceTracker::createDepthImageFloat(cv::Mat& depth_frame) {
    // Convert Kinect depth image from image-of-shorts (mm) to image-of-floats (m)
    if (depth_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        ROS_DEBUG("Converting Kinect-style depth image to floating point depth image.");
        int width = cv_depthimg_ptr->image.cols;
        int height = cv_depthimg_ptr->image.rows;
        float bad_point = std::numeric_limits<float>::quiet_NaN();
        uint16_t* uint_depthvals = (uint16_t *) cv_depthimg_ptr->image.data;
        float* float_depthvals = (float *) depth_frame.data;
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                if (uint_depthvals[row * width + col] == 0) {
                    float_depthvals[row * width + col] = bad_point;
                } else { // mm to m for kinect
                    float_depthvals[row * width + col] = uint_depthvals[row * width + col]*0.001f;
                }
            }
        }
    } else if (depth_encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth_frame = cv_depthimg_ptr->image.clone();
    }
}

void ROS_RgbdSurfaceTracker::cameraInfoToCVMats(const sensor_msgs::CameraInfoConstPtr &cam_info,
        bool useRectifiedParameters, cv::Mat &cameraMatrix,
        cv::Mat &distortionCoeffs, cv::Size &imSize) {
    imSize = cv::Size(cam_info->height, cam_info->width);
    //std::cout << "Setting camera_info " << cam_info << std::endl;
    if (useRectifiedParameters) {
        cameraMatrix.at<float>(0, 0) = cam_info->P[0];
        cameraMatrix.at<float>(0, 1) = cam_info->P[1];
        cameraMatrix.at<float>(0, 2) = cam_info->P[2];
        cameraMatrix.at<float>(1, 0) = cam_info->P[4];
        cameraMatrix.at<float>(1, 1) = cam_info->P[5];
        cameraMatrix.at<float>(1, 2) = cam_info->P[6];
        cameraMatrix.at<float>(2, 0) = cam_info->P[8];
        cameraMatrix.at<float>(2, 1) = cam_info->P[9];
        cameraMatrix.at<float>(2, 2) = cam_info->P[10];

        for (int i = 0; i < 5; ++i)
            distortionCoeffs.at<float>(i, 0) = 0;
    } else {
        for (int i = 0; i < 9; ++i) {
            //std::cout << cam_info.K[i] <<" " << std::endl;
            cameraMatrix.at<float>(i / 3, i % 3) = cam_info->K[i];
        }
        for (int i = 0; i < 5; ++i) {
            //std::cout << cam_info.D[i] <<" " << std::endl;
            distortionCoeffs.at<float>(i, 0) = cam_info->D[i];
        }
    }
}

void ROS_RgbdSurfaceTracker::initializeSubscribersAndPublishers() {
    int queue_size = 10;
    image_transport::ImageTransport it_depth(*nodeptr);
    // parameter for depth_image_transport hint
    std::string depth_image_transport_param = "depth_image_transport";
    // depth image can use different transport.(e.g. compressedDepth)
    image_transport::TransportHints depth_hints("raw", ros::TransportHints(),
            *nodeptr, depth_image_transport_param);
    //    image_transport::Subscriber sub_depthImage = it_depth.subscribe("depth/image_raw", 1, depth_hints);
    //image_transport::SubscriberFilter sub_depthImage;
    sub_depthImage.subscribe(it_depth, "depth_registered/input_image", 1, depth_hints);

    if (useRGBStream) {
        //message_filters::Subscriber<sensor_msgs::CameraInfo>
        //        sub_rgbImage(*nodeptr, "rgb/image_raw", 1);
        image_transport::ImageTransport it_rgb(*nodeptr);
        // rgb uses normal ros transport hints.
        image_transport::TransportHints hints("raw", ros::TransportHints(), *nodeptr);
        //image_transport::SubscriberFilter sub_rgbImage;
        sub_rgbImage.subscribe(it_rgb, "rgb/input_image", 1, hints);

        //    message_filters::Subscriber<sensor_msgs::CameraInfo>
        //            sub_rgbCameraInfo;
        sub_rgbCameraInfo.subscribe(*nodeptr, "rgb/camera_info", 1);

        // option 1
        //    message_filters::TimeSynchronizer
        //            <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
        //            syncTime(sub_depthImage, sub_rgbImage, sub_rgbCameraInfo, queue_size);
        //    syncTime.registerCallback(boost::bind(&Feature3DEngine::rgbdImageCallback,
        //            engineptr, _1, _2, _3));
        // option 2
        //    typedef message_filters::sync_policies::ExactTime
        //            <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncPolicy;
        // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(queue_size)
        //    message_filters::Synchronizer<MyExactSyncPolicy> syncExact(MyExactSyncPolicy(queue_size),
        //            sub_depthImage, sub_rgbImage, sub_rgbCameraInfo);
        // option 3
        typedef message_filters::sync_policies::ApproximateTime
                <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproximateSyncPolicy;
        static message_filters::Synchronizer<MyApproximateSyncPolicy> syncApprox(MyApproximateSyncPolicy(queue_size),
                sub_depthImage, sub_rgbImage, sub_rgbCameraInfo);
        syncApprox.registerCallback(boost::bind(&ROS_RgbdSurfaceTracker::rgbdImageCallback,
                this, _1, _2, _3));
    } else {
        sub_depthCameraInfo.subscribe(*nodeptr, "depth_registered/camera_info", 1);
        //sub_depthCameraInfo.subscribe(*nodeptr, "ir/camera_info", 1);
        typedef message_filters::sync_policies::ApproximateTime
                <sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproximateSyncPolicy;
        static message_filters::Synchronizer<MyApproximateSyncPolicy> syncApprox(MyApproximateSyncPolicy(queue_size),
                sub_depthImage, sub_depthCameraInfo);
        syncApprox.registerCallback(boost::bind(&ROS_RgbdSurfaceTracker::depthImageCallback,
                this, _1, _2));
    }

    pubPose_w_cov = nodeptr->advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_w_cov", 1000);
    pubOdom_w_cov = nodeptr->advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_w_cov", 1000);
}

int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "ros_rgbd_surface_tracker");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ROS_RgbdSurfaceTracker surf_tracker;
    //surf_tracker_ptr = &surf_tracker;
    
    surf_tracker.initializeSubscribersAndPublishers();
    ros::spin();
    return EXIT_SUCCESS;
}

