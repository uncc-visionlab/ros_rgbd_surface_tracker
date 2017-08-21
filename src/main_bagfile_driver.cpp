/* 
 * File:   main_bagfile_driver.cpp
 * Author: arwillis
 *
 * Created on August 21, 2017, 6:39 AM
 */

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS Bridge to OpenCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

#define LOCAL_DEBUG false

class BagFileDriver {
    rosbag::Bag input_bag;
    rosbag::View* viewptr;
    rosbag::View::iterator view_iter;
    rosbag::View::iterator view_iter_end;
    std::vector<std::string> topics;

    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback_;

    sensor_msgs::Image::ConstPtr rgb_image_msg;
    sensor_msgs::Image::ConstPtr depth_image_msg;
    sensor_msgs::CameraInfo::ConstPtr camera_info_msg;
    cv_bridge::CvImageConstPtr cv_depthimg_ptr;

public:

    BagFileDriver(char *bagfile) {
        input_bag.open(bagfile, rosbag::bagmode::Read);
    }

    ~BagFileDriver() {
        input_bag.close();
    }

    void setTopics(std::vector<std::string>& _topics) {
        topics = _topics;
        viewptr = new rosbag::View(input_bag, rosbag::TopicQuery(topics));
        view_iter = viewptr->begin();
        view_iter_end = viewptr->end();
    }

    void readRGBDMessages() {
        bool haveRGBImage = false, haveDepthImage = false, haveCameraInfo = false;
        bool stop = false;

        for (; view_iter != view_iter_end && !stop; ++view_iter) {
            rosbag::MessageInstance m = *view_iter;

            sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
            if (img != NULL) {
                if (LOCAL_DEBUG) {
                    std::cout << "Heard an image message on topic " << m.getTopic()
                            << " Time = " << m.getTime() << "." << std::endl;
                }
                if (m.getTopic().find("depth") != std::string::npos) {
                    // THIS IS A DEPTH IMAGE MESSAGE
                    depth_image_msg = img;
                    haveDepthImage = true;
                }
                if (m.getTopic().find("rgb") != std::string::npos) {
                    // THIS IS A RGB IMAGE MESSAGE
                    rgb_image_msg = img;
                    haveRGBImage = true;
                    if (false) {
                        cv_bridge::CvImageConstPtr cv_rgbimg_ptr = cv_bridge::toCvShare(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
                        cv::UMat frame = cv_rgbimg_ptr->image.getUMat(cv::ACCESS_READ);
                        //cv::imwrite("bagfile_" + std::to_string(++outputIndex) + "_image.png", frame);
                    }
                }
            }
            sensor_msgs::CameraInfo::ConstPtr ci = m.instantiate<sensor_msgs::CameraInfo>();
            if (ci != NULL) {
                if (LOCAL_DEBUG) {
                    std::cout << "Heard an camera info message on topic " << m.getTopic()
                            << " Time = " << m.getTime() << "." << std::endl;
                }
                // THIS IS A CAMERA INFO MESSAGE
                camera_info_msg = ci;
                haveCameraInfo = true;
            }
            if (haveRGBImage && haveDepthImage && haveCameraInfo) {
                haveRGBImage = haveDepthImage = haveCameraInfo = false;
                invokeCallback();
            }
        }
    }

    void setCallback(boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&)> const &cb) {
        callback_ = cb;
    }

    void createDepthImageFloat(cv::Mat& depth_frame) {
        // Convert Kinect depth image from image-of-shorts (mm) to image-of-floats (m)
        if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
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
        } else if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_frame = cv_depthimg_ptr->image;
        }
    }

    void cameraInfoToCVMats(const sensor_msgs::CameraInfoConstPtr &cam_info,
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

    void invokeCallback() {
        cv_bridge::CvImageConstPtr cv_rgbimg_ptr;
        //cv_bridge::CvImageConstPtr cv_depthimg_ptr;
        if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_depthimg_ptr = cv_bridge::toCvShare(depth_image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } else if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_depthimg_ptr = cv_bridge::toCvShare(depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        cv::Mat distortionCoeffs(5, 1, CV_32F), cameraMatrix(3, 3, CV_32F);
        cv::Size imSize;
        cameraInfoToCVMats(camera_info_msg, false, cameraMatrix, distortionCoeffs, imSize);

        cv::Mat _ocv_depthframe_float(cv_depthimg_ptr->image.size(), CV_32F);
        createDepthImageFloat(_ocv_depthframe_float);

        cv_rgbimg_ptr = cv_bridge::toCvShare(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat _ocv_rgbframe = cv_rgbimg_ptr->image.clone();
        //cv::Mat _ocv_rgbframe = cv::Mat(_ocv_depthframe_float.size(), CV_8UC3, cv::Scalar(127, 127, 127));
        float cx = cameraMatrix.at<float>(0, 2);
        float cy = cameraMatrix.at<float>(1, 2);
        float fx = cameraMatrix.at<float>(0, 0);
        float fy = cameraMatrix.at<float>(1, 1);
        callback_(_ocv_rgbframe, _ocv_depthframe_float, distortionCoeffs, cameraMatrix);
    }

};

int main(int argc, char **argv) {

    int numBagfiles = argc - 1;
    if (numBagfiles < 1) {
        std::cout << "Please specify the input bag file as an argument." << std::endl;
        return 0;
    }

    ros::Time::init();

    std::cout << "Opening bag file " << 1
            << ": " << argv[1] << " for reading ...." << std::endl;
    BagFileDriver bagReader(argv[1]);

    // hook up driver to surface tracking class
    cv::rgbd::RgbdSurfaceTracker rgbdSurfTracker;
    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback(
            boost::bind(&cv::rgbd::RgbdSurfaceTracker::callback, &rgbdSurfTracker, _1, _2, _3, _4));

    bagReader.setCallback(callback);    
    
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    //topics.push_back(std::string("/camera/rgb/input_image"));
    topics.push_back(std::string("/camera/rgb/camera_info"));
    //topics.push_back(std::string("/camera/depth_registered/input_image"));
    topics.push_back(std::string("/camera/depth/image"));
    bagReader.setTopics(topics);

    ros::Time myTime = ros::Time::now();
    bagReader.readRGBDMessages();

    std::cout << "Closing bag file " << 1
            << ": " << argv[1] << " for reading ...." << std::endl;
}
