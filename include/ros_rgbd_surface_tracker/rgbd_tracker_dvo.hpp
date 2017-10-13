/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RGBD_TRACKER_DVO_HPP_
#define RGBD_TRACKER_DVO_HPP_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/core/eigen.hpp>

#include <image_transport/image_transport.h>

#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>

#include <ros_rgbd_surface_tracker/rgbd_tracker.hpp>
#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>

#include <dvo/dense_tracking.h>
#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/surface_pyramid.h>
#include <dvo/visualization/camera_trajectory_visualizer.h>

namespace dvo_ros {

    class DVOCameraDenseTracker : public RgbdCameraTracker {
    private:

        uint32_t width;
        uint32_t height;

        boost::shared_ptr<dvo::DenseTracker> tracker;
        dvo::DenseTracker::Config tracker_cfg;
        dvo::core::RgbdCameraPyramidPtr camera;
        dvo::core::RgbdImagePyramidPtr current, reference;

        Eigen::Affine3d accumulated_transform, from_baselink_to_camera, latest_absolute_transform_;

        Pose pose, delta_pose;
        size_t frames_since_last_success;

        bool use_dense_tracking_estimate_;

    public:
        typedef boost::shared_ptr<DVOCameraDenseTracker> Ptr;

        DVOCameraDenseTracker() :
        tracker_cfg(dvo::DenseTracker::getDefaultConfig()),
        frames_since_last_success(0),
        use_dense_tracking_estimate_(false) {
            //ROS_INFO("CameraDenseTracker::ctor(...)");

            //pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("rgbd/pose", 1);

            //ReconfigureServer::CallbackType reconfigure_server_callback = boost::bind(&CameraDenseTracker::handleConfig, this, _1, _2);
            //reconfigure_server_.setCallback(reconfigure_server_callback);

            //dvo_ros::util::tryGetTransform(from_baselink_to_asus, tl, "base_link", "asus");

            //ROS_INFO_STREAM("transformation: base_link -> asus" << std::endl << from_baselink_to_asus.matrix());

            //pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pelican/pose", 1, &CameraDenseTracker::handlePose, this);

            latest_absolute_transform_.setIdentity();
            from_baselink_to_camera.setIdentity();
            accumulated_transform.setIdentity();
        };

        virtual ~DVOCameraDenseTracker() {
        };

        static DVOCameraDenseTracker::Ptr create() {
            return DVOCameraDenseTracker::Ptr(boost::make_shared<DVOCameraDenseTracker>());
        }

        void callback(cv::Mat& _ocv_rgbframe, cv::Mat& _ocv_depthframe_float,
                cv::Mat& _rgb_distortionCoeffs, cv::Mat& _rgb_cameraMatrix) {
            //std::cout << "In dvo_ros callback!" << std::endl;

            // different size of rgb and depth image
            int depth_width = _ocv_depthframe_float.cols;
            int depth_height = _ocv_depthframe_float.rows;
            int rgb_width = _ocv_rgbframe.cols;
            int rgb_height = _ocv_rgbframe.rows;
            if (depth_width != rgb_width || depth_height != rgb_height) {
                std::cout << "WARN: RGB and depth image have different size!" << std::endl;
                return;
            }

            // something has changed
            if (hasChanged(rgb_width, rgb_height)) {
                ROS_WARN("RGB image size has changed, resetting tracker!");
                reset(_rgb_cameraMatrix, rgb_width, rgb_height);
            }


            cv::Mat intensity, depth;

            if (_ocv_rgbframe.channels() == 3) {
                cv::Mat tmp;
                cv::cvtColor(_ocv_rgbframe, tmp, CV_BGR2GRAY, 1);

                tmp.convertTo(intensity, CV_32F);
            } else {
                _ocv_rgbframe.convertTo(intensity, CV_32F);
            }

            if (_ocv_depthframe_float.type() == CV_16UC1) {
                dvo::core::SurfacePyramid::convertRawDepthImageSse(_ocv_depthframe_float, depth, 0.001);
            } else {
                depth = _ocv_depthframe_float;
            }

            reference.swap(current);
            current = camera->create(intensity, depth);

            static Eigen::Affine3d first;

            if (!reference) {
                accumulated_transform = latest_absolute_transform_ * from_baselink_to_camera;
                first = accumulated_transform;
                return;
            }

            Eigen::Affine3d transform;
            bool success = tracker->match(*reference, *current, transform);
            Eigen::Matrix4d matVal;
            matVal = transform.matrix();
            cv::Mat cvTransform(4, 4, CV_32F);
            cv::eigen2cv(matVal, cvTransform);
            delta_pose.set(cvTransform);
            //std::cout << "transform = [" << matVal << "]" << std::endl;
            //std::cout << "cvTransform = [" << cvTransform << "]" << std::endl;

            if (success) {
                frames_since_last_success = 0;
                accumulated_transform = accumulated_transform * transform;
                Pose::multiplyInPlace(pose, delta_pose, pose);
                Eigen::Matrix<double, 6, 6> covariance;

                //tracker->getCovarianceEstimate(covariance);

                //std::cerr << covariance << std::endl << std::endl;

            } else {
                frames_since_last_success++;
                reference.swap(current);
                //ROS_WARN("fail");
                std::cout << "fail" << std::endl;
            }
        }

        Pose getPose() {
            return pose;
        }

        Pose getDeltaPose() {
            return delta_pose;
        }

        bool hasChanged(const int lwidth, const int lheight) {
            return width != lwidth || height != lheight;
        }

        void reset(cv::Mat& _rgb_cameraMatrix, const int lwidth, const int lheight) {

            //IntrinsicMatrix intrinsics = IntrinsicMatrix::create(camera_info_msg->K[0], camera_info_msg->K[4], camera_info_msg->K[2], camera_info_msg->K[5]);
            dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(_rgb_cameraMatrix.at<float>(0, 0),
                    _rgb_cameraMatrix.at<float>(1, 1), _rgb_cameraMatrix.at<float>(0, 2), _rgb_cameraMatrix.at<float>(1, 2));

            camera.reset(new dvo::core::RgbdCameraPyramid(lwidth, lheight, intrinsics));
            camera->build(tracker_cfg.getNumLevels());

            tracker.reset(new dvo::DenseTracker(tracker_cfg));

            static dvo::core::RgbdImagePyramid * const __null__ = 0;

            reference.reset(__null__);
            current.reset(__null__);

            width = lwidth;
            height = lheight;

            //vis_->reset();
        }

    };

} /* namespace dvo_ros */
#endif /* RGBD_TRACKER_DVO_HPP_ */
