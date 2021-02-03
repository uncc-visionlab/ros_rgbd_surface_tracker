/* 
 * File:   rgbd_uncc_contrib.h
 * Author: arwillis
 *
 * Created on June 30, 2017, 10:01 AM
 */

#ifndef RGBD_TRACKER_HPP
#define RGBD_TRACKER_HPP

#ifdef __cplusplus

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <rgbd_drivers_uncc/rgbd_driver_client.hpp>
#include <rgbd_odometry/pose.hpp>

class RgbdCameraTracker : public RgbdDriverClient {
public:
    typedef boost::shared_ptr<RgbdCameraTracker> Ptr;

    virtual Eigen::Affine3d getPose() = 0;

    virtual Eigen::Affine3d getDeltaPose() = 0;

    static Eigen::Affine3d toEigen(const Pose pose) {
        cv::Matx44f pose_cv = pose.getTransform();
        Eigen::Matrix4f pose_eigen;
        cv::cv2eigen(pose_cv, pose_eigen);
        Eigen::Affine3d pose_out;
        pose_out.matrix() = pose_eigen.cast<double>();
        return pose_out;
    }

    static Pose fromEigen(const Eigen::Matrix4d& pose_eigen) {
        cv::Matx44f pose_cv;
        Eigen::Matrix4f pose_eigenf = pose_eigen.cast<float>();
        cv::eigen2cv(pose_eigenf, pose_cv);
        Pose pose;
        pose.set(pose_cv);
        return pose;
    }

    static Pose fromEigen(const Eigen::Affine3d& pose_eigen) {
        return fromEigen(pose_eigen.matrix());
    }
}; /* class RgbdCameraTracker */

#endif /* __cplusplus */
#endif /* RGBD_UNCC_CONTRIB_H */

