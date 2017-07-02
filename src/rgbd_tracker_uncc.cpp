/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */

#include <boost/make_shared.hpp>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>

#include <ros_rgbd_surface_tracker/AlgebraicSurface.hpp>
#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

#define PROFILE_CALLGRIND false

#ifdef PROFILE_CALLGRIND
#include <valgrind/callgrind.h>
#endif

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
            int tile_width = 100, tile_height = 100;
            int x0 = floor(0.5 * (rgbd_img.getWidth() - tile_width));
            int y0 = floor(0.5 * (rgbd_img.getHeight() - tile_height));
            int num_samples = 100;

            std::vector<cv::Point3f> data;
            data.reserve(num_samples);

            bool got_all_samples = rgbd_img.getTileData_Uniform(
                    x0, y0, tile_width, tile_height, data, num_samples);

            //for (int ii = 0; ii < data.size(); ii++) {
            //    std::cout << "datapt[ " << ii << "] = " << data[ii] << std::endl;
            //}

            static bool have_initial_guess;
            static cv::Plane3f plane;
            cv::RectWithError rect;

            AlgebraicSurfaceProduct<double> surface(3);
            Eigen::Matrix4d transform_matrix;
            transform_matrix.setIdentity();

            bool convergence = true;

            if (data.size() < num_samples / 2) {
                return;
            }

            // convert points to matrix form
            Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
            for (std::size_t i = 0; i != data.size(); ++i) {
                data_mat.row(i) << data[i].x, data[i].y, data[i].z;
            }

            if (have_initial_guess) {
                // run nonlinear optimization to estimate frame-frame odometry
                boost::shared_ptr<AlgebraicSurface<double>> subsurface_ptr =
                        boost::make_shared<PlanarSurface<double>>(
                        Eigen::RowVector4d(plane.d, plane.x, plane.y, plane.z));
                surface.addSubSurface(subsurface_ptr);
                convergence = EigenLeastSquaresSurfaceFitLM(surface, data_mat, transform_matrix);

            } else {
                // fit a plane to use as an initial guess
                rgbd_img.fitImplicitPlaneLeastSquares(data, plane, rect.error,
                        rect.noise, rect.inliers, rect.outliers, rect.invalid);
                have_initial_guess = true;
            }

            if (convergence) { // transform the plane and publish
                //rgbd_targeting::planes planesmsg;
                //planesmsg.header.frame_id = map_frame_id_str;
                //planesmsg.header.stamp = frame_time;

                cv::Point2i middle_pixel;
                middle_pixel.x = floor(0.5 * rgbd_img.getWidth());
                middle_pixel.y = floor(0.5 * rgbd_img.getHeight());

                if (!std::isnan(rgbd_img.getDepth().at<float>(middle_pixel.y, middle_pixel.x))) {
                    // rgbd_targeting::plane planedata;
                    cv::Point3f middle = rgbd_img.backproject(middle_pixel);
                    middle.z = -(plane.d + plane.x * middle.x + plane.y * middle.y) / plane.z;

                    Eigen::Vector4f plane_point(middle.x, middle.y, middle.z, 1.0);
                    Eigen::Vector4f transformed_point;
                    transformed_point = transform_matrix.cast<float>() * plane_point;
                    Eigen::Vector4f plane_norm(plane.x, plane.y, plane.z, 0.0);
                    Eigen::Vector4f transformed_norm;
                    transformed_norm = transform_matrix.cast<float>() * plane_norm;

                    plane.x = transformed_norm(0);
                    plane.y = transformed_norm(1);
                    plane.z = transformed_norm(2);
                    plane.d = -(plane.x * transformed_point(0)
                            + plane.y * transformed_point(1)
                            + plane.z * transformed_point(2));
                    plane.convertHessianNormalForm();
                    //planedata.normal.x = transformed_norm(0);
                    //planedata.normal.y = transformed_norm(1);
                    //planedata.normal.z = transformed_norm(2);
                    //planedata.point.x = transformed_point(0);
                    //planedata.point.y = transformed_point(1);
                    //planedata.point.z = transformed_point(2);
                    //planesmsg.planes.push_back(planedata);
                    //pubPlanes.publish(planesmsg);
                }
            }
            std::cout << "plane coeffs: " << plane.toString() << "\n";
#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif
        }
    } /* namespace rgbd */
} /* namespace cv */
