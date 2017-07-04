/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <boost/make_shared.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>

#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

#define PROFILE_CALLGRIND false

#ifdef PROFILE_CALLGRIND
#include <valgrind/callgrind.h>
#endif

namespace cv {
    namespace rgbd {

        void RgbdSurfaceTracker::iterativeAlignment(cv::rgbd::RgbdImage& rgbd_img) {

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

            static bool have_initial_guess;
            static AlgebraicSurfaceProduct<double> surface(3);
            Eigen::Matrix4d transform_matrix;
            transform_matrix.setIdentity();

            bool convergence = false;

            if (data.size() > num_samples / 2) {

                // convert points to matrix form
                Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
                for (std::size_t i = 0; i != data.size(); ++i) {
                    data_mat.row(i) << data[i].x, data[i].y, data[i].z;
                }

                if (have_initial_guess) {
                    // run nonlinear optimization to estimate frame-frame odometry
                    convergence = leastSquaresSurfaceFitLM<double>(surface, data_mat, transform_matrix);

                } else {
                    // fit a plane to use as an initial guess
                    cv::Plane3f plane;
                    cv::RectWithError rect;
                    rgbd_img.fitImplicitPlaneLeastSquares(data, plane, rect.error,
                            rect.noise, rect.inliers, rect.outliers, rect.invalid);
                    AlgebraicSurface<double>::Ptr subsurface_ptr =
                            boost::make_shared<PlanarSurface<double>>(
                            Eigen::RowVector4d(plane.d, plane.x, plane.y, plane.z));
                    surface.addSubSurface(subsurface_ptr);
                    have_initial_guess = true;
                }

                if (convergence) { // transform the plane and publish
                    AlgebraicSurface<double>::Ptr subsurface = surface.subsurfaces[0];
                    subsurface->affineTransform(transform_matrix);
                    std::cout << "surface: " << subsurface->toString() << "\n";

                    cv::Point2i middle_pixel;
                    middle_pixel.x = floor(0.5 * rgbd_img.getWidth());
                    middle_pixel.y = floor(0.5 * rgbd_img.getHeight());

                    if (!std::isnan(rgbd_img.getDepth().at<float>(middle_pixel.y, middle_pixel.x))) {

                        cv::Point3f middle = rgbd_img.backproject(middle_pixel);

                        middle.z = -(subsurface->coeffs(0)
                                + subsurface->coeffs(1) * middle.x
                                + subsurface->coeffs(2) * middle.y) / subsurface->coeffs(3);

                        PlaneVisualizationData* viz_data = this->getPlaneVisualizationData();
                        viz_data->plane_point << middle.x, middle.y, middle.z;
                        viz_data->plane_normal << subsurface->coeffs(1),
                                subsurface->coeffs(2),
                                subsurface->coeffs(3);

                    }
                }
            }

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif

        }
    } /* namespace rgbd */
} /* namespace cv */

