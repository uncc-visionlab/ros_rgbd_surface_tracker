/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <boost/make_shared.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>

#include <opencv2/imgproc.hpp>

#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>
#include <ros_rgbd_surface_tracker/Polygonizer.hpp>

namespace cv {
    namespace rgbd {
        
        void planeAlignment(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker) {

            int tile_width = 100, tile_height = 100;
            int x0 = floor(0.5 * (rgbd_img.getWidth() - tile_width));
            int y0 = floor(0.5 * (rgbd_img.getHeight() - tile_height));
            int num_samples = 100;
            
            cv::Point2i tl, tr, br, bl;
                    
            tl.x = x0;
            tl.y = y0;

            tr.x = x0 + tile_width;
            tr.y = y0;

            br.x = x0 + tile_width;
            br.y = y0 + tile_height;

            bl.x = x0;
            bl.y = y0 + tile_height;

            std::vector<cv::Point2i> corners;
            corners.reserve(4);
            corners.push_back(bl);
            corners.push_back(br);
            corners.push_back(tr);
            corners.push_back(tl);
            
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
                    
                    PlaneVisualizationData* vis_data = surface_tracker.getPlaneVisualizationData();
                    vis_data->rect_points.clear();
                    
                    for (std::size_t i = 0; i != 4; i++) {

                        if (!std::isnan(rgbd_img.getDepth().at<float>(corners[i].y, corners[i].x))) {

                            cv::Point3f point = rgbd_img.backproject(corners[i]);

                            point.z = -(subsurface->coeffs(0)
                                    + subsurface->coeffs(1) * point.x
                                    + subsurface->coeffs(2) * point.y) / subsurface->coeffs(3);

                            
                            vis_data->rect_points.push_back(Eigen::Vector3f(point.x, point.y, point.z));

                        }
                    }
                }
            }

        }

        void edgeAlignment(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker) {
            
             // collecting data from two tiles in depth image
            int tile_width = 100, tile_height = 100;
            int x1 = floor(0.5*(rgbd_img.getWidth() - tile_width)) - 100;
            int x2 = x1 + 200;
            int y1 = floor(0.5*(rgbd_img.getHeight() - tile_height));
            int y2 = y1;
            int samples_per_tile = 100;
            
            std::vector<cv::Point3f> data, tileA_data, tileB_data;
            data.reserve(2*samples_per_tile);
            tileA_data.reserve(samples_per_tile);
            tileB_data.reserve(samples_per_tile);

            rgbd_img.getTileData_Uniform(
                x1, y1, tile_width, tile_height, tileA_data, samples_per_tile);

            rgbd_img.getTileData_Uniform(
                x2, y1, tile_width, tile_height, tileB_data, samples_per_tile);

            if (tileA_data.size() + tileB_data.size() > samples_per_tile) {

                static bool have_initial_guess;

                static AlgebraicSurfaceProduct<double> surface(3);
                Eigen::Matrix4d transform_matrix;
                transform_matrix.setIdentity();

                bool convergence = false;

                if (have_initial_guess) {

                    // move points to common std::vector (this avoids copy)
                    data = std::move(tileA_data);
                    std::move(std::begin(tileB_data), std::end(tileB_data), std::back_inserter(data));

                    // convert points to matrix form (copying unfortunatly)
                    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
                    for (std::size_t i = 0; i != data.size(); ++i) {
                        data_mat.row(i) << data[i].x, data[i].y, data[i].z;
                    }

                    // run nonlinear optimization to estimate frame-frame odometry
                    convergence = leastSquaresSurfaceFitLM<double>(surface, data_mat, transform_matrix);

                } else {
                    // fit edge for initial guess
                    cv::Plane3f planeA, planeB;
                    cv::RectWithError rect;
                    rgbd_img.fitImplicitPlaneLeastSquares(tileA_data, planeA, rect.error,
                        rect.noise, rect.inliers, rect.outliers, rect.invalid);
                    rgbd_img.fitImplicitPlaneLeastSquares(tileB_data, planeB, rect.error,
                        rect.noise, rect.inliers, rect.outliers, rect.invalid);
                    AlgebraicSurface<double>::Ptr subsurfaceA_ptr = 
                        boost::make_shared<PlanarSurface<double>>(
                            Eigen::RowVector4d(planeA.d, planeA.x, planeA.y, planeA.z));
                    AlgebraicSurface<double>::Ptr subsurfaceB_ptr = 
                        boost::make_shared<PlanarSurface<double>>(
                            Eigen::RowVector4d(planeB.d, planeB.x, planeB.y, planeB.z));
                    surface.addSubSurface(subsurfaceA_ptr);
                    surface.addSubSurface(subsurfaceB_ptr);
                    have_initial_guess = true;
                }

                if (convergence) { // transform the surface and publish
                    
                    surface.affineTransform(transform_matrix);
                    
                    PlaneVisualizationData* vis_data_ptr = surface_tracker.getPlaneVisualizationData();
                    vis_data_ptr->rect_points.clear();
                    vis_data_ptr->triangles.clear();
                    
                    static Polygonizer<double> poly(
                        Polygonizer<double>::EvaluationVolume(0, 0, 2.5, 6, 4, 5, 10, 10, 10),
                        &surface, 
                        vis_data_ptr);
                    poly.polygonize();
                    
                }

            }

        }
        
        constexpr double pi() { return std::atan(1)*4; }
        
        void cornerAlignment(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker,
                cv::Mat& rgb_result) {
            
             // collecting data from three tiles in depth image
            int tile_width = 30, tile_height = 30;
            int samples_per_tile = 100;
            
            float radius = 150; //pixels
            std::array<float, 3> angles = {pi()/2 + pi()/3, 7*pi()/6 + pi()/3, 11*pi()/6 + pi()/3};
            std::vector<cv::Rect2i> rects;
            rects.reserve(3);
            
            for (auto& angle : angles) {
                cv::Rect2i rectVal(radius*std::cos(angle) + (rgbd_img.getWidth() - tile_width)/2.0,
                        radius*std::sin(angle) + (rgbd_img.getHeight() - tile_height)/2.0,
                        tile_width, tile_height);
                rects.emplace_back(rectVal
                  );
                cv::rectangle(rgb_result, rectVal, cv::Scalar( 0, 255, 0), 3);
            }
            
            std::array<std::vector<cv::Point3f>, 3> tile_data; // data for each tile in array of vectors
            int amount_data = 0;
            
            for (std::size_t rectid = 0; rectid != rects.size(); ++rectid) {
                
                tile_data[rectid].reserve(samples_per_tile);
                
                rgbd_img.getTileData_Uniform(rects[rectid].x, rects[rectid].y, 
                        rects[rectid].width, rects[rectid].height, 
                        tile_data[rectid], samples_per_tile);
                
                amount_data += tile_data[rectid].size();
                
            }
            
            std::vector<cv::Point3f> data; // this will hold the points from all of tile_data
            data.reserve(amount_data);

            if (amount_data > 2*samples_per_tile) {

                static bool have_initial_guess;

                static AlgebraicSurfaceProduct<double> surface(3);
                Eigen::Matrix4d transform_matrix;
                transform_matrix.setIdentity();

                bool convergence = false;

                if (have_initial_guess) {

                    // move points to common std::vector (this avoids copy)
                    data = std::move(tile_data[0]);
                    for (std::size_t rectid = 1; rectid != tile_data.size(); ++rectid)
                        std::move(std::begin(tile_data[rectid]), 
                                std::end(tile_data[rectid]), 
                                std::back_inserter(data));

                    // convert points to matrix form (copying unfortunatly)
                    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
                    for (std::size_t i = 0; i != data.size(); ++i) {
                        data_mat.row(i) << data[i].x, data[i].y, data[i].z;
                    }

                    // run nonlinear optimization to estimate frame-frame odometry
                    convergence = leastSquaresSurfaceFitLM<double>(surface, data_mat, transform_matrix);

                } else {
                    // fit surface for initial guess
                    
                    for (std::size_t rectid = 0; rectid != rects.size(); ++rectid) {
                        cv::Plane3f plane;
                        cv::RectWithError rect;
                        
                        rgbd_img.fitImplicitPlaneLeastSquares(tile_data[rectid], plane, rect.error,
                            rect.noise, rect.inliers, rect.outliers, rect.invalid);
                        
                        AlgebraicSurface<double>::Ptr subsurface_ptr = 
                        boost::make_shared<PlanarSurface<double>>(
                            Eigen::RowVector4d(plane.d, plane.x, plane.y, plane.z));
                        
                        surface.addSubSurface(subsurface_ptr);
                    }
                    
                    have_initial_guess = true;
                }

                if (convergence) { // transform the surface and publish
                    
                    surface.affineTransform(transform_matrix);
                    
                    PlaneVisualizationData* vis_data_ptr = surface_tracker.getPlaneVisualizationData();
                    vis_data_ptr->rect_points.clear();
                    vis_data_ptr->triangles.clear();
                    
                    static Polygonizer<double> poly(
                        Polygonizer<double>::EvaluationVolume(0, 0, 3, 1, 1, 6, 40, 40, 40),
                        &surface, 
                        vis_data_ptr);
                    poly.polygonize();
                    
                }

            }

        }

        void RgbdSurfaceTracker::iterativeAlignment(cv::rgbd::RgbdImage& rgbd_img, cv::Mat& rgb_result) {
            
            cornerAlignment(rgbd_img, *this, rgb_result);

        }
        
    } /* namespace rgbd */
} /* namespace cv */

