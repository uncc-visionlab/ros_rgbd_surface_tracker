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
            int half_width = floor(0.5*tile_width);
            int half_height = floor(0.5*tile_height);
            int x1 = floor(0.5*(rgbd_img.getWidth() - tile_width)) - 100;
            int x2 = x1 + 200;
            int y1 = floor(0.5*(rgbd_img.getHeight() - tile_height));
            int y2 = y1;
            int samples_per_tile = 100;
            
            std::vector<cv::Rect2i> tiles;
            tiles.reserve(2);
            tiles.emplace_back(x1, y1, tile_width, tile_height);
            tiles.emplace_back(x2, y2, tile_width, tile_height);
            
            std::vector<cv::Point2i> corners;
            corners.reserve(4*tiles.size());
            for (auto& tile : tiles) {
                corners.emplace_back(tile.tl());
                corners.emplace_back(tile.x + tile.width, tile.y);
                corners.emplace_back(tile.br());
                corners.emplace_back(tile.x, tile.y + tile.height);
            }
            
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
                    
                    PlaneVisualizationData* vis_data = surface_tracker.getPlaneVisualizationData();
                    vis_data->rect_points.clear();
                    
                    cv::Mat camera_matrix = rgbd_img.getCameraMatrix();
                    float fx = camera_matrix.at<float>(0, 0); // px
                    float fy = camera_matrix.at<float>(1, 1); // px
                    float cx = camera_matrix.at<float>(0, 2); // px
                    float cy = camera_matrix.at<float>(1, 2); // px
                    float s = 1.9e-6; // m/px
                    float f = s*(fx + fy)/2; // m
                    
                    for (auto& corner_px : corners) {
                        
                        cv::Line3f ray(
                            cv::Point3f(s*(corner_px.x - cx), s*(corner_px.y - cy), s*(fx + fy)/2),
                            cv::Point3f(0, 0, 0));
                            
                        cv::Point3f closest_pt(0, 0, std::numeric_limits<float>::infinity());
                        
                        for (auto& subsurface : surface.subsurfaces) {
                            
                            cv::Plane3f subsurface_plane(
                                    subsurface->coeffs(1), 
                                    subsurface->coeffs(2), 
                                    subsurface->coeffs(3), 
                                    subsurface->coeffs(0));
                            
                            cv::Point3f intersection_pt;
                            subsurface_plane.intersect(ray, intersection_pt);
                                
                            if (intersection_pt.z < closest_pt.z)
                                closest_pt = intersection_pt;

                        }
                        
                        vis_data->rect_points.emplace_back(closest_pt.x, closest_pt.y, closest_pt.z);
                        
                    }
                    
                }

            }

        }

        void RgbdSurfaceTracker::iterativeAlignment(cv::rgbd::RgbdImage& rgbd_img) {
            
            edgeAlignment(rgbd_img, *this);

        }
        
    } /* namespace rgbd */
} /* namespace cv */

