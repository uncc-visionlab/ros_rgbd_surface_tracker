/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */
#include <opencv2/rgbd.hpp>
#include <ros_rgbd_surface_tracker/AlgebraicSurface.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>
#include <ros_rgbd_surface_tracker/Polygonizer.hpp>

extern int supermain(AlgebraicSurface<float>& surf, PlaneVisualizationData& vis_data,
         const Eigen::Matrix<float, 8, 3>& cube, float cubesize, float levelset);

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

            rgbd_img.computeNormals();
            //iterativeAlignment(rgbd_img);

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      
            
            PlaneVisualizationData* vis_data_ptr = this->getPlaneVisualizationData();
            vis_data_ptr->triangles.clear();
            vis_data_ptr->rect_points.clear();
            AlgebraicSurface<float> surf(Eigen::RowVector4f(-0.8, 0, 0, 1), 3, 1);
            Eigen::Matrix<float, 8, 3> cube;
            cube << -1, -1,  0,
                     1, -1,  0,
                     1,  1,  0,
                    -1,  1,  0,
                    -1, -1,  1,
                     1, -1,  1,
                     1,  1,  1,
                    -1,  1,  1;
            cube *= 1;
            
            float cubesize = 0.1;
            float levelset = 0;
//            supermain(surf, *vis_data_ptr, cube, cubesize, levelset);
            
            Polygonizer<float> poly(&surf, vis_data_ptr);
            poly.dataset_size = 1.0/cubesize;
            poly.step_size = cubesize;
            poly.level_set = 0;
            for (int row = 0; row < 8; ++row) {
                for (int col = 0; col < 3; ++col) {
                    poly.vertex_offset[row][col] = cube(row, col);
                }
            }   
            poly.polygonize();
     
            std::cout << "num tris from marching cubes: " << vis_data_ptr->triangles.size() << std::endl;

        }
    } /* namespace rgbd */
} /* namespace cv */
