/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   surface_fitting.hpp
 * Author: arwillis
 *
 * Created on July 13, 2017, 12:47 PM
 */

#ifndef SURFACE_FITTING_HPP
#define SURFACE_FITTING_HPP

#ifdef __cplusplus

#include <vector>

#include <opencv2/core.hpp>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>

namespace cv {
    namespace uncc {

        class SurfaceFitting {
        public:
            void fitExplicitPlaneLeastSquares(std::vector<Point3f>& data3, cv::Plane3f& plane3, float& error,
                    float& noise, int& inliers, int& outliers, int& invalid) const;

            void fitImplicitPlaneLeastSquares(std::vector<Point3f>& data3, Plane3f& plane3, float& error,
                    float& noise, int& inliers, int& outliers, int& invalid) const;
        };
    } /* namespace uncc */
} /* namespace cv */

#endif /* __cplusplus */
#endif /* SURFACE_FITTING_HPP */

