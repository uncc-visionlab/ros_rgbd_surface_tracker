/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rgbd_tracker_datatypes.hpp
 * Author: arwillis
 *
 * Created on July 13, 2017, 1:36 PM
 */

#ifndef RGBD_TRACKER_DATATYPES_HPP
#define RGBD_TRACKER_DATATYPES_HPP
#ifdef __cplusplus
namespace cv {
    namespace rgbd {

        class AlgebraicSurfacePatch {
        public:
            int surfaceType;
        }; /* class AlgebraicSurfacePatch */

        class ObjectGeometry {
        public:
            std::vector<cv::Vec3f> verts;
            std::vector<cv::Vec3f> colors;
        }; /* class ObjectGeometry */

    } /* namespace rgbd */
} /* namespace cv */
#endif /* __cplusplus */
#endif /* RGBD_TRACKER_DATATYPES_HPP */

