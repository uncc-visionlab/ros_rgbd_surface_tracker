/* 
 * File:   rgbd_tracker_datatypes.hpp
 * Author: arwillis
 *
 * Created on July 13, 2017, 1:36 PM
 */

#ifndef RGBD_TRACKER_DATATYPES_HPP
#define RGBD_TRACKER_DATATYPES_HPP
#ifdef __cplusplus

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>

namespace cv {
    namespace rgbd {

        class AlgebraicSurfacePatch {
            TesselatedPlane3f::Ptr planeptr;
            int surfaceType;
        public:

            enum Type {
                PLANE,
                EDGE,
                CORNER,
                BOX
            };

            AlgebraicSurfacePatch(TesselatedPlane3f::Ptr _plane) : planeptr(_plane),
            surfaceType(AlgebraicSurfacePatch::PLANE) {
            }
        }; /* class AlgebraicSurfacePatch */

        class ObjectGeometry {
        public:
            std::vector<cv::Vec3f> verts;
            std::vector<cv::Vec3f> normals;
            std::vector<cv::Vec3f> colors;
        }; /* class ObjectGeometry */

    } /* namespace rgbd */
} /* namespace cv */
#endif /* __cplusplus */
#endif /* RGBD_TRACKER_DATATYPES_HPP */

