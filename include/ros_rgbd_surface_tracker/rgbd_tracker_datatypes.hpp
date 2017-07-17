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

#include "ShapeGrammarLibrary.hpp"

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
            std::vector<AlgebraicSurfacePatch> patches;
            std::vector<sg::Shape::Ptr> parentShape;

        public:
            std::vector<cv::Vec3f> verts;
            std::vector<cv::Vec3f> normals;
            std::vector<cv::Vec3f> colors;

            void addPart(AlgebraicSurfacePatch& patch) {
                patches.push_back(patch);
            }
        }; /* class ObjectGeometry */

    } /* namespace rgbd */
} /* namespace cv */
#endif /* __cplusplus */
#endif /* RGBD_TRACKER_DATATYPES_HPP */

