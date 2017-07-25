/* 
 * File:   rgbd_tracker_datatypes.hpp
 * Author: arwillis
 *
 * Created on July 13, 2017, 1:36 PM
 */

#ifndef RGBD_TRACKER_DATATYPES_HPP
#define RGBD_TRACKER_DATATYPES_HPP
#ifdef __cplusplus

#include <map>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>

//namespace sg {
//    class Shape;
//    class Plane;
//    typedef boost::shared_ptr<Shape> ShapePtr;
//    typedef boost::shared_ptr<Shape> PlanePtr;
//}

namespace cv {
    namespace rgbd {

        enum SurfaceType {
            UNKNOWN = 0,
            PLANE,
            EDGE,
            CORNER,
            BOX
        };
        extern std::map<SurfaceType, const char*> surfaceTypeToString;

        class AlgebraicSurfacePatch {
            sg::Plane::Ptr plane_ptr;
            SurfaceType surfaceType;
            float avgDepth;
            //sg::Plane::Ptr shapePtr;
        public:
            typedef boost::shared_ptr<AlgebraicSurfacePatch> Ptr;

            AlgebraicSurfacePatch(cv::Plane3f _plane) :
            plane_ptr(new sg::Plane(_plane)),
            surfaceType(SurfaceType::PLANE) {
            }

            AlgebraicSurfacePatch(TesselatedPlane3f::Ptr _plane,
                    const cv::rgbd::RgbdImage& rgbdImg) :
            plane_ptr(new sg::Plane(*_plane)),
            surfaceType(SurfaceType::PLANE) {
                //shapePtr.reset(&plane);
                // create in-plane (u,v) coordinates for 3D points
                cv::Point3f pts[4];
                std::vector<RectWithError> errRect = _plane->getQuads();
                RectWithError& r0 = errRect[0];
                rgbdImg.getPoint3f(r0.x, r0.y, pts[0]);
                rgbdImg.getPoint3f(r0.x + r0.width, r0.y, pts[1]);
                rgbdImg.getPoint3f(r0.x + r0.width, r0.y + r0.height, pts[2]);
                rgbdImg.getPoint3f(r0.x, r0.y + r0.height, pts[3]);
                std::vector<cv::Vec2f> plane_uv_coords;
                avgDepth = 0.25 * (pts[0].z + pts[1].z + pts[2].z + pts[3].z);
                for (cv::Point3f pt : pts) {
                    cv::Point2f uv_coord = _plane->xyzToUV(pt);
                    //std::cout << "xyz = " << pt << "-> uv = " << uv_coord << std::endl;
                    plane_uv_coords.push_back(uv_coord);
                    cv::Point3f xyz3 = _plane->uvToXYZ(uv_coord);
                    //std::cout << "uv = " << uv_coord << "-> xyz = " << xyz3 << std::endl;
                }
                //sg::Plane& plane = *plane_ptr;
                plane_ptr->addCoords(plane_uv_coords);
                // create texture coordinates
                std::vector<cv::Vec2f> plane_uv_texcoords(4);
                plane_uv_texcoords[0] = cv::Vec2f(((float) r0.x) / rgbdImg.getWidth(),
                        1.0f - ((float) r0.y / (float) rgbdImg.getHeight()));
                plane_uv_texcoords[1] = cv::Vec2f(((float) r0.x + r0.width) / rgbdImg.getWidth(),
                        1.0f - ((float) r0.y / (float) rgbdImg.getHeight()));
                plane_uv_texcoords[2] = cv::Vec2f(((float) r0.x + r0.width) / rgbdImg.getWidth(),
                        1.0f - ((float) r0.y + r0.height) / (float) rgbdImg.getHeight());
                plane_uv_texcoords[3] = cv::Vec2f(((float) r0.x) / rgbdImg.getWidth(),
                        1.0f - ((float) r0.y + r0.height) / (float) rgbdImg.getHeight());
                plane_ptr->addTexCoords(plane_uv_texcoords);
                //std::cout << "shape1 " << shapePtr->toString() << std::endl;
                //std::cout << "shape2 " << getShape()->toString() << std::endl;
            }

            virtual ~AlgebraicSurfacePatch() {
            }

            void setPlane(cv::Plane3f _plane) {

            }

            SurfaceType getSurfaceType() {
                return surfaceType;
            }

            sg::Shape::Ptr getShape() {
                return plane_ptr;
            }

            float getAverageDepth() {
                return avgDepth;
            }

            static AlgebraicSurfacePatch::Ptr create(cv::Plane3f _plane) {
                return AlgebraicSurfacePatch::Ptr(boost::make_shared<AlgebraicSurfacePatch>(_plane));
            }

            static AlgebraicSurfacePatch::Ptr create(TesselatedPlane3f::Ptr _plane,
                    const cv::rgbd::RgbdImage& rgbdImg) {
                return AlgebraicSurfacePatch::Ptr(boost::make_shared<AlgebraicSurfacePatch>(_plane, rgbdImg));
            }
        }; /* class AlgebraicSurfacePatch */

        class ObjectGeometry {
            std::vector<AlgebraicSurfacePatch::Ptr> patchVec;
            sg::Shape::Ptr parentShape;
            SurfaceType surfaceType;
        public:
            std::vector<cv::Vec3f> verts;
            std::vector<cv::Vec3f> normals;
            std::vector<cv::Vec3f> colors;
            typedef boost::shared_ptr<ObjectGeometry> Ptr;

            ObjectGeometry() {
            }

            virtual ~ObjectGeometry() {
            }

            void addPart(AlgebraicSurfacePatch::Ptr patch) {
                patchVec.push_back(patch);
                if (surfaceType != patch->getSurfaceType()) {
                    surfaceType = patch->getSurfaceType();
                }
                parentShape = patch->getShape();
                //std::cout << "patchShape " << patch.getShape()->toString() << std::endl;
                //std::cout << "parentShape " << parentShape->toString() << std::endl;
            }

            void setShape(sg::Shape::Ptr _parentShape) {
                parentShape = _parentShape;
            }

            void setSurfaceType(SurfaceType _surfaceType) {
                surfaceType = _surfaceType;
            }

            SurfaceType getSurfaceType() {
                return surfaceType;
            }

            sg::Shape::Ptr getShape() {
                return parentShape;
            }

            std::vector<cv::Plane3f::Ptr> getPlanes() {
                std::vector<cv::Plane3f::Ptr> planeVec;
                for (AlgebraicSurfacePatch::Ptr patchPtr : patchVec) {
                    sg::Shape::Ptr shapePtr = patchPtr->getShape();
                    //std::cout << shapePtr->toString() << std::endl;
                    if (patchPtr->getSurfaceType() == SurfaceType::PLANE) {
                        sg::Plane::Ptr planePtr = boost::static_pointer_cast<sg::Plane>(shapePtr);
                        //std::cout << planePtr->toString() << std::endl;
                        planeVec.push_back(planePtr);
                    }
                }
                return planeVec;
            }

            static ObjectGeometry::Ptr create() {
                return ObjectGeometry::Ptr(boost::make_shared<ObjectGeometry>());
            }
        }; /* class ObjectGeometry */

    } /* namespace rgbd */
} /* namespace cv */
#endif /* __cplusplus */
#endif /* RGBD_TRACKER_DATATYPES_HPP */

