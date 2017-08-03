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
        extern std::map<SurfaceType, std::vector<sg::Shape::Ptr>> shapeMap;
    } /* namespace rgbd */
} /* namespace cv */

// function to compute hash of the SurfaceType enum
namespace std {

    template <>
    struct hash<cv::rgbd::SurfaceType> {

        std::size_t operator()(const cv::rgbd::SurfaceType& t) const {
            // Compute individual hash values for first,
            // second and third and combine them using XOR
            // and bit shifting:
            return std::hash<int>()((int) t);
            //         ^ (hash<string>()(k.second) << 1)) >> 1)
            //         ^ (hash<int>()(k.third) << 1);
        }
    };
}

namespace cv {
    namespace rgbd {

        class ShapeMatch {
        public:
            cv::rgbd::SurfaceType surfaceType;
            sg::Shape::Ptr query_shape;
            sg::Shape::Ptr train_shape;
            float distance;

            ShapeMatch() {
            }

            ShapeMatch(sg::Shape::Ptr _queryS, sg::Shape::Ptr _trainS,
                    float _distance, SurfaceType _type) :
            query_shape(_queryS), train_shape(_trainS),
            distance(_distance), surfaceType(_type) {

            }

            virtual ~ShapeMatch() {
            }
        };

        class WorldMap {
        public:
            std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>> map;

            WorldMap() {

            }

            virtual ~WorldMap() {

            }

            void insert(std::vector<sg::Shape::Ptr>& newShapes, Pose global_pose);

            void update(const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                    const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                    const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& prev_quadTree,
                    const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&train_shapeMap,
                    std::vector<cv::rgbd::ShapeMatch>& matches, Pose global_pose);

        };

        class ObjectGeometry {
            //std::vector<AlgebraicSurfacePatch::Ptr> patchVec;
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

            //            void addPart(AlgebraicSurfacePatch::Ptr patch) {
            //                patchVec.push_back(patch);
            //                if (surfaceType != patch->getSurfaceType()) {
            //                    surfaceType = patch->getSurfaceType();
            //                }
            //                parentShape = patch->getShape();
            //                //std::cout << "patchShape " << patch.getShape()->toString() << std::endl;
            //                //std::cout << "parentShape " << parentShape->toString() << std::endl;
            //            }

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

            static ObjectGeometry::Ptr create() {
                return ObjectGeometry::Ptr(boost::make_shared<ObjectGeometry>());
            }
        }; /* class ObjectGeometry */

    } /* namespace rgbd */
} /* namespace cv */
#endif /* __cplusplus */
#endif /* RGBD_TRACKER_DATATYPES_HPP */

