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

            std::string toString() {
                std::ostringstream stringStream;
                stringStream << "Match{A,B}: " << cv::rgbd::surfaceTypeToString[surfaceType]
                        << " distance = " << distance << " :"
                        << " ShapeA = " << query_shape->toString()
                        << " ShapeB = " << train_shape->toString() << std::endl;
                return stringStream.str();
            }
        };

        class ocvMat {
        protected:
            cv::Mat data;
        public:

            void setData(const cv::Mat& _data) {
                data = _data;
            }

            cv::Mat& getData() {
                return data;
            }
        };

        class CameraInfo : public ocvMat {
        public:
            typedef boost::shared_ptr<CameraInfo> Ptr;
            typedef boost::shared_ptr<const CameraInfo> ConstPtr;

            CameraInfo() {
            }

            virtual ~CameraInfo() {
            }

            static CameraInfo::Ptr create() {
                return CameraInfo::Ptr(boost::make_shared<CameraInfo>());
            }
        };

        class PlaneImage : public ocvMat {
            CameraInfo::Ptr cameraInfo;
        public:
            typedef boost::shared_ptr<PlaneImage> Ptr;
            typedef boost::shared_ptr<const PlaneImage> ConstPtr;

            PlaneImage() {
            }

            virtual ~PlaneImage() {
            }

            static PlaneImage::Ptr create() {
                return PlaneImage::Ptr(boost::make_shared<PlaneImage>());
            }

            cv::Mat visualizeAsImage() {
                cv::Mat abcd[4];
                cv::split(data, abcd);
                std::vector<cv::Mat> abc = {abcd[0], abcd[1], abcd[2]};
                cv::Mat normMat(abcd[0].rows, abcd[0].cols, CV_8UC3);
                cv::merge(abc, normMat);
                cv::Mat visImageU8C3;
                cv::normalize(normMat, visImageU8C3, 0, 255, cv::NORM_MINMAX, CV_8UC3);
                return visImageU8C3;
            }
        };

        class RGB8Image : public ocvMat {
            CameraInfo::Ptr cameraInfo;
        public:
            typedef boost::shared_ptr<RGB8Image> Ptr;
            typedef boost::shared_ptr<const RGB8Image> ConstPtr;

            RGB8Image() {
            }

            virtual ~RGB8Image() {
            }

            static RGB8Image::Ptr create() {
                return RGB8Image::Ptr(boost::make_shared<RGB8Image>());
            }
        };

        class DepthImage : public ocvMat {
            CameraInfo::Ptr cameraInfo;
        public:
            typedef boost::shared_ptr<DepthImage> Ptr;
            typedef boost::shared_ptr<const DepthImage> ConstPtr;

            DepthImage() {
            }

            virtual ~DepthImage() {
            }

            static DepthImage::Ptr create() {
                return DepthImage::Ptr(boost::make_shared<DepthImage>());
            }
        };

        class PlaneNetLabelImage : public ocvMat {
        public:
            typedef boost::shared_ptr<PlaneNetLabelImage> Ptr;
            typedef boost::shared_ptr<const PlaneNetLabelImage> ConstPtr;

            PlaneNetLabelImage() {
            }

            virtual ~PlaneNetLabelImage() {
            }

            static PlaneNetLabelImage::Ptr create() {
                return PlaneNetLabelImage::Ptr(boost::make_shared<PlaneNetLabelImage>());
            }
        };

        class ShapeMap : public std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>
        {
            public:
            typedef boost::shared_ptr<ShapeMap> Ptr;
            typedef boost::shared_ptr<const ShapeMap> ConstPtr;

            ShapeMap() {
            }

            virtual ~ShapeMap() {
            }

            void insert(std::vector<sg::Shape::Ptr>& newShapes, Pose global_pose);

            void update(const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                    const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                    const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& prev_quadTree,
                    const std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&train_shapeMap,
                    std::vector<cv::rgbd::ShapeMatch>& matches, Pose global_pose);

            static ShapeMap::Ptr create() {
                return ShapeMap::Ptr(boost::make_shared<ShapeMap>());
            }

        };

        class ObjectGeometry {
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

