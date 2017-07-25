/* 
 * File:   ShapeGrammarLibrary.hpp
 * Author: arwillis
 *
 * Created on July 11, 2017, 9:35 PM
 */

#ifndef SHAPEGRAMMARLIBRARY_HPP
#define SHAPEGRAMMARLIBRARY_HPP

#include <vector>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <GL/glut.h>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>

#ifdef __cplusplus

namespace cv {
    namespace rgbd {
        class ObjectGeometry;
        typedef boost::shared_ptr<ObjectGeometry> ObjectGeometryPtr;
    }
}

class Pose {
public:

    Pose() : rodrigues(0, 0, 0), position(0, 0, 0) {

    }

    Pose(cv::Vec3f _position, cv::Vec3f _rodrigues = cv::Vec3f(0, 0, 0)) {
        position = _position;
        rodrigues = _rodrigues;
    }

    virtual ~Pose() {
    }

    void rotateInPlace(cv::Vec3f& vec) {
        cv::Mat rotMat;
        cv::Rodrigues(rodrigues, rotMat);
        float *mm = rotMat.ptr<float>(0, 0);
        vec[0] = mm[0] * vec[0] + mm[1] * vec[1] + mm[2] * vec[2];
        vec[1] = mm[3] * vec[0] + mm[4] * vec[1] + mm[5] * vec[2];
        vec[2] = mm[6] * vec[0] + mm[7] * vec[1] + mm[8] * vec[2];
    }

    void transformInPlace(cv::Vec3f& pt) {
        cv::Mat rotMat;
        cv::Rodrigues(rodrigues, rotMat);
        //std::cout << "rodMat " << rotMat << std::endl;
        float *mm = rotMat.ptr<float>(0, 0);
        pt[0] = mm[0] * pt[0] + mm[1] * pt[1] + mm[2] * pt[2] + position[0];
        pt[1] = mm[3] * pt[0] + mm[4] * pt[1] + mm[5] * pt[2] + position[1];
        pt[2] = mm[6] * pt[0] + mm[7] * pt[1] + mm[8] * pt[2] + position[2];
    }

    cv::Vec3f getTranslation() {
        return position;
    }
private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    //Pose(const Pose& ref);
    //Pose& operator=(const Pose& ref);
    cv::Vec3f rodrigues;
    cv::Vec3f position;
};

// Global interface for shapes
namespace sg {

    class Shape {
    public:
        typedef boost::shared_ptr<Shape> Ptr;

        Shape() : pose() {
        }

        Shape(Pose _pose) : pose(_pose) {
        }

        virtual ~Shape() {
        }

        virtual std::vector<cv::Vec3f> generateCoords() = 0;
        virtual std::vector<int> generateCoordIndices() = 0;
        virtual std::vector<cv::Vec3f> generateNormals() = 0;
        virtual std::vector<int> generateNormalCoordIndices() = 0;
        virtual std::vector<cv::Vec3f> generateColorCoords() = 0;
        virtual std::vector<int> generateColorCoordIndices() = 0;
        virtual std::string toString() = 0;

    protected:
        Pose pose;
    };

    class Plane : public Shape, public cv::Plane3f {
        std::vector< std::vector<cv::Vec2f> > uv_coords; // 2d parametric coords within the plane
        std::vector< std::vector<cv::Vec2f> > uv_texCoords; // 2d parametric coords within the RGB image
    public:
        typedef boost::shared_ptr<Plane> Ptr;

        Plane() : sg::Shape(), cv::Plane3f() {
        }

        Plane(cv::Plane3f& _p) : sg::Shape(), cv::Plane3f(_p.x, _p.y, _p.z, _p.d) {
        }

        //Plane(cv::Vec3f ctr, cv::Vec2f dims, cv::Plane3f& _p) : cv::Plane3f(_p.x, _p.y, _p.z, _p.d) {
        //    // TODO construct uv coords   
        //}

        Plane(float a, float b, float c, float d) : sg::Shape(), cv::Plane3f(a, b, c, d) {
        }

        virtual ~Plane() {
        }

        void addCoords(std::vector<cv::Vec2f>& coordVec) {
            uv_coords.push_back(coordVec);
        }

        void addTexCoords(std::vector<cv::Vec2f>& texcoordVec) {
            uv_texCoords.push_back(texcoordVec);
        }

        std::vector<cv::Vec3f> generateCoords();

        std::vector<int> generateCoordIndices();

        std::vector<cv::Vec3f> generateNormals();

        std::vector<int> generateNormalCoordIndices();

        std::vector<cv::Vec3f> generateColorCoords();

        std::vector<int> generateColorCoordIndices();

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << cv::Plane3f::toString();
            return stringStream.str();
        }

        static Plane::Ptr create() {
            return Plane::Ptr(boost::make_shared<Plane>());
        }
    };

    class Edge : public Shape, public cv::LineSegment3f {
        Plane::Ptr surfaces[2]; // sorted by increasing z normal components
    public:
        typedef boost::shared_ptr<Edge> Ptr;

        Edge(Plane::Ptr planeA, Plane::Ptr planeB) : sg::Shape(), cv::LineSegment3f() {
            if (planeA->z < planeB->z) {
                surfaces[0] = planeA;
                surfaces[1] = planeB;
            } else {
                surfaces[0] = planeB;
                surfaces[1] = planeA;
            }
            surfaces[0]->intersect(*surfaces[1], *this);
            if (v.z < 0) {
                v = -v;
            }
            //float normf = 1.0f/std::sqrt(v.dot(v));
            //v *= normf;
            float lambda;
            float tstart[2], tend[2];
            tstart[0] = tstart[1] = -std::numeric_limits<float>::infinity();
            tend[0] = tend[1] = std::numeric_limits<float>::infinity();
            for (int surfIdx = 0; surfIdx < 2; ++surfIdx) {
                std::vector<cv::Vec3f> pts = surfaces[surfIdx]->generateCoords();
                for (cv::Vec3f pt : pts) {
                    lambda = xyzToLambda(pt);
                    if (lambda > tstart[surfIdx]) {
                        tstart[surfIdx] = lambda;
                    }
                    if (lambda < tend[surfIdx]) {
                        tend[surfIdx] = lambda;
                    }
                }
            }
            start = std::min(tstart[0], tstart[1]);
            end = std::min(tend[0], tend[1]);
        }

        virtual ~Edge() {
        }

        std::vector<cv::Vec3f> generateCoords();

        std::vector<int> generateCoordIndices();

        std::vector<cv::Vec3f> generateNormals();

        std::vector<int> generateNormalCoordIndices();

        std::vector<cv::Vec3f> generateColorCoords();

        std::vector<int> generateColorCoordIndices();

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << cv::LineSegment3f::toString();
            return stringStream.str();
        }

        static Edge::Ptr create(Plane::Ptr planeA, Plane::Ptr planeB) {
            return Edge::Ptr(boost::make_shared<Edge>(planeA, planeB));
        }
    };

    class Box : public Shape {
    public:
        typedef boost::shared_ptr<Box> Ptr;

        //        enum Faces {
        //            UNKNOWN = 0,
        //            FRONT,
        //            BACK,
        //            LEFT,
        //            RIGHT,
        //            TOP,
        //            BOTTOM
        //        };

        Box() : sg::Shape() {
        }

        Box(cv::Vec3f _dims, Pose _pose) : dims(_dims), sg::Shape(_pose) {
        }

        virtual ~Box() {
        }

        std::vector<cv::Vec3f> generateCoords();

        std::vector<int> generateCoordIndices();

        std::vector<cv::Vec3f> generateNormals();

        std::vector<int> generateNormalCoordIndices();

        std::vector<cv::Vec3f> generateColorCoords();

        std::vector<int> generateColorCoordIndices();

        std::vector<cv::rgbd::ObjectGeometryPtr> getCorners();

        std::vector<cv::rgbd::ObjectGeometryPtr> getEdges();

        std::vector<cv::rgbd::ObjectGeometryPtr> getPlanes();

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << "I AM A BOX";
            return stringStream.str();
        }

        static Box::Ptr create() {
            return Box::Ptr(boost::make_shared<Box>());
        }

    private:
        // -------------------------
        // Disabling default copy constructor and default
        // assignment operator.
        // -------------------------
        //Box(const Box& ref);
        //Box& operator=(const Box& ref);
        cv::Point3f dims; // length, width, height
    };

    class Cylinder : public Shape {
    public:
        typedef boost::shared_ptr<Cylinder> Ptr;

        Cylinder() : sg::Shape() {
        }

        Cylinder(float _radius, float _height, Pose _pose)
        : r(_radius), h(_height), sg::Shape(_pose) {
        }

        virtual ~Cylinder() {
        }

        std::vector<cv::Vec3f> generateCoords() {
            static int N = DEFAULT_RESOLUTION;
            return generateCoords(N);
        }

        std::vector<cv::Vec3f> generateCoords(int N);

        std::vector<int> generateCoordIndices() {
            static int N = DEFAULT_RESOLUTION;
            return generateCoordIndices(N);
        }

        std::vector<int> generateCoordIndices(int N);

        std::vector<cv::Vec3f> generateNormals();

        std::vector<int> generateNormalCoordIndices();

        std::vector<cv::Vec3f> generateColorCoords();

        std::vector<int> generateColorCoordIndices();

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << "I AM A CYLINDER";
            return stringStream.str();
        }

        static Cylinder::Ptr create() {
            return Cylinder::Ptr(boost::make_shared<Cylinder>());
        }
    private:
        // -------------------------
        // Disabling default copy constructor and default
        // assignment operator.
        // -------------------------
        //Cylinder(const Cylinder& ref);
        //Cylinder& operator=(const Cylinder& ref);
        static constexpr float DEFAULT_RESOLUTION = 16;
        float r, h; // length, width, height
    };
} /* namespace sg */
#endif /* __cplusplus */

#endif /* SHAPEGRAMMARLIBRARY_HPP */

