/* 
 * File:   ShapeGrammarLibrary.hpp
 * Author: arwillis
 *
 * Created on July 11, 2017, 9:35 PM
 */

#ifndef SHAPEGRAMMARLIBRARY_HPP
#define SHAPEGRAMMARLIBRARY_HPP

#include <vector>
#include <map>

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

    enum CornerType {
        UNKNOWN = -1,
        BACK_BOTTOM_RIGHT,
        BACK_BOTTOM_LEFT,
        BACK_TOP_RIGHT,
        BACK_TOP_LEFT,
        FRONT_BOTTOM_RIGHT,
        FRONT_BOTTOM_LEFT,
        FRONT_TOP_RIGHT,
        FRONT_TOP_LEFT
    };

    extern std::map<CornerType, const char*> cornerTypeToString;

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

    template <typename _Tpl>
    class Plane : public Shape, public cv::Plane3_<_Tpl> {
        std::vector< std::vector< cv::Vec<_Tpl, 2> > > uv_coords; // 2d parametric coords within the plane
        std::vector< std::vector< cv::Vec<_Tpl, 2> > > uv_texCoords; // 2d parametric coords within the RGB image
    public:
        typedef boost::shared_ptr<Plane> Ptr;

        Plane() : sg::Shape(), cv::Plane3_<_Tpl>() {
        }

        Plane(cv::Plane3_<_Tpl>& _p) : sg::Shape(), cv::Plane3_<_Tpl>(_p.x, _p.y, _p.z, _p.d) {
        }

        //Plane(cv::Vec3f ctr, cv::Vec2f dims, cv::Plane3f& _p) : cv::Plane3f(_p.x, _p.y, _p.z, _p.d) {
        //    // TODO construct uv coords   
        //}

        Plane(_Tpl a, _Tpl b, _Tpl c, _Tpl d) : sg::Shape(), cv::Plane3_<_Tpl>(a, b, c, d) {
        }

        virtual ~Plane() {
        }

        void addCoords(std::vector<cv::Vec<_Tpl, 2 >> &coordVec) {
            uv_coords.push_back(coordVec);
        }

        void addTexCoords(std::vector<cv::Vec<_Tpl, 2 >> &texcoordVec) {
            uv_texCoords.push_back(texcoordVec);
        }

        std::vector<cv::Vec3f> generateCoords() {
            std::vector<cv::Vec<_Tpl, 2 >> uv_poly_coords = uv_coords[0];
            std::vector<cv::Vec3f> pts(uv_poly_coords.size() + 1);
            cv::Vec3f ctr_pt(0, 0, 0);
            for (int ptidx = 0; ptidx < uv_poly_coords.size(); ++ptidx) {
                pts[ptidx] = this->uvToXYZ(uv_poly_coords[ptidx]);
                ctr_pt += pts[ptidx];
            }
            ctr_pt *= 1.0f / uv_poly_coords.size();
            pts[uv_poly_coords.size()] = ctr_pt;
            for (int idx = 0; idx < pts.size(); ++idx) {
                pose.transformInPlace(pts[idx]);
            }
            return pts;
        }

        std::vector<int> generateCoordIndices() {
            std::vector<cv::Vec<_Tpl, 2 >> uv_poly_coords = uv_coords[0];
            std::vector<int> ptidxs; // reserve(uv_poly_coords.size()*3)
            int ctr_pt_idx = uv_poly_coords.size();
            for (int triIdx = 0; triIdx < uv_poly_coords.size() - 1; ++triIdx) {
                ptidxs.insert(ptidxs.end(),{ctr_pt_idx, triIdx, triIdx + 1});
            }
            ptidxs.insert(ptidxs.end(),{ctr_pt_idx, (int) uv_poly_coords.size() - 1, 0});
            return ptidxs;
        }

        std::vector<cv::Vec3f> generateNormals() {
            std::vector<cv::Vec3f> norms = {cv::Vec3f(this->x, this->y, this->z)};
            for (int idx = 0; idx < norms.size(); ++idx) {
                pose.rotateInPlace(norms[idx]);
            }
            return norms;
        }

        std::vector<int> generateNormalCoordIndices() {
            std::vector<cv::Vec<_Tpl, 2 >> uv_poly_coords = uv_coords[0];
            std::vector<int> normidxs; // reserve(uv_poly_coords.size()*3);
            for (int triIdx = 0; triIdx < uv_poly_coords.size(); ++triIdx) {
                normidxs.insert(normidxs.end(),{0, 0, 0});
            }
            return normidxs;
        }

        std::vector<cv::Vec3f> generateColorCoords() {
            std::vector<cv::Vec3f> colors(1);
            cv::Mat hsv(1, 1, CV_32FC3, cv::Scalar(this->x, this->y, 0.7));
            cv::Mat rgb(1, 1, CV_32FC3);
            cv::cvtColor(hsv, rgb, CV_HSV2BGR);
            //colors[0] = cv::Vec3f(1.0f, 0.0f, 0.0f);
            colors[0] = hsv.at<cv::Vec3f>(0, 0);
            return colors;
        }

        std::vector<int> generateColorCoordIndices() {
            std::vector<cv::Vec<_Tpl, 2 >> uv_poly_coords = uv_coords[0];
            std::vector<int> coloridxs; // reserve(uv_poly_coords.size()*3);
            for (int triIdx = 0; triIdx < uv_poly_coords.size(); ++triIdx) {
                coloridxs.insert(coloridxs.end(),{0, 0, 0});
            }
            return coloridxs;
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << cv::Plane3_<_Tpl>::toString();
            return stringStream.str();
        }

        static Plane<_Tpl>::Ptr create() {
            return Plane<_Tpl>::Ptr(boost::make_shared<Plane < _Tpl >> ());
        }
    };

    template <typename _Tpl>
    class Edge : public Shape, public cv::LineSegment3_<_Tpl> {
        //class Edge : public Shape, public cv::LineSegment3_<float> {
        Plane<float>::Ptr surfaces[2]; // sorted by increasing z normal components
    public:
        typedef boost::shared_ptr<Edge> Ptr;

        Edge(Plane<float>::Ptr planeA, Plane<float>::Ptr planeB) : sg::Shape(), cv::LineSegment3_<_Tpl>() {
            if (planeA->z < planeB->z) {
                surfaces[0] = planeA;
                surfaces[1] = planeB;
            } else {
                surfaces[0] = planeB;
                surfaces[1] = planeA;
            }
            surfaces[0]->intersect(*surfaces[1], *this);
            if (this->v.z < 0) {
                this->v = -this->v;
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
                    //cv::Vec<_Tpl, 3> cvec(pt[0], pt[1], pt[2]);
                    lambda = this->xyzToLambda(pt);
                    if (lambda > tstart[surfIdx]) {
                        tstart[surfIdx] = lambda;
                    }
                    if (lambda < tend[surfIdx]) {
                        tend[surfIdx] = lambda;
                    }
                }
            }
            this->start = std::min(tstart[0], tstart[1]);
            this->end = std::min(tend[0], tend[1]);
        }

        virtual ~Edge() {
        }

        std::vector<cv::Vec3f> generateCoords() {
            std::vector<cv::Vec3f> pts = {
                this->getPoint(this->start),
                this->getPoint(this->end)
                //getPoint(tstart[0]),
                //getPoint(tend[0]),
                //getPoint(tstart[1]),
                //getPoint(tend[1])
            };
            return pts;
        }

        std::vector<int> generateCoordIndices() {
            std::vector<int> ptIdxs = {0, 1}; //, 2, 3};
            return ptIdxs;
        }

        std::vector<cv::Vec3f> generateNormals() {
            std::vector<cv::Vec3f> norms = {cv::Vec3f(0, 0, -1)};
            return norms;
        }

        std::vector<int> generateNormalCoordIndices() {
            std::vector<int> normIdxs = {0, 0}; //, 0, 0};
            return normIdxs;
        }

        std::vector<cv::Vec3f> generateColorCoords() {
            std::vector<cv::Vec3f> colors = {cv::Vec3f(1, 0, 0)}; //, cv::Vec3f(0, 1, 0)};
            return colors;
        }

        std::vector<int> generateColorCoordIndices() {
            std::vector<int> colorIdxs = {0, 0}; //, 1, 1};
            return colorIdxs;
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << cv::LineSegment3_<_Tpl>::toString();
            return stringStream.str();
        }

        static Edge<_Tpl>::Ptr create(Plane<float>::Ptr planeA, Plane<float>::Ptr planeB) {
            return Edge<_Tpl>::Ptr(boost::make_shared<Edge < _Tpl >> (planeA, planeB));
        }
        //static Edge::Ptr create(Plane<float>::Ptr planeA, Plane<float>::Ptr planeB) {
        //    return Edge::Ptr(boost::make_shared<Edge> (planeA, planeB));
        //}
    };

    template <typename _Tpl>
    class Corner : public Shape, public cv::Point3_<_Tpl> {
        Edge<float>::Ptr edges[3]; // sorted by increasing z normal components        
        //  parametric location of corner 
        // on each edge line pair, (0,1), (1,2), (0,2)
        _Tpl eline_lambdas[3];
    public:
        typedef boost::shared_ptr<Corner> Ptr;

        Corner(Edge<float>::Ptr edgeA, Edge<float>::Ptr edgeB, Edge<float>::Ptr edgeC) :
        sg::Shape(), cv::Point3_<_Tpl>() {
            edges[0] = edgeA;
            edges[1] = edgeB;
            edges[2] = edgeC;
            // solve for corner location and set (cv::Point3f) this to the corner position
            edges[0]->intersect(*edges[1], *this);
            // get lambda for plane  pair (0,2)
            eline_lambdas[0] = edges[0]->xyzToLambda(*this);
            eline_lambdas[1] = edges[1]->xyzToLambda(*this);
            eline_lambdas[2] = edges[2]->xyzToLambda(*this);
        }

        virtual ~Corner() {
        }

        std::vector<cv::Vec3f> generateCoords() {
            std::vector<cv::Vec3f> pts = {
                edges[0]->getPoint(edges[0]->end),
                edges[0]->getPoint(eline_lambdas[0]),
                edges[1]->getPoint(edges[1]->end),
                edges[1]->getPoint(eline_lambdas[1]),
                edges[2]->getPoint(edges[2]->end),
                edges[2]->getPoint(eline_lambdas[2])
            };
            return pts;
        }

        std::vector<int> generateCoordIndices() {
            std::vector<int> ptIdxs = {0, 1, 2, 3, 4, 5};
            return ptIdxs;
        }

        std::vector<cv::Vec3f> generateNormals() {
            std::vector<cv::Vec3f> norms = {cv::Vec3f(0, 0, -1)};
            return norms;
        }

        std::vector<int> generateNormalCoordIndices() {
            std::vector<int> normIdxs = {0, 0, 0, 0, 0, 0};
            return normIdxs;
        }

        std::vector<cv::Vec3f> generateColorCoords() {
            std::vector<cv::Vec3f> colors = {
                cv::Vec3f(0, 1, 0),
                cv::Vec3f(0, 1, 0),
                cv::Vec3f(0, 1, 0)
            };
            return colors;
        }

        std::vector<int> generateColorCoordIndices() {
            std::vector<int> colorIdxs = {0, 0, 1, 1, 2, 2};
            return colorIdxs;
        }

        bool isConvex() {
            cv::Point3_<_Tpl> cornerPt = *this;
            cv::Point3_<_Tpl> meanOfNeighbors = edges[0]->getPoint(edges[0]->end);
            meanOfNeighbors += edges[1]->getPoint(edges[1]->end);
            meanOfNeighbors += edges[2]->getPoint(edges[2]->end);
            meanOfNeighbors *= 1.0 / 3.0;
            return (meanOfNeighbors.z - cornerPt.z) > 0;
        }

        void getXYZAxes(std::vector< cv::Vec<_Tpl, 3 >> &axes) {
            cv::Point3_<_Tpl> cornerPt = *this;
            for (int idx = 0; idx < 3; ++idx) {
                axes[idx] = edges[idx]->getPoint(edges[idx]->end) - cornerPt;
                axes[idx] *= 1.0 / std::sqrt(axes[idx].dot(axes[idx]));
            }
            std::sort(axes.begin(), axes.end(),
                    [](const cv::Vec<_Tpl, 3>& a, const cv::Vec<_Tpl, 3>& b) {
                        return std::abs(a[0]) > std::abs(b[0]);
                    });
            if (std::abs(axes[1][1]) < std::abs(axes[2][1]))
                std::swap(axes[1], axes[2]);
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << this;
            return stringStream.str();
        }

        static Corner::Ptr create(Edge<float>::Ptr edgeA, Edge<float>::Ptr edgeB, Edge<float>::Ptr edgeC) {
            return Corner::Ptr(boost::make_shared<Corner>(edgeA, edgeB, edgeC));
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

        static sg::CornerType getCameraFrameCornerType(Corner<float>::Ptr corner);

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

