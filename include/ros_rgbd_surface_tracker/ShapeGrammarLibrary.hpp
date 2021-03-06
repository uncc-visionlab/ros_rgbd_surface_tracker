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
#include <unordered_set>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <GL/glut.h>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>
#include <rgbd_odometry/pose.hpp>

#ifdef __cplusplus

//namespace cv {
//    namespace rgbd {
//        class ObjectGeometry;
//        typedef boost::shared_ptr<ObjectGeometry> ObjectGeometryPtr;
//    }
//}

// needed to use boost::shared_ptr elements as keys for std::unordered_set, std::unordered_map
namespace std {
    template<class T>
    struct hash<boost::shared_ptr<T>>
    {
        public:

        size_t operator()(const boost::shared_ptr<T>& key) const {
            return (size_t) key.get();
        }
    };
}

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
        virtual float matchDistance(Shape::Ptr qShape) = 0;

        virtual void serializeParameters(std::vector<float>& params) {
            std::cout << "serializeParameters() not implemented!" << std::endl;
        };

        virtual void serializeBoundary(std::vector<float>& boundary) {
            std::cout << "serializeBoundary() not implemented!" << std::endl;
        };

        virtual void setBoundary(std::vector<cv::Vec2f>& boundary) {
            std::cout << "setBoundary() not implemented!" << std::endl;
        };

        virtual std::vector<cv::Vec2f> getBoundary() const {
            std::cout << "getBoundary() not implemented!" << std::endl;
            return std::vector<cv::Vec2f>();
        };

        void setPose(Pose _pose) {
            pose = _pose;
        }

        Pose getPose() {
            return pose;
        }
    protected:
        Pose pose;
    };

    template <typename _Tpl>
    //class Plane : public Shape, public cv::Plane3_<_Tpl> {
    class Plane : public Shape, public cv::TesselatedPlane3_<_Tpl> {
        std::vector< std::vector< cv::Vec<_Tpl, 2> > > uv_coords; // 2d parametric coords within the plane
        std::vector< std::vector< cv::Vec<_Tpl, 2> > > uv_texCoords; // 2d parametric coords within the RGB image
    public:
        typedef boost::shared_ptr<Plane> Ptr;
        typedef boost::shared_ptr<const Plane> ConstPtr;

        Plane() : sg::Shape(), cv::TesselatedPlane3_<_Tpl>() {
        }

        Plane(cv::TesselatedPlane3_<_Tpl>& _p) : sg::Shape(), cv::TesselatedPlane3_<_Tpl>(_p) {
        }

        Plane(_Tpl a, _Tpl b, _Tpl c, _Tpl d) : sg::Shape(), cv::TesselatedPlane3_<_Tpl>(a, b, c, d) {
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
            //for (int idx = 0; idx < pts.size(); ++idx) {
            //    pose.transformInPlace(pts[idx]);
            //}
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
            //for (int idx = 0; idx < norms.size(); ++idx) {
            //    pose.rotateInPlace(norms[idx]);
            //}
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

        float matchDistance(Shape::Ptr qShape) {
            sg::Plane<_Tpl>::Ptr planePtr = boost::dynamic_pointer_cast<sg::Plane < _Tpl >> (qShape);
            if (!planePtr) {
                return std::numeric_limits<_Tpl>::infinity();
            }
            cv::Vec<_Tpl, 3> errorVec(planePtr->x - this->x,
                    planePtr->y - this->y,
                    planePtr->z - this->z);
            //cv::Vec<_Tpl, 4> errorVec(planePtr->x - this->x,
            //        planePtr->y - this->y,
            //        planePtr->z - this->z,
            //        planePtr->d - this->d);
            return std::sqrt(errorVec.dot(errorVec));
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << cv::Plane3_<_Tpl>::toString();
            return stringStream.str();
        }

        static Plane<_Tpl>::Ptr create() {
            return Plane<_Tpl>::Ptr(boost::make_shared<Plane < _Tpl >> ());
        }

        void serializeParameters(std::vector<_Tpl>& params) {
            params.push_back(this->x);
            params.push_back(this->y);
            params.push_back(this->z);
            params.push_back(this->d);
            params.push_back(this->u[0]);
            params.push_back(this->u[1]);
            params.push_back(this->u[2]);
            params.push_back(this->v[0]);
            params.push_back(this->v[1]);
            params.push_back(this->v[2]);
        }

        void deserializeParameters(const std::vector<_Tpl>& params) {
            this->x = params[0];
            this->y = params[1];
            this->z = params[2];
            this->d = params[3];
            this->u[0] = params[4];
            this->u[1] = params[5];
            this->u[2] = params[6];
            this->v[0] = params[7];
            this->v[1] = params[8];
            this->v[2] = params[9];
        }

        void serializeBoundary(std::vector<_Tpl>& boundary) {
            std::vector<cv::Vec<_Tpl, 2 >> uv_poly_coords = uv_coords[0];
            for (cv::Vec<_Tpl, 2 > uv_coord : uv_poly_coords) {
                boundary.push_back(uv_coord[0]);
                boundary.push_back(uv_coord[1]);
            }
        }

        void deserializeBoundary(const std::vector<_Tpl>& boundary) {
            std::vector<cv::Vec<_Tpl, 2 >> uv_poly_coords;
            for (int ptIdx = 0; ptIdx < boundary.size(); ptIdx += 2) {
                cv::Vec<_Tpl, 2> uv_coord(boundary[ptIdx], boundary[ptIdx + 1]);
                uv_poly_coords.push_back(uv_coord);
            }
            addCoords(uv_poly_coords);
        }

        std::vector<cv::Vec<_Tpl, 2 >> getBoundary() const {
            std::vector<cv::Vec<_Tpl, 2 >> uv_poly_coords;
            return (uv_coords.size() > 0) ? uv_coords[0] : uv_poly_coords;
        }

        void setBoundary(std::vector<cv::Vec<_Tpl, 2 >> &boundary) {
            if (uv_coords.size() == 0) {
                uv_coords.push_back(boundary);
            } else {
                uv_coords[0] = boundary;
            }
        }
    };

    template <typename _Tpl>
    class Edge : public Shape, public cv::LineSegment3_<_Tpl> {
        Plane<float>::Ptr surfaces[2]; // sorted by increasing z normal components
        bool _isReal;
        bool _isConvex;
    public:
        typedef boost::shared_ptr<Edge> Ptr;
        typedef boost::shared_ptr<const Edge> ConstPtr;

        Edge(Plane<float>::Ptr planeA, Plane<float>::Ptr planeB) : sg::Shape(),
        cv::LineSegment3_<_Tpl>(), _isReal(false), _isConvex(false) {
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
            // Note: the intersect method uses the cross product of plane normals
            // and these two vectors should be unit vectors making v, the edge direction, 
            // a unit vector hence, no normalization is needed unless planes are not in 
            // Hessian normals form.
            // However, for single precision unit vectors the cross product  
            // produces a vector whose norm is around 95% of unit (5% error!)
            this->v *= 1.0f / std::sqrt(this->v.dot(this->v));
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
            cv::Vec3f midpt = 0.5 * (this->getPoint(this->start) + this->getPoint(this->end));
            this->setPose(Pose(midpt, this->v));
            this->setIsConvexFlag();
        }

        virtual ~Edge() {
        }

        std::vector<cv::Vec3f> generateCoords() {
            cv::Vec3f midpoint;
            this->getPose().getTranslation(midpoint);
            cv::Vec3f posArr[2];
            this->surfaces[0]->getPose().getTranslation(posArr[0]);
            this->surfaces[1]->getPose().getTranslation(posArr[1]);
            std::vector<cv::Vec3f> pts = {
                this->getPoint(this->start),
                this->getPoint(this->end),
                //midpoint,
                //posArr[0],
                //midpoint,
                //posArr[1],
                //getPoint(tstart[0]),
                //getPoint(tend[0]),
                //getPoint(tstart[1]),
                //getPoint(tend[1])
            };
            //for (int idx = 0; idx < pts.size(); ++idx) {
            //    pose.transformInPlace(pts[idx]);
            //}            
            return pts;
        }

        std::vector<int> generateCoordIndices() {
            std::vector<int> ptIdxs = {0, 1}; //, 2, 3, 4, 5};
            return ptIdxs;
        }

        std::vector<cv::Vec3f> generateNormals() {
            std::vector<cv::Vec3f> norms = {cv::Vec3f(0, 0, -1)};
            //for (int idx = 0; idx < norms.size(); ++idx) {
            //    pose.rotateInPlace(norms[idx]);
            //}            
            return norms;
        }

        std::vector<int> generateNormalCoordIndices() {
            std::vector<int> normIdxs = {0, 0}; //, 0, 0, 0, 0}; //, 0, 0};
            return normIdxs;
        }

        std::vector<cv::Vec3f> generateColorCoords() {
            static cv::Vec3f red(1.0f, 0.0f, 0.0f);
            static cv::Vec3f orange(1.0f, 0.5f, 0.0f);
            if (_isReal) {
                return std::vector<cv::Vec3f>{red};
            }
            return std::vector<cv::Vec3f>{orange};
            //std::vector<cv::Vec3f> colors = {cv::Vec3f(1, 0, 0)}; //, cv::Vec3f(0, 1, 0)};
            //return colors;
        }

        std::vector<int> generateColorCoordIndices() {
            std::vector<int> colorIdxs = {0, 0}; //, 0, 0, 0, 0}; //, 1, 1};
            return colorIdxs;
        }

        bool equal(Edge<_Tpl>::Ptr queryEdge) {
            return (surfaces[0] == queryEdge->surfaces[0] && surfaces[1] == queryEdge->surfaces[1]) ||
                    (surfaces[1] == queryEdge->surfaces[0] && surfaces[0] == queryEdge->surfaces[1]);
        }

        bool sharesPlane(Edge<_Tpl>::Ptr queryEdge) {
            return (surfaces[0] == queryEdge->surfaces[0] || surfaces[1] == queryEdge->surfaces[1] ||
                    surfaces[1] == queryEdge->surfaces[0] || surfaces[0] == queryEdge->surfaces[1]);
        }

        bool contains(Plane<float>::Ptr queryPlane) {
            return (surfaces[0] == queryPlane || surfaces[1] == queryPlane);
        }

        void getPlanes(std::unordered_set<Plane<float>::Ptr>& planeSet) {
            planeSet.insert(surfaces[0]);
            planeSet.insert(surfaces[1]);
        }

        float matchPlanes(Edge<_Tpl>::Ptr edgePtr, std::pair<Plane<float>::Ptr, Plane<float>::Ptr>* planePairs) {
            float distance_A = surfaces[0]->matchDistance(edgePtr->surfaces[0])
                    + surfaces[1]->matchDistance(edgePtr->surfaces[1]);
            float distance_B = surfaces[0]->matchDistance(edgePtr->surfaces[1])
                    + surfaces[1]->matchDistance(edgePtr->surfaces[0]);
            if (distance_A < distance_B) {
                planePairs[0] = {surfaces[0], edgePtr->surfaces[0]};
                planePairs[1] = {surfaces[1], edgePtr->surfaces[1]};
                return distance_A;
            }
            planePairs[0] = {surfaces[0], edgePtr->surfaces[1]};
            planePairs[1] = {surfaces[1], edgePtr->surfaces[0]};
            return distance_B;
        }

        bool isConvex() {
            return _isConvex;
        }

        bool setIsConvexFlag() {
            cv::Vec<_Tpl, 3> meanOfNeighbors;
            this->surfaces[0]->getPose().getTranslation(meanOfNeighbors);
            cv::Vec<_Tpl, 3> planeBPt;
            this->surfaces[1]->getPose().getTranslation(planeBPt);
            meanOfNeighbors += planeBPt;
            meanOfNeighbors *= 0.5;
            cv::Vec<_Tpl, 3> edgePt;
            this->getPose().getTranslation(edgePt);
            _isConvex = (meanOfNeighbors[2] - edgePt[2]) > 0;
            return _isConvex;
        }

        bool isReal() const {
            return _isReal;
        }

        bool setReal(bool realFlag) {
            _isReal = realFlag;
            return true;
        }

        bool isInside(const std::vector<cv::Point3f>& pts) {
            static _Tpl ISREAL_THRESHOLD = 0.005;

            int numPts = pts.size();
            int convexSignFlip = isConvex() ? 1 : -1;
            _Tpl signed_distance;
            _Tpl max_signed_distance;
            _Tpl avg_max_signed_distance = 0;
            for (cv::Vec<_Tpl, 3> pt : pts) {
                max_signed_distance = -std::numeric_limits<_Tpl>::infinity();
                for (int idx = 0; idx < 2; ++idx) {
                    signed_distance = convexSignFlip * surfaces[idx]->evaluate(pt);
                    if (signed_distance > max_signed_distance) {
                        max_signed_distance = signed_distance;
                    }
                }
                avg_max_signed_distance += max_signed_distance;
            }
            avg_max_signed_distance /= numPts;
            return avg_max_signed_distance > 0;
        }

        cv::Matx<_Tpl, 3, 3> getNonOrthogonalCoordinateSystem() {
            cv::Vec<_Tpl, 3> featurePosition;
            this->getPose().getTranslation(featurePosition);
            cv::Matx<_Tpl, 3, 3> coordVecs = cv::Matx<_Tpl, 3, 3>::eye();
            cv::Vec<_Tpl, 3> eVec[2];
            cv::Vec<_Tpl, 3> eVec_perp[2];
            for (int idx = 0; idx < 2; ++idx) {
                surfaces[idx]->getPose().getTranslation(eVec[idx]);
                eVec[idx] = eVec[idx] - featurePosition;
                //std::cout << "eVec = " << eVec[idx] << " proj = " << eVec[idx].dot(this->v) << std::endl;                
            }
            // remove projection on the line direction (v) and half the projection onto the vector to the other plane
            // assumes equal noise in eVec[0] and eVec[1]
            eVec_perp[0] = eVec[0] - 0.5 * eVec[0].dot(eVec[1]) * eVec[1]
                    - eVec[0].dot(this->v) * (cv::Vec3f) this->v;
            eVec_perp[1] = eVec[0].cross(this->v);
            if (eVec_perp[1].dot(eVec[1]) < 0) {
                eVec_perp[1] *= -1;
            }
            for (int idx = 0; idx < 2; ++idx) {
                eVec_perp[idx] *= 1.0 / std::sqrt(eVec_perp[idx].dot(eVec_perp[idx]));
                coordVecs(idx, 0) = eVec_perp[idx][0];
                coordVecs(idx, 1) = eVec_perp[idx][1];
                coordVecs(idx, 2) = eVec_perp[idx][2];
            }
            coordVecs(2, 0) = this->v.x;
            coordVecs(2, 1) = this->v.y;
            coordVecs(2, 2) = this->v.z;
            //std::cout << "coordVecs = " << coordVecs << std::endl;
            return coordVecs;
        }

        float matchDistance(Shape::Ptr qShape) {
            sg::Edge<_Tpl>::Ptr edgePtr = boost::dynamic_pointer_cast<sg::Edge < _Tpl >> (qShape);
            if (!edgePtr) {
                return std::numeric_limits<_Tpl>::infinity();
            }
            cv::Vec<_Tpl, 3> errorVec(edgePtr->v.x - this->v.x,
                    edgePtr->v.y - this->v.y,
                    edgePtr->v.z - this->v.z);
            //cv::Vec<_Tpl, 3> delta_start = edgePtr->getPoint(edgePtr->start) - this->getPoint(this->start);
            //cv::Vec<_Tpl, 3> delta_end  = edgePtr->getPoint(edgePtr->end) - this->getPoint(this->end);
            //cv::Vec<_Tpl, 3> errorVec = delta_start + delta_end;
            //cv::Vec<_Tpl, 6> errorVec(edgePtr->v.x - this->v.x,
            //        edgePtr->v.y - this->v.y,
            //        edgePtr->v.z - this->v.z,
            //        edgePtr->p0.x - this->p0.x,
            //        edgePtr->p0.y - this->p0.y,
            //        edgePtr->p0.z - this->p0.z);
            return std::sqrt(errorVec.dot(errorVec));
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << cv::LineSegment3_<_Tpl>::toString();
            stringStream << " " << surfaces[0].get() << " " << surfaces[1].get();
            return stringStream.str();
        }

        static Edge<_Tpl>::Ptr create(Plane<float>::Ptr planeA, Plane<float>::Ptr planeB) {
            return Edge<_Tpl>::Ptr(boost::make_shared<Edge < _Tpl >> (planeA, planeB));
        }
    };

    template <typename _Tpl>
    class Corner : public Shape, public cv::Point3_<_Tpl> {
        Edge<float>::Ptr edges[3]; // sorted by increasing z normal components        
        //  parametric location of corner 
        // on each edge line pair, (0,1), (1,2), (0,2)
        _Tpl eline_lambdas[3];
        bool _isReal;
        bool _isConvex;
    public:
        typedef boost::shared_ptr<Corner> Ptr;
        typedef boost::shared_ptr<const Corner> ConstPtr;

        Corner(Edge<float>::Ptr edgeA, Edge<float>::Ptr edgeB, Edge<float>::Ptr edgeC) :
        sg::Shape(), cv::Point3_<_Tpl>(), _isReal(false), _isConvex(false) {
            edges[0] = edgeA;
            edges[1] = edgeB;
            edges[2] = edgeC;

            // map the three corner axes to the closest camera frame axes
            sg::CornerType id = getCameraFrameCornerType();
            static bool verbose = false;
            if (verbose) {
                std::cout << "CornerType = " << sg::cornerTypeToString[id] << std::endl;
            }

            // solve for corner location and set (cv::Point3f) this to the corner position
            edges[0]->intersect(*edges[1], *this);
            // get lambda for plane  pairs (0,1), (1,2), (0,2)
            eline_lambdas[0] = edges[0]->xyzToLambda(*this);
            eline_lambdas[1] = edges[1]->xyzToLambda(*this);
            eline_lambdas[2] = edges[2]->xyzToLambda(*this);
            if (verbose) {
                std::string shapeStr = isConvex() ? "convex" : "concave";
                std::cout << "Corner is " << shapeStr << std::endl;
            }
            // set pose translation = corner point
            // orientation = 
            //            cv::Mat cvTransform(4, 4, CV_32FC1);
            //            //directly use the buffer allocated by OpenCV
            //            Eigen::Map<Eigen::Matrix4f> eigenTransformMap(cvTransform.ptr<float>(0, 0));
            //            Eigen::Matrix4f eigenTransform = Eigen::Matrix4f::Zero(4, 4);
            //            std::vector<cv::Plane3f::Ptr> moving_planes(3);
            //            std::vector<cv::Plane3f::Ptr> fixed_planes(3);
            //            //moving_planes[0] = boost::make_shared<cv::Plane3f>(0, 0, 1, -0.0);
            //            //moving_planes[1] = boost::make_shared<cv::Plane3f>(-1, 0, 0, -0.0);
            //            //moving_planes[2] = boost::make_shared<cv::Plane3f>(0, 1, 0, -0.0);
            //            moving_planes[0] = boost::make_shared<cv::Plane3f>(1, 0, 0, -0.0);
            //            moving_planes[1] = boost::make_shared<cv::Plane3f>(0, 1, 0, -0.0);
            //            moving_planes[2] = boost::make_shared<cv::Plane3f>(0, 0, 1, -0.0);
            //            fixed_planes[0] = planeA;
            //            fixed_planes[1] = planeB;
            //            fixed_planes[2] = planeC;
            //            int alignment_result = planeListAlignmentCV(moving_planes,
            //                    fixed_planes, eigenTransform);
            //            //std::cout << "eigen mat = " << eigenTransform << std::endl;
            //            eigenTransformMap = eigenTransform;            
            this->setPose(Pose(*this, cv::Vec3f(0, 0, 0)));
            this->setIsConvexFlag();
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
            //for (int idx = 0; idx < pts.size(); ++idx) {
            //    pose.transformInPlace(pts[idx]);
            //}              
            return pts;
        }

        std::vector<int> generateCoordIndices() {
            std::vector<int> ptIdxs = {0, 1, 2, 3, 4, 5};
            return ptIdxs;
        }

        std::vector<cv::Vec3f> generateNormals() {
            std::vector<cv::Vec3f> norms = {cv::Vec3f(0, 0, -1)};
            //for (int idx = 0; idx < norms.size(); ++idx) {
            //    pose.rotateInPlace(norms[idx]);
            //}             
            return norms;
        }

        std::vector<int> generateNormalCoordIndices() {
            std::vector<int> normIdxs = {0, 0, 0, 0, 0, 0};
            return normIdxs;
        }

        std::vector<cv::Vec3f> generateColorCoords() {
            static cv::Vec3f green(0.0f, 1.0f, 0.0f);
            static cv::Vec3f orange(1.0f, 0.5f, 0.0f);
            if (_isReal) {
                return std::vector<cv::Vec3f>{green, green, green};
            }
            return std::vector<cv::Vec3f>{orange, orange, orange};
        }

        std::vector<int> generateColorCoordIndices() {
            std::vector<int> colorIdxs = {0, 0, 1, 1, 2, 2};
            return colorIdxs;
        }

        bool contains(Edge<float>::Ptr queryEdge) {
            return (edges[0] == queryEdge || edges[1] == queryEdge || edges[2] == queryEdge);
        }

        void getPlanes(std::unordered_set<Plane<float>::Ptr>& planeSet) {
            edges[0]->getPlanes(planeSet);
            edges[1]->getPlanes(planeSet);
            assert(planeSet.size() == 3);
        }

        float matchPlanes(Corner<_Tpl>::Ptr cornerPtr, std::pair<Plane<float>::Ptr, Plane<float>::Ptr>* planePairs) {
            std::unordered_set<Plane<float>::Ptr> planeSetA, planeSetB;
            getPlanes(planeSetA);
            cornerPtr->getPlanes(planeSetB);
            Plane<float>::Ptr bestMatch[3];
            int idxA = 0;
            float distanceAB, minDistance, distance = 0;
            for (auto plane_iterA = planeSetA.begin(); plane_iterA != planeSetA.end(); ++plane_iterA, ++idxA) {
                minDistance = std::numeric_limits<float>::infinity();
                for (auto plane_iterB = planeSetB.begin(); plane_iterB != planeSetB.end(); ++plane_iterB) {
                    distanceAB = (*plane_iterA)->matchDistance(*plane_iterB);
                    if (distanceAB < minDistance) {
                        minDistance = distanceAB;
                        bestMatch[idxA] = *plane_iterB;
                    }
                }
                planePairs[idxA] = {*plane_iterA, bestMatch[idxA]};
                distance += minDistance;
            }
            // This is a case of the "assignment problem" where we need to assign
            // planes from cornerA to planes of cornerB and minimize cost/dissimilarity
            // https://en.wikipedia.org/wiki/Assignment_problem
            //
            // if this assert fails then one must run the Hungarian algorithm  
            // https://en.wikipedia.org/wiki/Hungarian_algorithm
            // https://github.com/RoboJackets/hungarian
            // or the Edmonds-Karp algorithm to find the optimal match
            // http://www.sanfoundry.com/cpp-program-implement-edmonds-karp-algorithm/
            assert(bestMatch[0] != bestMatch[1] && bestMatch[0] != bestMatch[2] && bestMatch[1] != bestMatch[2]);
            return distance;
        }

        bool isConvex() {
            return _isConvex;
        }

        bool setIsConvexFlag() {
            cv::Point3_<_Tpl> cornerPt = *this;
            cv::Point3_<_Tpl> meanOfNeighbors = edges[0]->getPoint(edges[0]->end);
            meanOfNeighbors += edges[1]->getPoint(edges[1]->end);
            meanOfNeighbors += edges[2]->getPoint(edges[2]->end);
            meanOfNeighbors *= 1.0 / 3.0;
            return (meanOfNeighbors.z - cornerPt.z) > 0;
        }

        bool isReal() const {
            return _isReal;
        }

        void setReal(bool realFlag) {
            _isReal = realFlag;
        }

        bool isInside(const std::vector<cv::Point3_<_Tpl>>&pts) {
            static _Tpl ISREAL_THRESHOLD = 0.005;
            std::unordered_set<Plane<float>::Ptr> planeSet;
            getPlanes(planeSet);

            int numPts = pts.size();
            int convexSignFlip = isConvex() ? 1 : -1;
            _Tpl signed_distance;
            _Tpl max_signed_distance;
            _Tpl avg_max_signed_distance = 0;
            for (cv::Vec<_Tpl, 3> pt : pts) {
                max_signed_distance = -std::numeric_limits<_Tpl>::infinity();
                for (auto plane_iter = planeSet.begin(); plane_iter != planeSet.end(); ++plane_iter) {
                    signed_distance = convexSignFlip * (*plane_iter)->evaluate(pt);
                    if (signed_distance > max_signed_distance) {
                        max_signed_distance = signed_distance;
                    }
                }
                avg_max_signed_distance += max_signed_distance;
            }
            avg_max_signed_distance /= numPts;
            return avg_max_signed_distance > 0;
        }

        cv::Matx<_Tpl, 3, 3> getNonOrthogonalCoordinateSystem() {
            std::vector<cv::Vec<_Tpl, 3 >> axes(3);
            getXYZAxes(axes);
            cv::Matx<_Tpl, 3, 3> coordVecs = cv::Matx<_Tpl, 3, 3>::eye();
            int idx = 0;
            for (auto axis_iter = axes.begin(); axis_iter != axes.end(); ++axis_iter, ++idx) {
                cv::Vec<_Tpl, 3>& axis = *axis_iter;
                coordVecs(idx, 0) = axis[0];
                coordVecs(idx, 1) = axis[1];
                coordVecs(idx, 2) = axis[2];
            }
            //std::cout << "coordVecs = " << coordVecs << std::endl;
            return coordVecs;
        }

        CornerType getCameraFrameCornerType() {
            std::vector<cv::Vec3f> axes(3);
            getXYZAxes(axes);
            //std::cout << "X-axis = " << axes[0] << std::endl;
            //std::cout << "Y-axis = " << axes[1] << std::endl;
            //std::cout << "Z-axis = " << axes[2] << std::endl;
            int idVal = (axes[0][0] > 0) | ((axes[1][1] > 0) << 1) | ((axes[2][2] > 0) << 2);
            //std::cout << "idVal = " << idVal << std::endl;
            CornerType id = CornerType(idVal);
            return id;
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

        float matchDistance(Shape::Ptr qShape) {
            sg::Corner<_Tpl>::Ptr cornerPtr = boost::dynamic_pointer_cast<sg::Corner < _Tpl >> (qShape);
            if (!cornerPtr) {
                return std::numeric_limits<_Tpl>::infinity();
            }
            cv::Vec<_Tpl, 3> errorVec(cornerPtr->x - this->x,
                    cornerPtr->y - this->y,
                    cornerPtr->z - this->z);
            return std::sqrt(errorVec.dot(errorVec));
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
        typedef boost::shared_ptr<const Box> ConstPtr;

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

        //std::vector<cv::rgbd::ObjectGeometryPtr> getCorners();

        //std::vector<cv::rgbd::ObjectGeometryPtr> getEdges();

        //std::vector<cv::rgbd::ObjectGeometryPtr> getPlanes();

        float matchDistance(Shape::Ptr qShape) {
            sg::Box::Ptr boxPtr = boost::dynamic_pointer_cast<sg::Box> (qShape);
            if (!boxPtr) {
                return std::numeric_limits<float>::infinity();
            }
            cv::Vec<float, 3> errorVec(boxPtr->dims.x - this->dims.x,
                    boxPtr->dims.y - this->dims.y,
                    boxPtr->dims.z - this->dims.z);
            return std::sqrt(errorVec.dot(errorVec));
        }

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
        std::vector< std::vector<cv::Vec2f>> uv_coords; // 2d parametric coords within the plane
    public:
        typedef boost::shared_ptr<Cylinder> Ptr;
        typedef boost::shared_ptr<Cylinder> ConstPtr;

        Cylinder() : sg::Shape() {
        }

        Cylinder(float _radius, float _height, Pose _pose)
        : r(_radius), h(_height), sg::Shape(_pose) {
        }

        virtual ~Cylinder() {
        }

        void addCoords(std::vector<cv::Vec2f> &coordVec) {
            uv_coords.push_back(coordVec);
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

        float matchDistance(Shape::Ptr qShape) {
            sg::Cylinder::Ptr cylinderPtr = boost::dynamic_pointer_cast<sg::Cylinder> (qShape);
            if (!cylinderPtr) {
                return std::numeric_limits<float>::infinity();
            }
            cv::Vec<float, 2> errorVec(cylinderPtr->r - this->r,
                    cylinderPtr->h - this->h);
            return std::sqrt(errorVec.dot(errorVec));
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << "Cylinder::r(" << r << "),h(" << h << ")";
            return stringStream.str();
        }

        static Cylinder::Ptr create() {
            return Cylinder::Ptr(boost::make_shared<Cylinder>());
        }

        void serializeBoundary(std::vector<float>& boundary) {
            if (uv_coords.size() < 1) {
                return;
            }
            std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
            for (cv::Vec2f uv_coord : uv_poly_coords) {
                boundary.push_back(uv_coord[0]);
                boundary.push_back(uv_coord[1]);
            }
        }

        void deserializeBoundary(const std::vector<float>& boundary) {
            std::vector<cv::Vec2f> uv_poly_coords;
            for (int ptIdx = 0; ptIdx < boundary.size(); ptIdx += 2) {
                cv::Vec2f uv_coord(boundary[ptIdx], boundary[ptIdx + 1]);
                uv_poly_coords.push_back(uv_coord);
            }
            addCoords(uv_poly_coords);
        }

        std::vector<cv::Vec2f> getBoundary() const {
            std::vector<cv::Vec2f> uv_poly_coords;
            return (uv_coords.size() > 0) ? uv_coords[0] : uv_poly_coords;
        }

        void setBoundary(std::vector<cv::Vec2f> &boundary) {
            if (uv_coords.size() == 0) {
                uv_coords.push_back(boundary);
            } else {
                uv_coords[0] = boundary;
            }
        }

        cv::Vec2f xyzToUV(const cv::Vec3f& xyz) const {
            cv::Vec2f uv(0, 0);
            cv::Vec3f center;
            cv::Vec3f xVec(1, 0, 0);
            cv::Vec3f yVec(0, 1, 0);
            cv::Vec3f zVec(0, 0, 1);
            pose.getTranslation(center);
            pose.rotateInPlace(xVec);
            pose.rotateInPlace(yVec);
            pose.rotateInPlace(zVec);
            uv[0] = (xyz - center).dot(zVec);
            uv[1] = std::atan2((xyz - center).dot(yVec), (xyz - center).dot(xVec));
            assert(!std::isnan(uv[0]) && !std::isnan(uv[1]));
            return uv;
        }

        float radius() {
            return r;
        };

        float height() {
            return h;
        };
    private:
        // -------------------------
        // Disabling default copy constructor and default
        // assignment operator.
        // -------------------------
        //Cylinder(const Cylinder& ref);
        //Cylinder& operator=(const Cylinder& ref);
        static constexpr float DEFAULT_RESOLUTION = 16;
        float r, h; // radius, height
    };

    class Sphere : public Shape {
        std::vector< std::vector<cv::Vec2f>> uv_coords; // 2d parametric coords within the plane
    public:
        typedef boost::shared_ptr<Sphere> Ptr;
        typedef boost::shared_ptr<Sphere> ConstPtr;

        Sphere() : sg::Shape() {
        }

        Sphere(float _radius, Pose _pose)
        : r(_radius), sg::Shape(_pose) {
        }

        virtual ~Sphere() {
        }

        void addCoords(std::vector<cv::Vec2f> &coordVec) {
            uv_coords.push_back(coordVec);
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

        float matchDistance(Shape::Ptr qShape) {
            sg::Sphere::Ptr sphere = boost::dynamic_pointer_cast<sg::Sphere> (qShape);
            if (!sphere) {
                return std::numeric_limits<float>::infinity();
            }
            float error = sphere->r - this->r;
            return error;
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << "Sphere::r(" << r << ")";
            return stringStream.str();
        }

        static Sphere::Ptr create() {
            return Sphere::Ptr(boost::make_shared<Sphere>());
        }

        cv::Vec2f xyzToUV(const cv::Vec3f& xyz) const {
            cv::Vec2f uv;
            cv::Vec3f normal;
            cv::Vec3f xVec(1, 0, 0);
            cv::Vec3f yVec(0, 1, 0);
            cv::Vec3f zVec(0, 0, 1);
            pose.getTranslation(normal);
            pose.rotateInPlace(xVec);
            pose.rotateInPlace(yVec);
            pose.rotateInPlace(zVec);
            normal = xyz - normal;
            normal /= cv::norm(normal);
            uv[0] = std::acos(normal.dot(zVec));
            uv[1] = std::atan2(normal.dot(yVec), normal.dot(xVec));
            assert(!std::isnan(uv[0]) && !std::isnan(uv[1]));
            return uv;
        }

        void serializeBoundary(std::vector<float>& boundary) {
            if (uv_coords.size() < 1) {
                return;
            }
            std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
            for (cv::Vec2f uv_coord : uv_poly_coords) {
                boundary.push_back(uv_coord[0]);
                boundary.push_back(uv_coord[1]);
            }
        }

        void deserializeBoundary(const std::vector<float>& boundary) {
            std::vector<cv::Vec2f> uv_poly_coords;
            for (int ptIdx = 0; ptIdx < boundary.size(); ptIdx += 2) {
                cv::Vec2f uv_coord(boundary[ptIdx], boundary[ptIdx + 1]);
                uv_poly_coords.push_back(uv_coord);
            }
            addCoords(uv_poly_coords);
        }

        std::vector<cv::Vec2f> getBoundary() const {
            std::vector<cv::Vec2f> uv_poly_coords;
            return (uv_coords.size() > 0) ? uv_coords[0] : uv_poly_coords;
        }

        void setBoundary(std::vector<cv::Vec2f> &boundary) {
            if (uv_coords.size() == 0) {
                uv_coords.push_back(boundary);
            } else {
                uv_coords[0] = boundary;
            }
        }

        float radius() {
            return r;
        };
    private:
        // -------------------------
        // Disabling default copy constructor and default
        // assignment operator.
        // -------------------------
        //Sphere(const Sphere& ref);
        //Sphere& operator=(const Sphere& ref);
        static constexpr float DEFAULT_RESOLUTION = 16;
        float r; // radius
    };
} /* namespace sg */
#endif /* __cplusplus */

#endif /* SHAPEGRAMMARLIBRARY_HPP */

