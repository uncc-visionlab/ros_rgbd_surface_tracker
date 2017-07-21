#ifndef OPENCV_FUNCTION_DEV_HPP
#define OPENCV_FUNCTION_DEV_HPP
#ifdef __cplusplus

#include <functional>
#include <iostream>
#include <math.h>
#include <queue>
#include <sstream>
#include <stdexcept>

#include <boost/shared_ptr.hpp>

#include <opencv2/core.hpp>

#define DEBUG false

namespace cv {

    // coeffs are stored in implicit form
    // e.g. To find signed distance from point (x0,y0,z0) to this plane evaluate
    // this->x*x0 + this->y*y0 + this->z*z0 + this->d = 0

    template<typename _Tpl> class Line2_ {
    public:

        friend std::ostream& operator<<(std::ostream& os, const Line2_<_Tpl>& l) {
            os << "[" << l.v << ", " << l.p0 << "]";
            return os;
        }

        Point_<_Tpl> getPoint(const _Tpl& lambda) const {
            Point_<_Tpl> p;
            p.x = lambda * v.x + p0.x;
            p.y = lambda * v.y + p0.y;
            return p;
        }

        void setPoint(Point_<_Tpl>& p, const _Tpl& lambda) const {
            p.x = lambda * v.x + p0.x;
            p.y = lambda * v.y + p0.y;
        }

        static void intersect(const Line2_<_Tpl>& l1, const Line2_<_Tpl>& l2,
                Point_<_Tpl>& p) {
            _Tpl ilambda = (l1.v.y * (l1.p0.x - l2.p0.x) - l1.v.x * (l1.p0.y - l2.p0.y)) / (l1.v.y * l2.v.x - l1.v.x * l2.v.y);
            p.x = l2.p0.x + ilambda * l2.v.x;
            p.y = l2.p0.y + ilambda * l2.v.y;
        }

        Point_<_Tpl> v;
        Point_<_Tpl> p0;
    };
    typedef Line2_<float> Line2f;
    typedef Line2_<double> Line2d;

    template<typename _Tpl> class LineSegment2_ : public Line2_<_Tpl> {
    public:

        friend std::ostream& operator<<(std::ostream& os, const LineSegment2_<_Tpl>& l) {
            os << "[" << l.v << ", " << l.p0 << ", "
                    << l.getPoint(l.start) << ", " << l.getPoint(l.end) << "]";
            return os;
        }
        _Tpl start, end;
    };
    typedef LineSegment2_<float> LineSegment2f;
    typedef LineSegment2_<double> LineSegment2d;

    template<typename _Tpl> class Line3_ {
    public:
        typedef boost::shared_ptr<Line3_<_Tpl> > Ptr;
        typedef boost::shared_ptr<const Line3_<_Tpl> > ConstPtr;

        Line3_(const Point3_<_Tpl>& tip, const Point3_<_Tpl>& tail) {
            this->p0 = tail;
            this->v = tip - tail;
        }

        Point3_<_Tpl> getPoint(const _Tpl& lambda) const {
            Point3_<_Tpl> p;
            p.x = lambda * v.x + p0.x;
            p.y = lambda * v.y + p0.y;
            p.z = lambda * v.z + p0.z;
            return p;
        }

        void setPoint(Point3_<_Tpl>& p, const _Tpl& lambda) const {
            p.x = lambda * v.x + p0.x;
            p.y = lambda * v.y + p0.y;
            p.z = lambda * v.z + p0.z;
        }

        _Tpl distanceSquared(const Point3_<_Tpl>& pt) {
            Point3_<_Tpl> num = v.cross(Point3_<_Tpl>(pt.x - p0.x, pt.y - p0.y, pt.z - p0.z));
            return (num.x * num.x + num.y * num.y + num.z * num.z) / (v.x * v.x + v.y * v.y + v.z * v.z);
        }

        friend std::ostream& operator<<(std::ostream& os, const Line3_<_Tpl>& l) {
            os << "[" << l.v << ", " << l.p0 << "]";
            return os;
        }
        Point3_<_Tpl> v;
        Point3_<_Tpl> p0;
    };
    typedef Line3_<float> Line3f;
    typedef Line3_<double> Line3d;

    template<typename _Tpl> class Plane3_ : public Point3_<_Tpl> {
    public:

        static constexpr float ANGLE_THRESHOLD = 5.f;
        static constexpr float COPLANAR_COS_ANGLE_THRESHOLD = cos(ANGLE_THRESHOLD * M_PI / 180.f); // degrees
        static constexpr float PERPENDICULAR_SIN_ANGLE_THRESHOLD = sin(ANGLE_THRESHOLD * M_PI / 180.f); // degrees
        static constexpr float COPLANAR_COS_EXPECTED_DIHEDRAL_ANGLE = 1.f; // cos(0)

        typedef boost::shared_ptr<Plane3_<_Tpl> > Ptr;
        typedef boost::shared_ptr<const Plane3_<_Tpl> > ConstPtr;

        Plane3_() : d(0), Point3_<_Tpl>() {
        }

        Plane3_(_Tpl _x, _Tpl _y, _Tpl _z, _Tpl _d) : d(_d), Point3_<_Tpl>(_x, _y, _z) {
        }
        
        Plane3_(const Point3_<_Tpl>& pt1, const Point3_<_Tpl>& pt2,
                const Point3_<_Tpl>& pt3) {
            setCoeffs(pt1, pt2, pt3);
        }

        Plane3_<_Tpl> clone() {
            return Plane3_<_Tpl>(this->x, this->y, this->z, this->d);
        }

        _Tpl orthogonalDistanceSquared(Point3_<_Tpl> pt) {
            return evaluate(pt) * evaluate(pt);
        }

        _Tpl signedOrthogonalDistance(Point3_<_Tpl> pt) {
            return evaluate(pt);
        }

        bool intersect(Plane3_<_Tpl> planeB, Line3_<_Tpl>& line) {
            line.v = this->cross(planeB);
            //double detB = (planeA.x * planeB.y - planeB.x * planeA.y);
            //double detA = line.v.z;
            if (line.v.z == 0) {
                return false;
            }
            //std::cout << "detA " << detA << " detB " << detB << std::endl;
            line.p0.x = (_Tpl) (-planeB.y * this->d + this->y * planeB.d) / line.v.z;
            line.p0.y = (_Tpl) (planeB.x * this->d - this->x * planeB.d) / line.v.z;
            line.p0.z = 0;
            return true;
        }

        bool intersect(const Line3_<_Tpl>& line, Point3_<_Tpl>& pt) {

            //            std::cout << "line.v" << line.v << std::endl;
            cv::Point3_<_Tpl> line_vec = line.v / cv::norm<_Tpl>(line.v);
            //            std::cout << "line_vec after norm" << line_vec << std::endl;
            //            std::cout << "norm line_vec after" << cv::norm<_Tpl>(line_vec) << std::endl;
            cv::Point3_<_Tpl> plane_normal(*this);

            pt = -(line.p0 + ((line.p0.dot(plane_normal) + this->d) / (line_vec.dot(plane_normal))) * line_vec);
            return true;

        }

        void setCoeffs(_Tpl _x, _Tpl _y, _Tpl _z, _Tpl _d) {
            this->x = _x;
            this->y = _y;
            this->z = _z;
            this->d = _d;
        }

        void scale(_Tpl scalef) {
            this->x *= scalef;
            this->y *= scalef;
            this->z *= scalef;
            this->d *= scalef;
        }

        _Tpl evaluate(const Point3_<_Tpl>& pt) {
            return this->x * pt.x + this->y * pt.y + this->z * pt.z + this->d;
        }

        _Tpl evaluate(_Tpl _x, _Tpl _y, _Tpl _z) {
            return this->x * _x + this->y * _y + this->z * _z + this->d;
        }

        _Tpl evaluateDerivative(int dim) {
            switch (dim) {
                case 1: return this->x;
                    break;
                case 2: return this->y;
                    break;
                case 3: return this->z;
                    break;
                default: throw std::invalid_argument("invalid dimension");
            }

        }

        _Tpl evaluateDerivative(int dim, _Tpl _x, _Tpl _y, _Tpl _z) {
            switch (dim) {
                case 1: return this->x;
                    break;
                case 2: return this->y;
                    break;
                case 3: return this->z;
                    break;
                default: throw std::invalid_argument("invalid dimension");
            }
        }

        _Tpl evaluateDerivative(int dim, const Point3_<_Tpl>& pt) {
            switch (dim) {
                case 1: return this->x;
                    break;
                case 2: return this->y;
                    break;
                case 3: return this->z;
                    break;
                default: throw std::invalid_argument("invalid dimension");
            }
        }

        void setCoeffs(const Point3_<_Tpl>& pt1, const Point3_<_Tpl>& pt2,
                const Point3_<_Tpl>& pt3) {
            this->x = (pt2.y - pt1.y)*(pt3.z - pt1.z)-(pt3.y - pt1.y)*(pt2.z - pt1.z);
            this->y = (pt2.z - pt1.z)*(pt3.x - pt1.x)-(pt3.z - pt1.z)*(pt2.x - pt1.x);
            this->z = (pt2.x - pt1.x)*(pt3.y - pt1.y)-(pt3.x - pt1.x)*(pt2.y - pt1.y);
            this->d = -(this->x * pt1.x + this->y * pt1.y + this->z * pt1.z);
        }

        _Tpl cosDihedralAngle(const Plane3_<_Tpl> test_plane) const {
            return this->x * test_plane.x + this->y * test_plane.y + this->z * test_plane.z;
        }

        _Tpl angleDistance(Plane3_<_Tpl> planeA) const {
            return (_Tpl) COPLANAR_COS_EXPECTED_DIHEDRAL_ANGLE - cosDihedralAngle(planeA);
        }

        bool epsilonEquals(Plane3_<_Tpl> planeA, float eps = COPLANAR_COS_ANGLE_THRESHOLD) const {
            return (COPLANAR_COS_EXPECTED_DIHEDRAL_ANGLE - cosDihedralAngle(planeA)
                    < COPLANAR_COS_ANGLE_THRESHOLD);
        }

        bool epsilonPerpendicular(Plane3_<_Tpl> planeA, float eps = PERPENDICULAR_SIN_ANGLE_THRESHOLD) const {
            return (abs(cosDihedralAngle(planeA)) < eps);
        }

        void interpolate(float alpha, Plane3_<_Tpl> planeA, Plane3_<_Tpl> planeB,
                Point3_<_Tpl> pt) {
            this->x = alpha * planeA.x + (1 - alpha) * planeB.x;
            this->y = alpha * planeA.x + (1 - alpha) * planeB.x;
            this->z = alpha * planeA.x + (1 - alpha) * planeB.x;
            this->d = -(this->x * pt.x + this->y * pt.y + this->z * pt.z);
        }

        void convertHessianNormalForm() {
            float normScale = 1.f / sqrt(this->x * this->x +
                    this->y * this->y + this->z * this->z);
            scale(normScale);
        }

        friend std::ostream& operator<<(std::ostream& os, const Plane3_<_Tpl>& p) {
            os << "[" << p.x << ", " << p.y << ", " << p.z
                    << ", " << p.d << "]";
            return os;
        }

        cv::Point3_<_Tpl> uvToXYZ(const cv::Point_<_Tpl>& uv) {
            _Tpl threshold = 0.6;
            static cv::Point3_<_Tpl> uVec;
            static cv::Point3_<_Tpl> vVec;
            if (std::abs<_Tpl>(this->x) <= threshold) {
                _Tpl inverse = 1.0 / std::sqrt(this->y * this->y + this->z * this->z);
                uVec = cv::Point3_<_Tpl>((_Tpl) 0, inverse * this->z, -inverse * this->y);
            } else if (std::abs<_Tpl>(this->y) <= threshold) {
                _Tpl inverse = 1.0 / std::sqrt(this->x * this->x + this->z * this->z);
                uVec = cv::Point3_<_Tpl>(-inverse * this->z, (_Tpl) 0, inverse * this->x);
            } else {
                _Tpl inverse = 1.0 / std::sqrt(this->x * this->x + this->y * this->y);
                uVec = cv::Point3_<_Tpl>(inverse * this->y, -inverse * this->x, (_Tpl) 0);
            }
            vVec = uVec.cross(*this);
            cv::Point3_<_Tpl> pt0(-d * this->x, -d * this->y, -d * this->z);
            //uVec3 *= 1.0/std::sqrt(uVec3.dot(uVec3));
            //vVec3 *= 1.0/std::sqrt(vVec3.dot(vVec3));
            pt0.x = pt0.x + uv.x * uVec.x + uv.y * vVec.x;
            pt0.y = pt0.y + uv.x * uVec.y + uv.y * vVec.y;
            pt0.z = pt0.z + uv.x * uVec.z + uv.y * vVec.z;
            return pt0;
        }

        cv::Point_<_Tpl> xyzToUV(const Point3_<_Tpl>& p) {
            _Tpl threshold = 0.6;
            cv::Point_<_Tpl> uv;
            static cv::Point3_<_Tpl> uVec;
            static cv::Point3_<_Tpl> vVec;
            if (std::abs(this->x) <= threshold) {
                _Tpl inverse = 1.0 / std::sqrt(this->y * this->y + this->z * this->z);
                uVec = cv::Point3_<_Tpl>((_Tpl) 0.0, inverse * this->z, -inverse * this->y);
            } else if (std::abs(this->y) <= threshold) {
                _Tpl inverse = 1.0 / std::sqrt(this->x * this->x + this->z * this->z);
                uVec = cv::Point3_<_Tpl>(-inverse * this->z, (_Tpl) 0.0, inverse * this->x);
            } else {
                _Tpl inverse = 1.0 / std::sqrt(this->x * this->x + this->y * this->y);
                uVec = cv::Point3_<_Tpl>(inverse * this->y, -inverse * this->x, (_Tpl) 0.0);
            }
            vVec = uVec.cross(*this);
            cv::Point3_<_Tpl> pt0(-d * this->x, -d * this->y, -d * this->z), uVec3, vVec3;
            //uVec3 *= 1.0/std::sqrt(uVec3.dot(uVec3));
            //vVec3 *= 1.0/std::sqrt(vVec3.dot(vVec3));
            pt0 = p - pt0;
            uv.x = pt0.dot(uVec);
            uv.y = pt0.dot(vVec);
            return uv;
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << "(" << this->x << ", " << this->y
                    << ", " << this->z << ", " << this->d << ")";
            return stringStream.str();
        }

        _Tpl d;
    };
    typedef Plane3_<float> Plane3f;
    typedef Plane3_<double> Plane3d;
    typedef Plane3_<int> Plane3i;

    template<typename _Tp> class DataDepth<Plane3_<_Tp> > {
    public:

        enum {
            value = CV_USRTYPE1, fmt = (int) 'r'
        };
    };

    template<typename _Tp> class DataType<Plane3_<_Tp> > {
    public:
        typedef Plane3_<_Tp> value_type;
        typedef value_type work_type;
        typedef _Tp channel_type;
        // DataDepth is another helper trait class

        enum {
            generic_type = 0,
            depth = cv::DataDepth<channel_type>::value, channels = 4,
            fmt = ((channels - 1) << 8) + DataDepth<value_type>::fmt,
            type = CV_MAKETYPE(depth, channels)
        };
        typedef Vec<channel_type, channels> vec_type;
    };

    template<typename _Tpl> class Edge3_ {
    public:
        typedef boost::shared_ptr<Edge3_<_Tpl> > Ptr;
        typedef boost::shared_ptr<const Edge3_<_Tpl> > ConstPtr;

        Edge3_(Plane3_<_Tpl> _p1, Plane3_<_Tpl> _p2) : p1(_p1), p2(_p2) {
        }

        _Tpl evaluate(_Tpl _x, _Tpl _y, _Tpl _z) {
            return this->p1.evaluate(_x, _y, _z) * this->p2.evaluate(_x, _y, _z);
        }

        _Tpl evaluate(const Point3_<_Tpl>& pt) {
            return this->p1.evaluate(pt) * this->p2.evaluate(pt);
        }

        _Tpl evaluateDerivative(int dim, _Tpl _x, _Tpl _y, _Tpl _z) {
            _Tpl f1 = this->p1.evaluate(_x, _y, _z);
            _Tpl f2 = this->p2.evaluate(_x, _y, _z);
            _Tpl d1 = this->p1.evaluateDerivative(dim);
            _Tpl d2 = this->p2.evaluateDerivative(dim);

            return 2 * f1 * f2 * (d1 * f2 + d2 * f1);
        }

        _Tpl evaluateDerivative(int dim, const Point3_<_Tpl>& pt) {
            _Tpl f1 = this->p1.evaluate(pt);
            _Tpl f2 = this->p2.evaluate(pt);
            _Tpl d1 = this->p1.evaluateDerivative(dim);
            _Tpl d2 = this->p2.evaluateDerivative(dim);

            return (d1 * f2 + d2 * f1);
        }

        friend std::ostream& operator<<(std::ostream& os, const Edge3_<_Tpl>& e) {
            os << "[" << e.l << ", " << e.p1 << ", " << e.p2 << "]";
            return os;
        }
        Line3_<_Tpl> l;
        Plane3_<_Tpl> p1, p2;
    };
    typedef Edge3_<float> Edge3f;
    typedef Edge3_<double> Edge3d;

    template<typename _Tpl> class Corner3_ {
    public:
        typedef boost::shared_ptr<Corner3_<_Tpl> > Ptr;
        typedef boost::shared_ptr<const Corner3_<_Tpl> > ConstPtr;

        friend std::ostream& operator<<(std::ostream& os, const Corner3_<_Tpl>& c) {
            os << "[" << c.p << "," << c.v1 << ", " << c.v2 << ", " << c.v3 << "]";
            return os;
        }
        Point3_<_Tpl> p;
        Line3_<_Tpl> v1;
        Line3_<_Tpl> v2;
        Line3_<_Tpl> v3;
    };
    typedef Corner3_<float> Corner3f;
    typedef Corner3_<double> Corner3d;

    template<typename _Tpl> class LabeledPlane3_ : public Plane3_<_Tpl> {
    public:

        typedef boost::shared_ptr<LabeledPlane3_<_Tpl> > Ptr;
        typedef boost::shared_ptr<const LabeledPlane3_<_Tpl> > ConstPtr;

        LabeledPlane3_() : label(0), Plane3_<_Tpl>() {

        };

        LabeledPlane3_(Plane3_<_Tpl> p, int _label) : label(_label), Plane3_<_Tpl>(p.x, p.y, p.z, p.d) {

        };

        LabeledPlane3_(_Tpl x, _Tpl y, _Tpl z, _Tpl d) : label(0),
        Plane3_<_Tpl>(x, y, z, d) {

        };

        LabeledPlane3_(_Tpl x, _Tpl y, _Tpl z, _Tpl d, int _label) : label(_label),
        Plane3_<_Tpl>(x, y, z, d) {

        };

        LabeledPlane3_<_Tpl> clone() {
            LabeledPlane3_<_Tpl> lp;
            lp.x = this->x;
            lp.y = this->y;
            lp.z = this->z;
            lp.d = this->d;
            lp.label = this->label;
            return lp;
        }

        float distance(float theta) {
            float myTheta = fastAtan2(this->y, this->x); // degrees
            return abs(myTheta - theta);
        }

        int label;
    };

    template<typename _Tpl> class DataType<LabeledPlane3_<_Tpl> > {
    public:
        typedef LabeledPlane3_<_Tpl> value_type;
        typedef value_type work_type;
        typedef value_type channel_type;
        // DataDepth is another helper trait class

        enum {
            generic_type = 0,
            depth = cv::DataDepth<channel_type>::value, channels = 1,
            fmt = ((channels - 1) << 8) + DataDepth<value_type>::fmt,
            type = CV_MAKETYPE(depth, channels)
        };
        typedef Vec<channel_type, channels> vec_type;
    };
    typedef LabeledPlane3_<float> LabeledPlane3f;

    template<typename _Tpl> class DataDepth<LabeledPlane3_<_Tpl> > {
    public:

        enum {
            value = CV_USRTYPE1, fmt = (int) 'r'
        };
    };

    class Consensus {
    public:
        int inliers, outliers, invalid;

        Consensus() : inliers(0), outliers(0), invalid(0) {
        }

        Consensus(int _inliers, int _outliers, int _invalid) :
        inliers(_inliers), outliers(_outliers), invalid(_invalid) {

        }

        float consensus() const {
            return ((float) inliers) / (inliers + outliers);
        }

        friend std::ostream& operator<<(std::ostream& os, const Consensus& c) {
            os << "[ i=" << c.inliers << ", o=" << c.outliers << ", nan=" << c.invalid << "]";
            return os;
        }
    };

    class RectWithError : public Rect, public Consensus {
    public:
        float error, noise;

        RectWithError() : Rect(), error(0), noise(0) {
        }

        RectWithError(int _x, int _y, int _width, int _height) :
        Rect(_x, _y, _width, _height), error(0), noise(0), Consensus() {

        }

        RectWithError(int _x, int _y, int _width, int _height, float _error,
                int _inliers, int _outliers, int _invalid = 0) :
        Rect(_x, _y, _width, _height), error(_error), noise(0),
        Consensus(_inliers, _outliers, _invalid) {

        }

        RectWithError clone() {
            return RectWithError(x, y, width, height, error,
                    inliers, outliers, invalid);
        }

        void clearStatistics() {
            error = inliers = outliers = invalid = 0;
        }

        void getCenter(int& ix, int& iy) {
            ix = x + (width >> 1);
            iy = y + (height >> 1);
        }

        friend std::ostream& operator<<(std::ostream& os, const RectWithError& r) {
            os << "[ x=" << r.x << ", y=" << r.y << ", w=" << r.width
                    << ", h=" << r.height << ", in=" << r.inliers
                    << ", out=" << r.outliers << ", bad=" << r.invalid << ", e=" << r.error << "]";
            return os;
        }

        class ErrorComparator {
        public:

            //            bool operator()(RectWithError* r1, RectWithError* r2) {
            //                return r1->error > r2->error;
            //            }

            bool operator()(RectWithError r1, RectWithError r2) {
                return r1.error > r2.error;
            }
        };

    };
    typedef std::priority_queue<RectWithError,
    std::vector<RectWithError>,
    RectWithError::ErrorComparator> ErrorSortedRectQueue;

    template<typename _Tpl> class TesselatedPlane3_ : public LabeledPlane3_<_Tpl> {
        std::vector<RectWithError> imgQuadArr;
        //        Point3f centroid;
    public:

        typedef boost::shared_ptr<TesselatedPlane3_<_Tpl> > Ptr;
        typedef boost::shared_ptr<const TesselatedPlane3_<_Tpl> > ConstPtr;

        TesselatedPlane3_(Plane3_<_Tpl> _p, int _label) :
        LabeledPlane3_<_Tpl>(_p, _label) {

        };

        void addQuad(const RectWithError& re) {
            imgQuadArr.push_back(re);
        }

        const std::vector<RectWithError>& getQuads() {
            return imgQuadArr;
        }

        // Comparator sorts objects in terms of increasing error

        class ErrorComparator {
        public:

            inline bool operator()(const TesselatedPlane3_<_Tpl>::Ptr& p1, const TesselatedPlane3_<_Tpl>::Ptr& p2) {
                return p1->totalError() < p2->totalError();
            }
        };

        // Comparator sorts objects in terms of decreasing area

        class AreaComparator {
        public:

            inline bool operator()(const TesselatedPlane3_<_Tpl>::Ptr& p1, const TesselatedPlane3_<_Tpl>::Ptr& p2) {
                return p1->imgQuadArr.size() > p2->imgQuadArr.size();
            }
        };

        //        Point3f getCentroid() {
        //            return centroid;
        //        }
        //        
        //        void setCentroid(Point3f& pt3) {
        //            centroid.x = pt3.x; 
        //            centroid.y = pt3.y;
        //            centroid.z = pt3.z;
        //        }
        //        

        float totalError() {
            float error = 0;
            for (std::vector<RectWithError>::iterator quadIter = imgQuadArr.begin();
                    quadIter != imgQuadArr.end(); ++quadIter) {
                error += quadIter->error;
            }
            return error;
        }

        float avgError() {
            float error = 0, numPoints = 0;
            for (std::vector<RectWithError>::iterator quadIter = imgQuadArr.begin();
                    quadIter != imgQuadArr.end(); ++quadIter) {
                error += quadIter->error;
                numPoints += quadIter->inliers + quadIter->outliers;
            }
            return error / numPoints;
        }

        float area() {
            float area = 0;
            for (std::vector<RectWithError>::iterator quadIter = imgQuadArr.begin();
                    quadIter != imgQuadArr.end(); ++quadIter) {
                //area += quadIter->width * quadIter->height;
                area += quadIter->area();
            }
            return area;
        }

        int getConsensus(int& inliers, int& outliers, int& invalid) {
            inliers = outliers = invalid = 0;
            for (std::vector<RectWithError>::iterator quadIter = imgQuadArr.begin();
                    quadIter != imgQuadArr.end(); ++quadIter) {
                inliers += quadIter->inliers;
                outliers += quadIter->outliers;
                invalid += quadIter->invalid;
            }
            return inliers;
        }
    };
    typedef TesselatedPlane3_<float> TesselatedPlane3f;

    typedef std::priority_queue<TesselatedPlane3f,
    std::vector<TesselatedPlane3f>,
    TesselatedPlane3f::ErrorComparator> ErrorSortedPlaneQueue;

    template<class T>
    class QuadPyramid {
    public:
        typedef boost::shared_ptr<QuadPyramid> Ptr;
        typedef boost::shared_ptr<QuadPyramid> ConstPtr;
        int level;
        int width;
        int height;
        int blockSize;
        std::vector<T> objects;
        QuadPyramid<T>* parent;
        QuadPyramid<T>::Ptr child;

        QuadPyramid(int _width, int _height, int _blockSize, int _level = 0,
                QuadPyramid<T>* _parent = NULL, QuadPyramid<T>* _child = NULL) :
        width(_width), height(_height), blockSize(_blockSize), level(_level),
        parent(_parent), child(_child) {
            //            int minDim = std::min(_width, _height);
            //            int maxPossibleLevels = 0;
            //            while (minDim >>= 1) ++maxPossibleLevels;
            //            int lastLevel = std::min(levels, maxPossibleLevels);
            //            std::cout << "Creating " << levels << " quad tree levels." << std::endl;
            std::cout << "QuadPyramid (x,y) = " << width << ", " << height << " allocated "
                    << (width * height) << " elements. " << std::endl;
            if (blockSize % 2 == 1) {
                std::cout << "THIS CODE HAS BUGS FOR QUADTREES WITH ODD BLOCKSIZE!" << std::endl;
            }
            assert(blockSize % 2 == 0);

            objects.resize(width * height);
        }

        bool inTree(int pos_x, int pos_y) {
            int xBlock = pos_x / blockSize;
            int yBlock = pos_y / blockSize;
            return xBlock < width && yBlock < height;
        }

        T& getObjectFromAncestors(int pos_x, int pos_y) {
            int xBlock = pos_x / blockSize;
            int yBlock = pos_y / blockSize;
            T& data = objects[yBlock * width + xBlock];
            //std::cout << "data " << data << " (xblock,yblock) = (" << xBlock << ", " << yBlock << ")" << std::endl;
            if (!data && parent) {
                return parent->getObjectFromAncestors(pos_x, pos_y);
            }
            return data;
        }

        std::vector<T>& getObjects() {
            return objects;
        }

        void setObject(int pos_x, int pos_y, T& data) {
            // BUG HERE WHEN CREATE TREE WITH ODD BLOCKSIZE
            objects[pos_y / blockSize * width + pos_x / blockSize] = data;
        }

        QuadPyramid<T> getPyramidLevel(int _level) {
            //std::cout << "_level = " << _level << " level = " << level << std::endl;
            if (_level == level) {
                return *this;
            } else {
                if (_level < level) {
                    int _width = width >> 1;
                    int _height = height >> 1;
                    if (width > 0 && height > 0) {
                        parent = new QuadPyramid<T>(_width, _height,
                                blockSize << 1, level - 1, NULL, this);
                        return parent->getPyramidLevel(_level);
                    }
                }
                if (_level > level) {
                    int _width = width << 1;
                    int _height = height << 1;
                    child = QuadPyramid<T>::Ptr(new QuadPyramid<T>(_width, _height,
                            blockSize >> 1, level + 1, this, NULL));
                    return child->getPyramidLevel(_level);
                }
            }
        }

        void getCorners(int pos_x, int pos_y, Point2i& tlc, Point2i& brc) {
            tlc.x = pos_x*blockSize;
            tlc.y = pos_y*blockSize;
            brc.x = tlc.x + blockSize;
            brc.y = tlc.y + blockSize;
        }
    };

}
#endif /* __cplusplus */
#endif /* OPENCV_FUNCTION_DEV_H */

