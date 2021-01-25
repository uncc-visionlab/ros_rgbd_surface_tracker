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

// smart pointer implementation for classes
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// to serialize cv::Mat objects for transfer
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

// for imemstream bytestream source for unpacking data
#include <streambuf>
#include <istream>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>
#include <ros_rgbd_surface_tracker/PlaneImageQuantizer.hpp>

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

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)

namespace boost {
    namespace serialization {

        /** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive & ar, const ::cv::Mat& m, const unsigned int version) {
            size_t elem_size = m.elemSize();
            size_t elem_type = m.type();

            ar & m.cols;
            ar & m.rows;
            ar & elem_size;
            ar & elem_type;

            const size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

        /** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive & ar, ::cv::Mat& m, const unsigned int version) {
            int cols, rows;
            size_t elem_size, elem_type;

            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;

            m.create(rows, cols, elem_type);

            size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }
        // Try read next object from archive

        template<class Archive>
        bool try_stream_next(Archive &ar, ::cv::Mat &o) {
            bool success = false;
            try {
                ar >> o;
                success = true;
            } catch (const boost::archive::archive_exception &e) {
                if (e.code != boost::archive::archive_exception::input_stream_error) {
                    throw;
                }
            }
            return success;
        }

        template<class Archive>
        bool try_stream_next(Archive &ar, const std::istream &s, ::cv::Mat &o) {
            bool success = false;
            try {
                ar >> o;
                success = true;
            } catch (const boost::archive::archive_exception &e) {
                if (e.code != boost::archive::archive_exception::input_stream_error) {
                    throw;
                }
            }
            return success;
        }
    }
}

struct membuf : std::streambuf {

    membuf(char const* base, size_t size) {
        char* p(const_cast<char*> (base));
        this->setg(p, p, p + size);
    }
};

//class imemstream : virtual membuf, public std::istream {
//public:
//    imemstream(char const* base, size_t size)
//    : membuf(base, size)
//    , std::istream(static_cast<std::streambuf*> (this)) {
//    }
//};

class VectorCompressor {
public:

    static std::vector<int8_t> compress(std::vector<int8_t> uncompressedData) {
        namespace io = boost::iostreams;
        //char* uncompressedChars = new char[uncompressedData.size()]; //init this with the correct size
        //std::copy(uncompressedData.begin(), uncompressedData.end(), uncompressedChars);
        std::string serial_str;
        {
            io::back_insert_device<std::string> inserter(serial_str);
            io::stream<io::back_insert_device<std::string> > s(inserter);

            io::filtering_streambuf<io::output> out;
            out.push(io::zlib_compressor());
            out.push(s);

            //boost::archive::binary_oarchive oa(s);
            boost::archive::binary_oarchive oa(out);
            oa << uncompressedData;
            s.flush();
        }
        return std::vector<int8_t>(serial_str.begin(), serial_str.end());
    }

    static std::vector<int8_t> decompress(std::vector<int8_t> compressed) {
        namespace io = boost::iostreams;
        std::vector<int8_t> decompressed;
        membuf _membuf(reinterpret_cast<const char*> (compressed.data()), compressed.size());
        {
            io::filtering_streambuf<io::input> in;
            in.push(io::zlib_decompressor());
            in.push(_membuf);

            //boost::archive::binary_iarchive ia(_membuf);
            boost::archive::binary_iarchive ia(in);
            ia >> decompressed;
        }
        return decompressed;
    }
};


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
            typedef boost::shared_ptr<ocvMat> Ptr;
            typedef boost::shared_ptr<const ocvMat> ConstPtr;

            void setData(const cv::Mat& _data) {
                if (_data.empty())
                    data.release();
                else
                    data = _data;
            }

            const cv::Mat& getData() const {
                return data;
            }

            std::vector<int8_t> toByteArray() const {
                if (data.empty()) {
                    return std::vector<int8_t>();
                }
                namespace io = boost::iostreams;
                std::string serial_str;
                {
                    io::back_insert_device<std::string> inserter(serial_str);
                    io::stream<io::back_insert_device<std::string> > s(inserter);

                    io::filtering_streambuf<io::output> out;
                    out.push(io::zlib_compressor(io::zlib::best_speed));
                    out.push(s);

                    //boost::archive::binary_oarchive oa(s);
                    boost::archive::binary_oarchive oa(out);
                    oa << data;
                    s.flush();
                }
                return std::vector<int8_t>(serial_str.begin(), serial_str.end());
            }

            static cv::Mat fromByteArray(std::vector<int8_t> byteVec) {
                cv::Mat _ocvMat;
                if (byteVec.size() == 0) {
                    return _ocvMat;
                }
                namespace io = boost::iostreams;
                membuf _membuf(reinterpret_cast<const char*> (byteVec.data()), byteVec.size());
                {
                    io::filtering_streambuf<io::input> in;
                    in.push(io::zlib_decompressor());
                    in.push(_membuf);

                    //boost::archive::binary_iarchive ia(_membuf);
                    boost::archive::binary_iarchive ia(in);
                    ia >> _ocvMat;
                }
                return _ocvMat;
            }

            static std::vector<cv::Mat> fromByteArray(std::vector<int8_t> byteVec, int numMats) {
                std::vector<cv::Mat> _ocvMatVec(numMats);
                if (byteVec.size() == 0) {
                    return _ocvMatVec;
                }
                namespace io = boost::iostreams;
                membuf _membuf(reinterpret_cast<const char*> (byteVec.data()), byteVec.size());
                {
                    io::filtering_streambuf<io::input> in;
                    in.push(io::zlib_decompressor());
                    in.push(_membuf);

                    boost::archive::binary_iarchive ia(in);
                    //boost::archive::binary_iarchive ia(_membuf);
                    //ia >> _ocvMat;
                    bool cont = true;
                    int matIdx = 0;
                    while (cont && matIdx < numMats) {
                        cont = boost::serialization::try_stream_next(ia, _ocvMatVec[matIdx++]);
                        //_ocvMatVec.push_back(_ocvMat);
                        //_ocvMat.release();
                    }
                }
                return _ocvMatVec;
            }

            static ocvMat::Ptr create() {
                return ocvMat::Ptr(boost::make_shared<ocvMat>());
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
                cameraInfo = CameraInfo::create();
            }

            virtual ~PlaneImage() {
            }

            static PlaneImage::Ptr create() {
                return PlaneImage::Ptr(boost::make_shared<PlaneImage>());
            }

            std::vector<int8_t> toByteArray() const {
                std::vector<int8_t> dataVec = ocvMat::toByteArray();
                std::vector<int8_t> infoVec = cameraInfo->toByteArray();
                dataVec.insert(dataVec.end(), infoVec.begin(), infoVec.end());
                return dataVec;
            }

            static PlaneImage::Ptr fromByteArray(std::vector<int8_t> byteVec) {
                std::vector<cv::Mat> _ocvMats = ocvMat::fromByteArray(byteVec, 2);
                PlaneImage::Ptr planeImage = create();
                planeImage->setData(_ocvMats[0]);
                planeImage->cameraInfo->setData(_ocvMats[1]);
                return planeImage;
            }

            void toQuantizedByteArray(std::vector<int8_t>& dataVec, PlaneImageQuantizer& myQuantizer) const {
                std::vector<int8_t> localData;
                localData.push_back((int8_t) ((data.rows & 0x0000ff00) >> 8));
                localData.push_back((int8_t) ((data.rows & 0x000000ff) >> 0));
                localData.push_back((int8_t) ((data.cols & 0x0000ff00) >> 8));
                localData.push_back((int8_t) ((data.cols & 0x000000ff) >> 0));
                myQuantizer.quantize(data, localData, 4);
                std::vector<int8_t> infoVec = cameraInfo->toByteArray();
                localData.insert(localData.end(), infoVec.begin(), infoVec.end());
                std::vector<int8_t> zlocalData = ::VectorCompressor::compress(localData);
                dataVec.insert(dataVec.end(), zlocalData.begin(), zlocalData.end());
            }

            static PlaneImage::Ptr fromQuantizedByteArray(const std::vector<int8_t>& byteVec, PlaneImageQuantizer& myQuantizer) {
                std::vector<int8_t> localData = ::VectorCompressor::decompress(byteVec);
                int rows = (uint8_t) localData[0];
                rows <<= 8;
                rows |= (uint8_t) localData[1];
                int cols = (uint8_t) localData[2];
                cols <<= 8;
                cols |= (uint8_t) localData[3];
                PlaneImage::Ptr planeImage = create();
                planeImage->setData(myQuantizer.dequantize(localData, rows, cols, 4));
                uint32_t vecPos = myQuantizer.getPos();
                std::vector<int8_t> remainder;
                remainder.insert(remainder.begin(), localData.begin() + vecPos, localData.end());
                planeImage->cameraInfo->setData(ocvMat::fromByteArray(remainder));
                return planeImage;
            }

            cv::Mat visualizeAsImage() const {
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
                cameraInfo = CameraInfo::create();
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
                cameraInfo = CameraInfo::create();
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

            cv::Mat visualizeAsImage() {
                cv::Mat labelImage(data.rows, data.cols, CV_8UC3); // label_data: [label; probability]
                // Color Mapping Method #1
                //cv::applyColorMap(label_img, labelImage, cv::COLORMAP_JET);
                //cv::imwrite("rgb_1.jpg", labelImage);

                // Color Mapping Method #2
                /*int labelNum = 37;
                cv::Mat lookupTable(1, 256, CV_8UC3);
                for(int i = 0; i < 256; i++)
                {
                    lookupTable.at<cv::Vec3b>(0, i) = cv::Vec3b(255 - i * int(255 / labelNum), 0, 255 + i * int(255 / labelNum));
                }          
                cv::Mat label3(label_img.rows, label_img.cols, CV_8UC3);
                std::vector<cv::Mat> channel3 = {label_img, label_img, label_img};
                cv::merge(channel3, label3);
                cv::LUT(label3, lookupTable, labelImage);  */

                // Color Mapping Method #3
                int golden_angle = (int) (180 * (3 - sqrt(5))) % 360;
                cv::Mat label_hue(data.rows, data.cols, CV_8UC1);
                for (int i = 0; i < data.rows; i++) {
                    for (int j = 0; j < data.cols; j++) {
                        label_hue.at<uchar>(i, j) = (int) (data.at<cv::Vec2f>(i, j)[0]) * golden_angle; // each label will be nearly maximally different in hue
                    }
                }
                cv::Mat blank(label_hue.rows, label_hue.cols, CV_8UC1, cv::Scalar(255));
                std::vector<cv::Mat> hsvChannels = {label_hue, blank, blank};
                cv::Mat hsvImage(label_hue.rows, label_hue.cols, CV_8UC3);
                cv::merge(hsvChannels, hsvImage);
                cv::cvtColor(hsvImage, labelImage, cv::COLOR_HSV2BGR);
                return labelImage;
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

