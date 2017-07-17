/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rgbd_image_uncc.hpp
 * Author: arwillis
 *
 * Created on July 2, 2017, 11:46 AM
 */

#ifndef RGBD_IMAGE_UNCC_HPP
#define RGBD_IMAGE_UNCC_HPP

#ifdef __cplusplus

#include <iostream>
#include <limits>
#include <unordered_map>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense> 
#include <Eigen/Eigenvalues> 

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>
#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>

// forward declaration for C++ compiler
//class cv::rgbd::DepthIntegralImages;

namespace cv {
    namespace rgbd {

        // Comment in/out CACHE_INVERSE to enable/disable caching of 
        // depth matrix inverses for explicit normal estimation. 
        // When active a significant speedup is obtained.
#define CACHE_INVERSE

        class DepthIntegralImages {
        private:
            cv::Mat ntan_theta_x, ntan_theta_y;
            cv::Mat tan_theta_x, tan_theta_y;
            cv::Mat tan_theta_xy, tan_theta_x_sq, tan_theta_y_sq;
            int width, height;
            cv::Size winSize;

            cv::Mat _tan_theta_x_div_z;
            cv::Mat _tan_theta_y_div_z;
            cv::Mat _one_div_z;
            cv::Mat _one_div_z_sq;
            cv::Mat _ones_valid_mask;

            cv::Mat numValidPts;
            cv::Mat tan_theta_x_div_z;
            cv::Mat tan_theta_y_div_z;
            cv::Mat one_div_z;
            cv::Mat one_div_z_sq;

            cv::Mat dz_dx, dz_dy, d2z_dx2, d2z_dxdy, d2z_dy2;
            cv::Mat idz_dx, idz_dy, id2z_dx2, id2z_dxdy, id2z_dy2;

            std::unordered_map<int, cv::Mat> matMap;
        public:

            class ImageWindow {
                cv::Size winSize;
                cv::Size imSize;
                cv::Point2i _begin, _end;
            public:
                int tlc, trc, blc, brc, ctr;

                ImageWindow(cv::Size _imSize, cv::Size _winSize) :
                imSize(_imSize),
                winSize(_winSize),
                _begin(winSize.width >> 1, winSize.height >> 1),
                _end(imSize.width - _begin.x, imSize.height - _begin.y) {
                    centerOn(winSize.width >> 1, winSize.height >> 1);
                }

                void shiftRight() {
                    tlc++;
                    trc++;
                    blc++;
                    brc++;
                    ctr++;
                }

                void centerOn(int x, int y) {
                    tlc = y * imSize.width + x;
                    trc = tlc + winSize.width;
                    blc = tlc + winSize.height * imSize.width;
                    brc = blc + winSize.width;
                    ctr = (y + (winSize.height >> 1)) * imSize.width + (winSize.width >> 1);
                }

                Point2i* end() {
                    return &_end;
                }

                Point2i* begin() {
                    return &_begin;
                }
            };

            typedef boost::shared_ptr<DepthIntegralImages> Ptr;

            void computeMat_LUT();
            void initialize(int _width, int _height, float _cx, float _cy, float _inv_f,
                    cv::Size _winSize);

            void computeImplicit_Impl(const cv::Mat& depth, cv::Mat & normals);

            void computeExplicit_Impl(const cv::Mat& depth, cv::Mat & normals);

            bool computeNormalExplicit(cv::Rect roi, const
                    cv::Mat& depth, cv::Vec3f& normals);

            void operator()(cv::InputArray depth_in, cv::OutputArray normals_out);

            void computeCurvatureFiniteDiff_Impl(const cv::Mat& depth, cv::Mat& normals);

            void plucker(cv::Mat& points, cv::Mat& normals,
                    cv::Mat& axisVecs, cv::Mat& axisDirs);

            int getWidth() {
                return width;
            }

            int getHeight() {
                return height;
            }

            int setWindowSize(cv::Size& _winSize) {
                winSize = _winSize;
            }

            cv::Size getWindowSize() {
                return winSize;
            }

            template<typename T>
            void computeIntegralImage(cv::Mat& input, cv::Mat& integral_im) const {
                T* integral_image = (T *) integral_im.data;
                int rows = input.rows;
                int cols = input.cols;
                const float* input_ptr = (const float *) input.data;
                T* integral_image_ptr = integral_image;
                T* integral_image_lu_ptr = integral_image;
                for (int x = 0; x < cols + 1; ++x) { // first row of zeros
                    *integral_image_ptr++ = 0;
                }
                T left_value = 0, top_left_value = 0, top_value = 0;
                T* integral_image_l_ptr;
                for (int y = 0; y < static_cast<int> (rows); ++y) {
                    integral_image_l_ptr = integral_image_ptr;
                    *integral_image_ptr++ = 0; // first column is zero
                    for (int x = 0; x < static_cast<int> (cols); ++x) {
                        left_value = *(integral_image_l_ptr++);
                        top_left_value = *(integral_image_lu_ptr++);
                        top_value = *(integral_image_lu_ptr);
                        T& integral_pixel = *(integral_image_ptr++);
                        integral_pixel = static_cast<T> (*input_ptr++);
                        integral_pixel += left_value + top_value - top_left_value;
                    }
                    integral_image_lu_ptr++;
                }
            }

            template<typename T>
            T getSum(cv::Mat iImg, cv::Rect2i& rect) const {
                return iImg.at<T>(rect.y, rect.x) + iImg.at<T>(rect.y + rect.height, rect.x + rect.width) -
                        iImg.at<T>(rect.y + rect.height, rect.x) - iImg.at<T>(rect.y, rect.x + rect.width);
            }
        }; /*class DepthIntegralImages */

        class RgbdImage {
        public:
            static constexpr float DEPTH_NOISE_CONSTANT = (1.425e-3f);

            typedef boost::shared_ptr<RgbdImage> Ptr;

            RgbdImage() {
            }

            RgbdImage(const cv::Mat &_img_I, const cv::Mat &_img_Z, // Mat& _img_L, 
                    float _cx, float _cy, float _f) :
            img_I(_img_I), img_Z(_img_Z), // img_L(_img_L), 
            cx(_cx), cy(_cy), inv_f(1.f / _f),
            width(_img_I.cols), height(_img_I.rows) {
                iptr = img_I.ptr<uchar>();
                zptr = img_Z.ptr<float>();
                //lptr = img_L.ptr<uchar>();
                istep = (int) (_img_I.step / _img_I.elemSize1());
                zstep = (int) (_img_Z.step / _img_Z.elemSize1());
                //lstep = (int) (_img_L.step / _img_L.elemSize1());
                cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);
                cameraMatrix.at<float>(0, 0) = _f;
                cameraMatrix.at<float>(1, 1) = _f;
                cameraMatrix.at<float>(2, 2) = 1.0f;
                cameraMatrix.at<float>(0, 2) = _cx;
                cameraMatrix.at<float>(1, 2) = _cy;
                if (!staticDataOK()) {
                    iImgs.initialize(width, height, cx, cy, inv_f, cv::Size(5, 5));
                }
            }

            virtual ~RgbdImage() {
            };

            bool staticDataOK();

            bool getPoint3f(const int ix, const int iy,
                    float& x3, float& y3, float& z3) const {
                z3 = zptr[iy * zstep + ix];
                x3 = (ix - cx) * z3 * inv_f;
                y3 = (iy - cy) * z3 * inv_f;
                return std::isnan(z3) ? false : true;
            }

            bool getPoint3f(const int ix, const int iy, Point3f& p3) const {
                return getPoint3f(ix, iy, p3.x, p3.y, p3.z);
            }

            void getPointCloud(cv::Mat& pts, cv::Mat& colors);

            bool getTileData_Random(int x0, int y0,
                    int loc_width, int loc_height,
                    std::vector<Point3f>& data, int numSamples = -1) const {
                //            if (x0 + loc_width > width) {
                //                loc_width = width - x0;
                //            }
                //            if (y0 + loc_height > height) {
                //                loc_height = height - y0;
                //            }
                int tileArea = loc_width*loc_height;
                if (numSamples > tileArea || numSamples <= 0) {
                    return false;
                }
                if (numSamples == -1) {
                    int decimateFactor = 1;
                    do {
                        numSamples = tileArea / decimateFactor;
                        decimateFactor >>= 2;
                    } while (numSamples > 50);
                }
                Point3f pt3;
                RNG rng(0x34985739);
                int ix, iy, numIterations = 0;
                do {
                    ix = x0 + rng.uniform(0, loc_width); // uniform [x0,x0+width-1]
                    iy = y0 + rng.uniform(0, loc_height); // uniform [y0,y0+height-1]
                    const float& depth = zptr[(iy * zstep) + ix];
                    if (!std::isnan(depth)) {
                        getPoint3f(ix, iy, pt3);
                        data.push_back(pt3);
                        //std::cout << "(x,y)=(" << ix << ", " << iy << ")"
                        //        << " (X,Y,Z)=" << pt3 << " data.size = " << data.size() << std::endl;
                    }
                } while (data.size() < numSamples && ++numIterations < 3 * numSamples);
                return (data.size() == numSamples) ? true : false;
            }

            bool getTileData_Uniform(int x0, int y0,
                    int width, int height,
                    std::vector<Point3f>& data, int numSamples = -1) const {
                int tileArea = width*height;
                float aspectRatio = (float) width / (float) height;
                float downSample = ((float) tileArea) / numSamples;
                float xFactor = downSample * aspectRatio;
                float yFactor = downSample / aspectRatio;
                if (numSamples > tileArea || numSamples <= 0) {
                    return false;
                }
                if (numSamples == -1) {
                    int decimateFactor = 1;
                    do {
                        numSamples = tileArea / decimateFactor;
                        decimateFactor >>= 2;
                    } while (numSamples > 50);
                }
                cv::Point2i skip(std::floor(xFactor), std::floor(yFactor));
                Point3f pt3;
                int offset = 0;
                for (int iy = y0 + offset; iy < y0 + height && data.size() < numSamples; iy += skip.y) {
                    for (int ix = x0 + offset; ix < x0 + width && data.size() < numSamples; ix += skip.x) {
                        const float& depth = zptr[iy * zstep + ix];
                        if (!std::isnan(depth)) {
                            getPoint3f(ix, iy, pt3);
                            data.push_back(pt3);
                            //std::cout << "(x,y)=(" << ix << ", " << iy << ")"
                            //        << " (X,Y,Z)=" << pt3 << " data.size = "
                            //        << data.size() << " numSamples =" << numSamples << std::endl;
                        }
                    }
                }
                return (data.size() == numSamples) ? true : false;
            }


            bool fitPlane(RectWithError& r, Plane3f& plane3) const;

            Point2f project(Point3f p3) const {
                Point2f p2;
                p2.x = p3.x / (p3.z * inv_f) + cx;
                p2.y = p3.y / (p3.z * inv_f) + cy;
                return p2;
            }

            Point3f backproject(Point2f p2) const {
                Point3f p3;
                int ix = cvRound(p2.x);
                int iy = cvRound(p2.y);
                p3.x = (p2.x - cx) * zptr[iy * zstep + ix] * inv_f;
                p3.y = (p2.y - cy) * zptr[iy * zstep + ix] * inv_f;
                p3.z = zptr[iy * zstep + ix];
                return p3;
            }

            Line2f project(Line3f line3) const {
                Line2f line2;
                line2.v.x = line3.v.x;
                line2.v.y = line3.v.y;
                //float normFactor = 1.0f / sqrt(line2.v.x * line2.v.x + line2.v.y * line2.v.y);
                //line2.v.x *= normFactor;
                //line2.v.y *= normFactor;
                line2.p0.x = (line3.p0.x / inv_f) + cx;
                line2.p0.y = (line3.p0.y / inv_f) + cy;
                return line2;
            }

            void computeDepthImageGradientSobel(int x0, int y0, float& dZ_dx, float& dZ_dy) const {
                const float* z_ptr0 = zptr + y0 * zstep + x0;
                dZ_dx = (z_ptr0[1] - z_ptr0[-1])*2
                        + (z_ptr0[-zstep + 1] - z_ptr0[-zstep - 1])
                        + (z_ptr0[zstep + 1] - z_ptr0[zstep - 1]);
                dZ_dy = (z_ptr0[zstep] - z_ptr0[-zstep])*2
                        + (z_ptr0[zstep - 1] - z_ptr0[-zstep - 1])
                        + (z_ptr0[zstep + 1] - z_ptr0[-zstep + 1]);
            }

            LineSegment2f getVisibleLineSegment2f(Line3f& line3) const;

            bool isInlier(const float error, const float& z3) const {
                return (error < getErrorStandardDeviation(z3));
            }

            float getErrorStandardDeviation(const RectWithError& re) const {
                return getErrorStandardDeviation(re.x + (width / 2), re.y + (height / 2));
            }

            float getErrorStandardDeviation(const float& z3) const {
                return DEPTH_NOISE_CONSTANT * z3 * z3;
            }

            float getErrorStandardDeviation(int ix, int iy) const {
                return getErrorStandardDeviation(zptr[iy * zstep + ix]);
            }

            void fitImplicitPlaneLeastSquares(std::vector<Point3f>& data3, Plane3f& plane3, float& error,
                    float& noise, int& inliers, int& outliers, int& invalid) const;

            const cv::Mat getDepth() const {
                return img_Z;
            }

            const cv::Mat getRGB() {
                return img_I;
            }

            const cv::Mat getCameraMatrix() {
                return cameraMatrix;
            }

            void setNormalsComputer(cv::Ptr<RgbdNormals> _nc);

            cv::Ptr<RgbdNormals> getNormalsComputer(cv::Ptr<RgbdNormals> _nc);

            bool computeNormals() const;

            int getWidth() {
                return width;
            }

            int getHeight() {
                return height;
            }
        private:
            // -------------------------
            // Disabling default copy constructor and default
            // assignment operator.
            // -------------------------
            RgbdImage(const RgbdImage & ref);
            RgbdImage& operator=(const RgbdImage & ref);

            cv::Mat img_Z;
            const float* zptr;
            int zstep;
            cv::Mat img_I;
            const uchar* iptr;
            int istep;

            //        Mat img_L;
            //        const uchar* lptr;
            //        int lstep;

            int width, height;
            float cx, cy, inv_f;
            cv::Mat cameraMatrix;
            mutable cv::Ptr<RgbdNormals> normalsComputer;
        public:
            static DepthIntegralImages iImgs;
        }; /* class RgbdImage */
    } /* namespace rgbd */
} /* namespace cv */

#endif /* __cplusplus */

#endif /* RGBD_IMAGE_UNCC_HPP */

