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

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>
#include <ros_rgbd_surface_tracker/surface_fitting.hpp>
#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>

#define OPENCV_TRAITS_ENABLE_DEPRECATED
// forward declaration for C++ compiler
//class cv::rgbd::DepthIntegralImages;

namespace cv {
    namespace rgbd {

        /* Implementation in surface_alignment_optimizer.cpp */
        int planeListAlignmentCV(std::vector<cv::Plane3f::Ptr>& moving_planes,
                std::vector<cv::Plane3f::Ptr>& fixed_planes,
                Eigen::Matrix4f& transformation_matrix);

        void benchmarkPointAlignment();

        float minimizePointToPointError(std::vector<cv::Point3f>& fixed_points,
                std::vector<cv::Point3f>& moving_points,
                Pose& delta_pose);

        // Comment in/out CACHE_INVERSE to enable/disable caching of 
        // depth matrix inverses for explicit normal estimation. 
        // When active a significant speedup is obtained.
#define CACHE_INVERSE

        class CV_EXPORTS_W DepthIntegralImages {
        private:
            cv::Mat tan_theta_x, tan_theta_y;
            cv::Mat ii_tan_theta_x, ii_tan_theta_y;
            cv::Mat ii_tan_theta_xy, ii_tan_theta_x_sq, ii_tan_theta_y_sq;
            int width, height;
            cv::Size winSize;

            cv::Mat _tan_theta_x_div_z;
            cv::Mat _tan_theta_y_div_z;
            cv::Mat _one_div_z;
            cv::Mat _one_div_z_sq;
            cv::Mat _ones_valid_mask;

            cv::Mat ii_numValidPts;
            cv::Mat ii_tan_theta_x_div_z;
            cv::Mat ii_tan_theta_y_div_z;
            cv::Mat ii_one_div_z;
            cv::Mat ii_one_div_z_sq;

            cv::Mat dz_dx, dz_dy, d2z_dx2, d2z_dxdy, d2z_dy2;
            cv::Mat idz_dx, idz_dy, id2z_dx2, id2z_dxdy, id2z_dy2;

            std::unordered_map<int, cv::Mat> matMap;
        public:

            class CV_EXPORTS_W ImageWindow {
                cv::Size winSize;
                cv::Size imSize;
                cv::Point2i _begin, _end;
            public:
                int tlc, trc, blc, brc, ctr;

                CV_WRAP ImageWindow(cv::Size _imSize, cv::Size _winSize) :
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
                    tlc = (y - (winSize.height >> 1)) * imSize.width + (x - (winSize.width >> 1));
                    trc = tlc + winSize.width;
                    blc = tlc + winSize.height * imSize.width;
                    brc = blc + winSize.width;
                    ctr = (y + (winSize.height >> 1)) * imSize.width + (x + (winSize.width >> 1));
                }

                cv::Point2i* end() {
                    return &_end;
                }

                cv::Point2i* begin() {
                    return &_begin;
                }
            };

            typedef boost::shared_ptr<DepthIntegralImages> Ptr;

            void computeMat_LUT();
            
            CV_WRAP void initialize(int _width, int _height, float _cx, float _cy, float _inv_f,
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

            void setWindowSize(cv::Size& _winSize) {
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

        class CV_EXPORTS_W RgbdImage {
        public:
            static constexpr float DEPTH_NOISE_CONSTANT = (1.425e-3f);

            typedef boost::shared_ptr<RgbdImage> Ptr;

            RgbdImage() {
            }

            CV_WRAP RgbdImage(const cv::Mat &_img_I, const cv::Mat &_img_Z, // Mat& _img_L, 
                    float _cx, float _cy, float _f) :
            img_I(_img_I), img_Z(_img_Z), // img_L(_img_L), 
            cx(_cx), cy(_cy), inv_f(1.f / _f),
            width(_img_I.cols), height(_img_I.rows) {
                iptr = img_I.ptr<uchar>(0, 0);
                zptr = img_Z.ptr<float>(0, 0);
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
                    initializeIntegralImages(cv::Size(5, 5));
                    //iImgs.initialize(width, height, cx, cy, inv_f, cv::Size(5, 5));
                }
            }

            virtual ~RgbdImage() {
            };

            CV_WRAP void initializeIntegralImages(cv::Size _winSize) {
                iImgs.initialize(width, height, cx, cy, inv_f, _winSize);
            }
            
            bool staticDataOK();

            bool isContinuous() const {
                // Checks color and depth cv::Mats for memory continuity
                return (this->img_I.isContinuous() and this->img_Z.isContinuous());
            }

            bool getPoint3f(const int ix, const int iy,
                    float& x3, float& y3, float& z3) const {
                z3 = zptr[iy * zstep + ix];
                x3 = (ix - cx) * z3 * inv_f;
                y3 = (iy - cy) * z3 * inv_f;
                return std::isnan(z3) ? false : true;
            }

            bool getPoint3f(const int ix, const int iy, cv::Point3f& p3) const {
                return getPoint3f(ix, iy, p3.x, p3.y, p3.z);
            }

            bool getPoint3f(const int ix, const int iy, cv::Vec3f& v) const {
                return getPoint3f(ix, iy, v[0], v[1], v[2]);
            }

            void getPointCloud(cv::Mat& pts, cv::Mat& colors) const;

            void getPointCloud(cv::Mat& pts) const;

            bool getTileData_Random(int x0, int y0,
                    int loc_width, int loc_height,
                    std::vector<cv::Point3f>& data, int numSamples = -1) const {
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

            void reproject(Pose camera_pose, RgbdImage& newImg) const {
                newImg.width = width;
                newImg.height = height;
                newImg.cameraMatrix = cameraMatrix.clone();
                newImg.cx = cx;
                newImg.cy = cy;
                newImg.inv_f = inv_f;
                float fNaN = std::numeric_limits<float>::quiet_NaN();
                newImg.img_Z = cv::Mat(height, width, CV_32F, cv::Scalar(fNaN));
                newImg.img_I = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
                newImg.iptr = newImg.img_I.ptr<uchar>(0, 0);
                newImg.zptr = newImg.img_Z.ptr<float>(0, 0);
                newImg.istep = (int) (newImg.img_I.step / newImg.img_I.elemSize1());
                newImg.zstep = (int) (newImg.img_Z.step / newImg.img_Z.elemSize1());
                cv::Vec3f t;
                camera_pose.getTranslation(t);
                cv::Matx33f R = camera_pose.getRotation_Matx33();
                const float *cur_z_ptr;
                float *new_z_ptr;
                Point3f p3d;
                Point2f p2d, p2dtest;
                for (int y = 0; y < height; ++y) {
                    cur_z_ptr = img_Z.ptr<float>(y, 0);
                    for (int x = 0; x < width; ++x, ++cur_z_ptr) {
                        if (!std::isnan(*cur_z_ptr)) {

                            getPoint3f(x, y, p3d);
                            float zval = p3d.z;
                            p3d = R * (cv::Vec3f) p3d + t;
                            p2d = project(p3d);
                            //std::cout << "p2d: " << p2d << std::endl;

                            //computeTarget(x, y, zval, camera_pose, p2dtest.x, p2dtest.y);
                            //std::cout << "p2d test: " << p2dtest << std::endl;
                            p2d.x = std::round(p2d.x);
                            p2d.y = std::round(p2d.y);
                            if (p2d.y >= 0 && p2d.y < height && p2d.x >= 0 && p2d.x < width) {
                                new_z_ptr = newImg.img_Z.ptr<float>(p2d.y, p2d.x);
                                if (std::isnan(*new_z_ptr) || *new_z_ptr > p3d.z) {
                                    *new_z_ptr = p3d.z;
                                    newImg.img_I.at<Vec3b>(p2d.y, p2d.x) = img_I.at<Vec3b>(y, x);
                                }
                            }
                        }
                    }
                }
            }

            void computeTarget(float x_im, float y_im, float z, const Pose pose,
                    float& x_im_prime, float& y_im_prime) {

                cv::Vec3f rVec, tVec;
                pose.get(tVec, rVec);
                float vx = rVec[0];
                float vy = rVec[1];
                float vz = rVec[2];
                float tx = tVec[0];
                float ty = tVec[1];
                float tz = tVec[2];
                float f = 1.0 / this->inv_f;

                float vx_sq = vx*vx;
                float vy_sq = vy*vy;
                float vz_sq = vz*vz;
                float theta_sq = vx_sq + vy_sq + vz_sq;
                float theta = std::sqrt(theta_sq);
                float cos_theta_m1 = cos(theta) - 1;
                float sin_theta = sin(theta);

                x_im_prime = cx + (f * (tx + z * ((vy * sin_theta) / theta - (vx * vz * (cos_theta_m1)) / theta_sq) - (z * (cx - x_im)*((vy_sq * (cos_theta_m1)) / theta_sq + (vz_sq * (cos_theta_m1)) / theta_sq + 1)) / f + (z * ((vz * sin_theta) / theta + (vx * vy * (cos_theta_m1)) / theta_sq)*(cy - y_im)) / f)) / (tz + z * ((vx_sq * (cos_theta_m1)) / theta_sq + (vy_sq * (cos_theta_m1)) / theta_sq + 1) + (z * ((vy * sin_theta) / theta + (vx * vz * (cos_theta_m1)) / theta_sq)*(cx - x_im)) / f - (z * ((vx * sin_theta) / theta - (vy * vz * (cos_theta_m1)) / theta_sq)*(cy - y_im)) / f);
                y_im_prime = cy - (f * (z * ((vx * sin_theta) / theta + (vy * vz * (cos_theta_m1)) / theta_sq) - ty + (z * (cy - y_im)*((vx_sq * (cos_theta_m1)) / theta_sq + (vz_sq * (cos_theta_m1)) / theta_sq + 1)) / f + (z * ((vz * sin_theta) / theta - (vx * vy * (cos_theta_m1)) / theta_sq)*(cx - x_im)) / f)) / (tz + z * ((vx_sq * (cos_theta_m1)) / theta_sq + (vy_sq * (cos_theta_m1)) / theta_sq + 1) + (z * ((vy * sin_theta) / theta + (vx * vz * (cos_theta_m1)) / theta_sq)*(cx - x_im)) / f - (z * ((vx * sin_theta) / theta - (vy * vz * (cos_theta_m1)) / theta_sq)*(cy - y_im)) / f);

            }

            bool getTileData_Uniform(int x0, int y0,
                    int width, int height,
                    std::vector<cv::Point3f>& data, int numSamples = -1) const {
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
                p2.x = (p3.x / (p3.z * inv_f)) + cx;
                p2.y = (p3.y / (p3.z * inv_f)) + cy;
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

            void invalidate_NaN_Neighbors() {
                float invalid = std::numeric_limits<float>::quiet_NaN();
                float infinity = std::numeric_limits<float>::infinity();
                for (int iy = 0; iy < img_Z.rows; ++iy) {
                    float *z_cur = img_Z.ptr<float>(iy, 0);
                    for (int ix = 0; ix < img_Z.cols; ++ix, ++z_cur) {
                        if (std::isnan(*z_cur)) {
                            if (iy > 0) {
                                *(z_cur - zstep) = infinity;
                            }
                            if (iy < img_Z.rows - 1) {
                                *(z_cur + zstep) = infinity;
                            }
                            if (ix > 0) {
                                *(z_cur - 1) = infinity;
                            }
                            if (ix < img_Z.cols - 1) {
                                *(z_cur + 1) = infinity;
                            }
                        }
                    }
                }
                for (int iy = 0; iy < img_Z.rows; ++iy) {
                    float *z_cur = img_Z.ptr<float>(iy, 0);
                    for (int ix = 0; ix < img_Z.cols; ++ix, ++z_cur) {
                        if (*z_cur == infinity) {
                            *z_cur = invalid;
                        }
                    }
                }
            }

            bool inImage(const float& px, const float& py) const {
                // checks that the pixel is within the image bounds
                return (py >= 0 && py < this->height && px >= 0 && px < this->width);
            }

            bool inImage(cv::Point2f& pixel) const {
                // checks that the pixel is within the image bounds
                return this->inImage(pixel.x, pixel.y);
            }

            LineSegment2f getVisibleLineSegment2f(Line3f & line3) const;

            bool isInlier(const float error, const float& z3) const {
                return (error < getErrorStandardDeviation(z3));
            }

            float getErrorStandardDeviation(const RectWithError & re) const {
                return getErrorStandardDeviation(re.x + (width / 2), re.y + (height / 2));
            }

            void checkVisibility(cv::Vec3f pt, cv::Rect r) {


            }

            float getErrorStandardDeviation(const float& z3) const {
                return DEPTH_NOISE_CONSTANT * z3 * z3;
            }

            float getErrorStandardDeviation(int ix, int iy) const {
                return getErrorStandardDeviation(zptr[iy * zstep + ix]);
            }

            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneImplicitLeastSquares(const std::vector<cv::Point3_<scalar_t>>&points) {
                return cv::uncc::SurfaceFitting::fitPlaneImplicitLeastSquares(points);
            }

            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneImplicitLeastSquares(const Eigen::Matrix<scalar_t, 4, Eigen::Dynamic, Eigen::ColMajor>& points_homogeneous) {
                return cv::uncc::SurfaceFitting::fitPlaneImplicitLeastSquares(points_homogeneous);
            }

            template <typename scalar_t>
            std::pair<cv::Plane3_<scalar_t>, cv::FitStatistics> fitPlaneImplicitLeastSquaresWithStats(const std::vector<cv::Point3_<scalar_t>>&points) const {
                return this->fitPlaneImplicitLeastSquaresWithStats(reinterpret_cast<const scalar_t*> (points.data()), points.size(), 3);
            }

            template <typename scalar_t>
            std::pair<cv::Plane3_<scalar_t>, cv::FitStatistics> fitPlaneImplicitLeastSquaresWithStats(const Eigen::Matrix<scalar_t, 4, Eigen::Dynamic, Eigen::ColMajor>& points_homogeneous) const {
                return this->fitPlaneImplicitLeastSquaresWithStats(points_homogeneous.data(), points_homogeneous.cols(), 4);
            }

            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneExplicitLeastSquares(const std::vector<cv::Point3_<scalar_t>>&points) {
                return cv::uncc::SurfaceFitting::fitPlaneExplicitLeastSquares(points);
            }
            
            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneExplicitLeastSquares(const Eigen::Matrix<scalar_t, 4, Eigen::Dynamic, Eigen::ColMajor>& points_homogeneous) {
                return cv::uncc::SurfaceFitting::fitPlaneExplicitLeastSquares(points_homogeneous);
            }

            template <typename scalar_t>
            std::pair<cv::Plane3_<scalar_t>, cv::FitStatistics> fitPlaneExplicitLeastSquaresWithStats(const std::vector<cv::Point3_<scalar_t>>&points) const {
                return this->fitPlaneExplicitLeastSquaresWithStats(reinterpret_cast<const scalar_t*> (points.data()), points.size(), 3);
            }
            
            template <typename scalar_t>
            std::pair<cv::Plane3_<scalar_t>, cv::FitStatistics> fitPlaneExplicitLeastSquaresWithStats(const Eigen::Matrix<scalar_t, 4, Eigen::Dynamic, Eigen::ColMajor>& points_homogeneous) const {
                return this->fitPlaneExplicitLeastSquaresWithStats(points_homogeneous.data(), points_homogeneous.cols(), 4);
            }
            
            std::pair<cv::Plane3f, cv::FitStatistics> fitPlaneRANSACWithStats(pcl::PointCloud<pcl::PointXYZ>::Ptr points, double distance_threshold = .01, size_t max_iterations = 1000, bool refine = true) const {

                pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model =
                        boost::make_shared<pcl::SampleConsensusModelPlane < pcl::PointXYZ >> (points);
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);

                ransac.setDistanceThreshold(distance_threshold);
                ransac.setMaxIterations(max_iterations);
                ransac.computeModel();

                if (refine) {
                    ransac.refineModel();
                }

                Eigen::VectorXf coeffs;
                ransac.getModelCoefficients(coeffs);
                std::vector<int> inlier_indicies;
                ransac.getInliers(inlier_indicies);

                cv::Plane3f plane;
                plane.x = coeffs[0];
                plane.y = coeffs[1];
                plane.z = coeffs[2];
                plane.d = coeffs[3];
                plane.scale((plane.z > 0) ? -1.0 : 1.0);

                cv::FitStatistics stats;
                stats.inliers = inlier_indicies.size();
                stats.outliers = points->size() - stats.inliers;
                for (int inlier_index : inlier_indicies) {
                    const pcl::PointXYZ& point = points->at(inlier_index);
                    float point_error = std::abs(plane.evaluate(point.x, point.y, point.z));
                    stats.error += point_error;
                    stats.noise += this->getErrorStandardDeviation(point.z);
                    if (stats.max_error < stats.error) {
                        stats.max_error = stats.error;
                    }
                }
                stats.error /= stats.inliers;
                stats.noise /= stats.inliers;

                return std::make_pair(plane, stats);

            }

            const cv::Mat getDepthImage() const {
                return img_Z;
            }

            const cv::Mat getRGBImage() const {
                return img_I;
            }

            const cv::Mat getCameraMatrix() const {
                return cameraMatrix;
            }

            void setNormalsComputer(cv::Ptr<RgbdNormals> _nc);

            cv::Ptr<RgbdNormals> getNormalsComputer(cv::Ptr<RgbdNormals> _nc);

            void setNormals(cv::Mat & _normals) {
                img_N = _normals;
            }

            CV_WRAP cv::Mat getNormals() const {
                return img_N;
            }

            CV_WRAP double computeNormals(int method = 0);

            void setPlanes(cv::Mat & _planes) {
                img_P = _planes;
            }

            CV_WRAP cv::Mat getPlanes() const {
                return img_P;
            }

            CV_WRAP double computePlanes(int method = 0);

            const float& getDepth(int x, int y) const {
                return zptr[y * zstep + x];
            }

            int getWidth() const {
                return width;
            }

            int getHeight() const {
                return height;
            }

            static RgbdImage::Ptr create(const cv::Mat &_img_I, const cv::Mat &_img_Z, // Mat& _img_L, 
                    float _cx, float _cy, float _f) {
                return RgbdImage::Ptr(boost::make_shared<RgbdImage>(_img_I, _img_Z, _cx, _cy, _f));
            }
        private:
            // -------------------------
            // Disabling default copy constructor and default
            // assignment operator.
            // -------------------------
            RgbdImage(const RgbdImage& ref);
            RgbdImage& operator=(const RgbdImage& ref);

            cv::Mat img_Z;
            const float* zptr;
            int zstep;
            cv::Mat img_I;
            cv::Mat img_N;
            cv::Mat img_P;
            const uchar* iptr;
            int istep;

            //        Mat img_L;
            //        const uchar* lptr;
            //        int lstep;

            int width, height;
            float cx, cy, inv_f;
            cv::Mat cameraMatrix;
            mutable cv::Ptr<RgbdNormals> normalsComputer;

            template <typename scalar_t>
            std::pair<cv::Plane3_<scalar_t>, FitStatistics> fitPlaneImplicitLeastSquaresWithStats(const scalar_t* points, size_t num_points, size_t stride) const {

                cv::Plane3_<scalar_t> plane3;
                cv::FitStatistics stats;

                //cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::traits::Type<scalar_t>::value);
                cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::DataType<scalar_t>::type);
                scalar_t* M = _M.ptr<scalar_t>();
                cv::Point3f centroid(0, 0, 0);
                for (size_t ptIdx = 0; ptIdx < num_points; ++ptIdx) {
                    size_t pt_begin = stride*ptIdx;
                    centroid.x += points[pt_begin];
                    centroid.y += points[pt_begin + 1];
                    centroid.z += points[pt_begin + 2];
                }
                centroid.x /= num_points;
                centroid.y /= num_points;
                centroid.z /= num_points;
                for (size_t ptIdx = 0; ptIdx < num_points; ++ptIdx) { // remove centroid -> make coefficients uncorrelated
                    size_t pt_begin = stride*ptIdx;
                    M[3 * ptIdx] = points[pt_begin] - centroid.x;
                    M[3 * ptIdx + 1] = points[pt_begin + 1] - centroid.y;
                    M[3 * ptIdx + 2] = points[pt_begin + 2] - centroid.z;
                    stats.noise += this->getErrorStandardDeviation(points[pt_begin + 2]);
                }
                cv::Mat _MtM = _M.t() * _M;
                cv::Mat eigVals, eigVecs;
                cv::eigen(_MtM, eigVals, eigVecs);
                cv::Mat _planeCoeffs = eigVecs.row(2).t();
                //cv::Mat _planeCoeffsCovariance = eigVals.at(2) * _MtM;
                //std::cout << "E = " << E << std::endl;
                //std::cout << "MtM = " << _MtM << std::endl;          
                //std::cout << "V = " << eigVecs << std::endl;          
                //std::cout << "coeffs = " << _planeCoeffs << std::endl; 
                plane3.x = _planeCoeffs.at<scalar_t>(0);
                plane3.y = _planeCoeffs.at<scalar_t>(1);
                plane3.z = _planeCoeffs.at<scalar_t>(2);
                scalar_t d3 = -(plane3.x * centroid.x + plane3.y * centroid.y + plane3.z * centroid.z);
                plane3.d = d3;
                //std::cout << "centroid_dist = " << plane3.evaluate(centroid) << std::endl;
                //cv::Mat _D = cv::Mat::ones(num_points, 1, cv::traits::Type<scalar_t>::value);
                cv::Mat _D = cv::Mat::ones(num_points, 1, cv::DataType<scalar_t>::type);
                _D *= plane3.d;
                for (size_t ptIdx = 0; ptIdx < num_points; ++ptIdx) {
                    M[3 * ptIdx] += centroid.x;
                    M[3 * ptIdx + 1] += centroid.y;
                    M[3 * ptIdx + 2] += centroid.z;
                }
                cv::Mat _error = _M * _planeCoeffs + _D;
                //std::cout << "plane.d =" << plane3.d << " D=" << _D << std::endl;
                //std::cout << "I_error=" << _error << std::endl;
                //std::cout << "M = " << M << std::endl;  [0.588991, 0.423888, -0.688047, 1.82959]
                //std::cout << "Z = " << Z << std::endl;
                //scalar_t* _z = _Z.ptr<scalar_t>();
                scalar_t* _err = _error.ptr<scalar_t>();

                for (size_t ptIdx = 0; ptIdx < num_points; ptIdx++) {
                    if (_err[ptIdx] < getErrorStandardDeviation(points[stride * ptIdx + 2])) {
                        stats.inliers++;
                    } else {
                        stats.outliers++;
                    }
                    stats.error += std::abs(_err[ptIdx]);
                    if (stats.max_error < stats.error) {
                        stats.max_error = stats.error;
                    }
                }
                stats.error /= num_points;
                stats.noise /= num_points;
                plane3.scale((plane3.z > 0) ? -1.0 : 1.0);
                //std::cout << "iplaneCoeffs = " << plane3 << " error = " << error << std::endl;
                //std::cout.flush();

                return std::make_pair(plane3, stats);

            }

            template <typename scalar_t>
            std::pair<cv::Plane3_<scalar_t>, cv::FitStatistics> fitPlaneExplicitLeastSquaresWithStats(
                    const scalar_t* points, size_t num_points, size_t stride) const {

                cv::Plane3_<scalar_t> plane3;
                cv::FitStatistics stats;

                //cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::traits::Type<scalar_t>::value);
                //cv::Mat _Z = cv::Mat::zeros(num_points, 1, cv::traits::Type<scalar_t>::value);
                cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::DataType<scalar_t>::type);
                cv::Mat _Z = cv::Mat::zeros(num_points, 1, cv::DataType<scalar_t>::type);
                scalar_t* M = _M.ptr<scalar_t>();
                scalar_t* Z = _Z.ptr<scalar_t>();

                for (size_t ptIdx = 0; ptIdx < num_points; ++ptIdx) {
                    size_t pt_begin = stride*ptIdx;
                    M[3 * ptIdx] = points[pt_begin];
                    M[3 * ptIdx + 1] = points[pt_begin + 1];
                    M[3 * ptIdx + 2] = 1.0;
                    Z[ptIdx] = points[pt_begin + 2];
                    stats.noise += this->getErrorStandardDeviation(points[pt_begin + 2]);
                }
                cv::Mat _MtM = _M.t() * _M;
                cv::Mat _planeCoeffs = _MtM.inv() * _M.t() * _Z;
                scalar_t* planeCoeffs = _planeCoeffs.ptr<scalar_t>();
                cv::Mat _error = _M * _planeCoeffs - _Z;
                scalar_t* _z = _Z.ptr<scalar_t>();
                scalar_t* _err = _error.ptr<scalar_t>();

                for (size_t ptIdx = 0; ptIdx < num_points; ptIdx++) {
                    if (_err[ptIdx] < this->getErrorStandardDeviation(_z[ptIdx])) {
                        stats.inliers++;
                    } else {
                        stats.outliers++;
                    }
                    stats.error += std::abs(_err[ptIdx]);
                    if (stats.max_error < stats.error) {
                        stats.max_error = stats.error;
                    }
                }

                stats.error /= num_points;
                //            stats.noise /= num_points;
                plane3.x = planeCoeffs[0];
                plane3.y = planeCoeffs[1];
                plane3.z = -1;
                plane3.d = planeCoeffs[2];
                scalar_t normScale = 1.0 / sqrt(plane3.x * plane3.x + plane3.y * plane3.y + plane3.z * plane3.z);
                plane3.scale(normScale);
                //std::cout << "planeCoeffs = " << plane3 << " error = " << stats.error << std::endl;
                //std::cout.flush();

                return std::make_pair(plane3, stats);

            }
            
        public:
            std::pair<cv::Plane3f, cv::FitStatistics> fitPlaneExplicitLeastSquaresWithStats_newImpl(
                    const cv::Mat blockDepth, const int blockRange[4], 
                    const float c_x, const float c_y, const float f_x, const float f_y, const int numValid) const {

                cv::Plane3f plane3;
                cv::FitStatistics stats;
                
                int n_r1 = blockRange[0];
                int n_r2 = blockRange[1];
                int n_c1 = blockRange[2];
                int n_c2 = blockRange[3];
                
                cv::Mat _MtM =  cv::Mat::zeros(3, 3, CV_32F);
                _MtM.at<float>(0,0)  = -(((n_c1*(6*c_x*c_x - 6*c_x*n_c1 + 6*c_x + 2*n_c1*n_c1 - 3*n_c1 + 1))/6 - ((n_c2 + 1)*(6*c_x*c_x - 6*c_x*n_c2 + 2*n_c2*n_c2 + n_c2))/6)*(n_r2 - n_r1 + 1))/(f_x*f_x);
                _MtM.at<float>(0,1) = ((c_x*(n_c2 - n_c1 + 1) + (n_c1*(n_c1 - 1))/2 - (n_c2*(n_c2 + 1))/2)*(c_y*(n_r2 - n_r1 + 1) + (n_r1*(n_r1 - 1))/2 - (n_r2*(n_r2 + 1))/2))/(f_x*f_y);
                _MtM.at<float>(0,2) = -((n_r2 - n_r1 + 1)*(c_x*(n_c2 - n_c1 + 1) + (n_c1*(n_c1 - 1))/2 - (n_c2*(n_c2 + 1))/2))/f_x;
                _MtM.at<float>(1,0) = ((c_x*(n_c2 - n_c1 + 1) + (n_c1*(n_c1 - 1))/2 - (n_c2*(n_c2 + 1))/2)*(c_y*(n_r2 - n_r1 + 1) + (n_r1*(n_r1 - 1))/2 - (n_r2*(n_r2 + 1))/2))/(f_x*f_y);
                _MtM.at<float>(1,1) = -(((n_r1*(6*c_y*c_y - 6*c_y*n_r1 + 6*c_y + 2*n_r1*n_r1- 3*n_r1 + 1))/6 - ((n_r2 + 1)*(6*c_y*c_y - 6*c_y*n_r2 + 2*n_r2*n_r2 + n_r2))/6)*(n_c2 - n_c1 + 1))/(f_y*f_y);
                _MtM.at<float>(1,2) = -((n_c2 - n_c1 + 1)*(c_y*(n_r2 - n_r1 + 1) + (n_r1*(n_r1 - 1))/2 - (n_r2*(n_r2 + 1))/2))/f_y;
                _MtM.at<float>(2,0) = -((n_r2 - n_r1 + 1)*(c_x*(n_c2 - n_c1 + 1) + (n_c1*(n_c1 - 1))/2 - (n_c2*(n_c2 + 1))/2))/f_x;
                _MtM.at<float>(2,1) = -((n_c2 - n_c1 + 1)*(c_y*(n_r2 - n_r1 + 1) + (n_r1*(n_r1 - 1))/2 - (n_r2*(n_r2 + 1))/2))/f_y;
                _MtM.at<float>(2,2) = (n_c2 - n_c1 + 1)*(n_r2 - n_r1 + 1);
                //std::cout << "MtM = " << _MtM << std::endl;
                cv::Mat iMtM = _MtM.inv();
                
                float i_f_x = 1.0f/f_x;
                float i_f_y = 1.0f/f_y;
                float depth, i_depth, *mtb_ptr, *sol_ptr;
                const float *_blockDepth;
                
                _blockDepth = blockDepth.ptr<float>(0);
                cv::Mat Mtb = cv::Mat::zeros(3, 1, CV_32F);
                cv::Mat sol(3, 1, CV_32F);
                mtb_ptr = Mtb.ptr<float>(0);
                sol_ptr = sol.ptr<float>(0);
                
                size_t total_points = (n_c2 - n_c1 + 1)*(n_r2 - n_r1 + 1);
                size_t valid_points = total_points;
                cv::Mat _M = cv::Mat::zeros(total_points, 3, CV_32F);
                cv::Mat _Z = cv::Mat::zeros(total_points, 1, CV_32F);
                float* M = _M.ptr<float>(0);
                float* Z = _Z.ptr<float>(0); 
                size_t ptIdx = 0;
                for (int row = n_r1; row < n_r2 + 1; row++) {
                    for (int col = n_c1; col < n_c2 + 1; col++) {
                        depth = _blockDepth[ptIdx];
                        if (std::isnan(depth)) {  // nan values in blockDepth have already been replaced by 0
                            i_depth = 0;
                            valid_points--;
                        } else {
                            i_depth = 1.0f / depth;
                            mtb_ptr[0] = mtb_ptr[0] + (col-c_x) * i_f_x * i_depth;
                            mtb_ptr[1] = mtb_ptr[1] + (row-c_y) * i_f_y * i_depth;
                            mtb_ptr[2] = mtb_ptr[2] + i_depth;  

                            M[3 * ptIdx] = (col - c_x) * i_f_x;
                            M[3 * ptIdx + 1] = (row - c_y) * i_f_y;
                            M[3 * ptIdx + 2] = 1.0;
                            Z[ptIdx] = depth;
                            //std::cout << "depth = " << depth << std::endl;
                            stats.noise += this->getErrorStandardDeviation(i_depth);
                        }
                        ptIdx++;
                    }
                }
                //std::cout << "mtb = " << Mtb << std::endl;
                sol_ptr[0] = iMtM.at<float>(0,0) * mtb_ptr[0] + iMtM.at<float>(0,1) * mtb_ptr[1] + iMtM.at<float>(0,2) * mtb_ptr[2];
                sol_ptr[1] = iMtM.at<float>(1,0) * mtb_ptr[0] + iMtM.at<float>(1,1) * mtb_ptr[1] + iMtM.at<float>(1,2) * mtb_ptr[2];
                sol_ptr[2] = iMtM.at<float>(2,0) * mtb_ptr[0] + iMtM.at<float>(2,1) * mtb_ptr[1] + iMtM.at<float>(2,2) * mtb_ptr[2];
                                
                cv::Mat planeCoeffs = cv::Mat::zeros(4, 1, CV_32F);
                float *planeCoeffs_ptr = planeCoeffs.ptr<float>(0);
                float normf = 0;
                for (int dim = 0; dim < 3; ++dim) {
                    planeCoeffs_ptr[dim] = sol_ptr[dim];
                    normf += planeCoeffs_ptr[dim] * planeCoeffs_ptr[dim];
                }
                normf = sqrt(normf);                                

                float normalsData[3] = {planeCoeffs_ptr[0], planeCoeffs_ptr[1], planeCoeffs_ptr[2]};
                cv::Mat normals = cv::Mat(3, 1, CV_32F, normalsData);
                //std::cout << "normals = " << normals << std::endl;
                cv::Mat _error = 1 / (_M * normals) - _Z;
                float* _z = _Z.ptr<float>();
                float* _err = _error.ptr<float>();

                for (size_t ptIdx = 0; ptIdx < total_points; ptIdx++) {
                    if (_err[ptIdx] < this->getErrorStandardDeviation(_z[ptIdx])) {
                        stats.inliers++;
                    } else {
                        stats.outliers++;
                    }
                    stats.error += std::abs(_err[ptIdx]);
                    if (stats.max_error < stats.error) {
                        stats.max_error = stats.error;
                    }
                }

                stats.error /= valid_points;
                
                planeCoeffs /= normf;
                planeCoeffs_ptr[3] = 1.0 / normf;
                plane3.x = planeCoeffs_ptr[0];
                plane3.y = planeCoeffs_ptr[1];
                plane3.z = planeCoeffs_ptr[2];
                plane3.d = planeCoeffs_ptr[3];
                //std::cout << "planeCoeffs = " << plane3 << " error = " << stats.error << std::endl;
                return std::make_pair(plane3, stats);

            }
            
        public:
            static DepthIntegralImages iImgs;
        }; /* class RgbdImage */
    } /* namespace rgbd */
} /* namespace cv */

#endif /* __cplusplus */

#endif /* RGBD_IMAGE_UNCC_HPP */

