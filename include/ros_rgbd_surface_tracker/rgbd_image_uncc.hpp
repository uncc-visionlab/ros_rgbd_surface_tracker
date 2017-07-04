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
#include <opencv2/core.hpp>
#include <opencv2/rgbd.hpp>
#include <boost/shared_ptr.hpp>
#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>

namespace cv {
    namespace rgbd {

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
                //                for (int r=0; r < height; r++) {
                //                    for (int c=0; c < width; c++) {
                //                            //const float& depth = zptr[r * img_Z.cols + c];
                //                            std::cout << "depth = " <<_img_Z.at<float>(r,c) << std::endl;                        
                //                    }
                //                }

                //lstep = (int) (_img_L.step / _img_L.elemSize1());
                cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);
                cameraMatrix.at<float>(0, 0) = _f;
                cameraMatrix.at<float>(1, 1) = _f;
                cameraMatrix.at<float>(2, 2) = 1.0f;
                cameraMatrix.at<float>(0, 2) = _cx;
                cameraMatrix.at<float>(1, 2) = _cy;
            }

            virtual ~RgbdImage() {
            };

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
                int areaPerSample = tileArea / numSamples;
                int skip = areaPerSample / 4;
                Point3f pt3;
                for (int offset = 0; offset < skip && data.size() < numSamples; offset++) {
                    for (int iy = y0 + offset; iy < y0 + height && data.size() < numSamples; iy += skip) {
                        for (int ix = x0 + offset; ix < x0 + width && data.size() < numSamples; ix += skip) {
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
                }
                return (data.size() == numSamples) ? true : false;
            }

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

            LineSegment2f getVisibleLineSegment2f(Line3f& line3) const {
                LineSegment2f lineseg2;
                float alpha;
                double lambda;
                Point3f ip3;
                //            if (abs(line3.v.x) > abs(line3.v.y)) {
                lambda = -line3.p0.x / line3.v.x;
                line3.setPoint(ip3, lambda); // intersect w/ x=0 plane
                //            } else {
                //                lambda = -line3.p0.y / line3.v.y;
                //                line3.setPoint(ip3, lambda); // intersect w/ y=0 plane
                //            }
                lineseg2.p0 = project(ip3); // project pt at intersection into 2D image
                lineseg2.v.x = line3.v.x;
                lineseg2.v.y = line3.v.y;

                Point2f ip;
                lineseg2.start = -lineseg2.p0.x / lineseg2.v.x; // guess line passes through left side of image
                lineseg2.setPoint(ip, lineseg2.start);
                if (ip.y < 0 || ip.y > height) { // line does not passes through left side of image
                    lineseg2.start = -lineseg2.p0.y / lineseg2.v.y; // guess line passes through top of image
                    lineseg2.setPoint(ip, lineseg2.start);
                    if (ip.x < 0 || ip.x > width) { // line does not pass through top side of image
                        lineseg2.start = (height - 1 - lineseg2.p0.y) / lineseg2.v.y; // get right intersection point
                        lineseg2.end = (width - 1 - lineseg2.p0.x) / lineseg2.v.x; // get bottom intersection point
                        return lineseg2; // line through bottom and right edges
                    }
                    lineseg2.end = (height - 1 - lineseg2.p0.y) / lineseg2.v.y; // guess line passes through bottom side of image
                    lineseg2.setPoint(ip, lineseg2.end);
                    if (ip.x < 0 || ip.x > width) { // line does not pass through bottom side of image
                        lineseg2.end = (width - 1 - lineseg2.p0.x) / lineseg2.v.x;
                        return lineseg2; // line through top and right edges
                    } else {
                        // line through top and bottom edges (already computed))
                    }
                } else { // guess line passes through right side of image
                    lineseg2.end = (width - 1 - lineseg2.p0.x) / lineseg2.v.x;
                    lineseg2.setPoint(ip, lineseg2.end);
                    if (ip.y < 0 || ip.y > height) { // wrong
                        lineseg2.end = -lineseg2.p0.y / lineseg2.v.y; // guess top edge
                        lineseg2.setPoint(ip, lineseg2.end);
                        if (ip.x < 0 || ip.x > width) { // wrong
                            lineseg2.end = (height - 1 - lineseg2.p0.y) / lineseg2.v.y; // has to be bottom edge
                        }
                    }
                }
                std::cout << "Visible 2D line segment = " << lineseg2 << " lineseg2.start " << lineseg2.start << std::endl;
                //std::cout << "Visible 3D line segment = " << line3 
                //        << " start " << p0 << " start 2d " << project(p0)
                //        << " end " << p1  << " end 2d " << project(p1)
                //        << std::endl;
                return lineseg2;
            }

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
                    float& noise, int& inliers, int& outliers, int& invalid) const {
                int numPoints = data3.size();
                noise = 0;
                Mat _M = Mat::zeros(numPoints, 3, DataType<float>::type);
                float* M = _M.ptr<float>();
                Point3f centroid(0, 0, 0);
                for (int ptIdx = 0; ptIdx < numPoints; ++ptIdx) {
                    centroid.x += data3[ptIdx].x;
                    centroid.y += data3[ptIdx].y;
                    centroid.z += data3[ptIdx].z;
                }
                centroid.x /= numPoints;
                centroid.y /= numPoints;
                centroid.z /= numPoints;
                for (int ptIdx = 0; ptIdx < numPoints; ++ptIdx) {
                    M[3 * ptIdx] = data3[ptIdx].x - centroid.x;
                    M[3 * ptIdx + 1] = data3[ptIdx].y - centroid.y;
                    M[3 * ptIdx + 2] = data3[ptIdx].z - centroid.z;
                    noise += getErrorStandardDeviation(data3[ptIdx].z);
                }
                Mat _MtM = _M.t() * _M;
                cv::Mat eigVals, eigVecs;
                cv::eigen(_MtM, eigVals, eigVecs);
                cv::Mat _planeCoeffs = eigVecs.row(2).t();
                //std::cout << "E = " << E << std::endl;
                //std::cout << "MtM = " << _MtM << std::endl;          
                //std::cout << "V = " << eigVecs << std::endl;          
                //std::cout << "coeffs = " << _planeCoeffs << std::endl; 
                plane3.x = _planeCoeffs.at<float>(0);
                plane3.y = _planeCoeffs.at<float>(1);
                plane3.z = _planeCoeffs.at<float>(2);
                float d3 = -(plane3.x * centroid.x + plane3.y * centroid.y + plane3.z * centroid.z);
                plane3.d = d3;
                //std::cout << "centroid_dist = " << plane3.evaluate(centroid) << std::endl;
                Mat _D = Mat::ones(numPoints, 1, DataType<float>::type);
                _D *= plane3.d;
                for (int ptIdx = 0; ptIdx < numPoints; ++ptIdx) {
                    M[3 * ptIdx] += centroid.x;
                    M[3 * ptIdx + 1] += centroid.y;
                    M[3 * ptIdx + 2] += centroid.z;
                }
                Mat _error = _M * _planeCoeffs + _D;
                //std::cout << "plane.d =" << plane3.d << " D=" << _D << std::endl;
                //std::cout << "I_error=" << _error << std::endl;
                //std::cout << "M = " << M << std::endl;  [0.588991, 0.423888, -0.688047, 1.82959]
                //std::cout << "Z = " << Z << std::endl;
                //float* _z = _Z.ptr<float>();
                float* _err = _error.ptr<float>();
                error = inliers = outliers = invalid = 0;
                for (int ptIdx = 0; ptIdx < numPoints; ptIdx++) {
                    if (isInlier(_err[ptIdx], data3[ptIdx].z)) {
                        inliers++;
                    } else {
                        outliers++;
                    }
                    //                error += abs(_error.at<float>(ptIdx));
                    error += abs(_err[ptIdx]);
                }
                //            error /= numPoints;
                //            noise /= numPoints;
                plane3.scale((plane3.z > 0) ? -1.f : 1.f);
                //std::cout << "iplaneCoeffs = " << plane3 << " error = " << error << std::endl;
                //std::cout.flush();
            }

            const cv::Mat getDepth() {
                return img_Z;
            }

            const cv::Mat getCameraMatrix() {
                return cameraMatrix;
            }

            void setNormalsComputer(cv::Ptr<RgbdNormals> _nc) {
                normalsComputer = _nc;
            }

            cv::Ptr<RgbdNormals> getNormalsComputer(cv::Ptr<RgbdNormals> _nc) {
                return normalsComputer;
            }

            bool computeNormals() {
                cv::Mat normals(img_Z.size(), CV_32FC3);
                cv::Mat points(img_Z.size(), CV_32FC3);
                for (int r = 0; r < img_Z.rows; ++r) {
                    for (int c = 0; c < img_Z.cols; ++c) {
                        const float& depth = zptr[r * zstep + c];
                        points.at<cv::Point3f>(r, c).x = (c - cx) * inv_f*depth;
                        points.at<cv::Point3f>(r, c).y = (r - cy) * inv_f*depth;
                        points.at<cv::Point3f>(r, c).z = depth;
                    }
                }
                (*normalsComputer)(points, normals);
                std::cout << "normals computed " << std::endl;
                return true;
            }

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
            RgbdImage(const RgbdImage& ref);
            RgbdImage& operator=(const RgbdImage& ref);
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
        };
    } /* namespace rgbd */
} /* namespace cv */

#endif /* __cplusplus */

#endif /* RGBD_IMAGE_UNCC_HPP */

