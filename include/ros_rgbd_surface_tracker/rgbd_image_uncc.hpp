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
            typedef boost::shared_ptr<DepthIntegralImages> Ptr;

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
                //int type = _src.type(), depth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);
                //cv::Size ssize = _src.size(), isize(ssize.width + 1, ssize.height + 1);
                //_sum.create(isize, CV_MAKETYPE(sdepth, cn));
                //Mat src = _src.getMat(), sum = _sum.getMat();
                T* integral_image = (T *) integral_im.data;
                int rows = input.rows;
                int cols = input.cols;
                const float* input_ptr = (const float *) input.data;
                T* integral_image_ptr = integral_image;
                T* integral_image_lu_ptr = integral_image;
                for (int x = 0; x < cols + 1; ++x) {
                    *integral_image_ptr++ = 0;
                }
                int coffset = 0, poffset = 1;
                //T test;
                T left_value = 0, top_left_value = 0, top_value = 0;
                T* integral_image_l_ptr;
                for (int y = 0; y < static_cast<int> (rows); ++y) {
                    integral_image_l_ptr = integral_image_ptr;
                    *integral_image_ptr++ = 0;
                    for (int x = 0; x < static_cast<int> (cols); ++x) {
                        left_value = *(integral_image_l_ptr++);
                        top_left_value = *(integral_image_lu_ptr++);
                        top_value = *(integral_image_lu_ptr);
                        T& integral_pixel = *(integral_image_ptr++);
                        integral_pixel = static_cast<T> (*input_ptr++);
                        //std::cout << "current pixel i(" << x << ", " << y << ") = " << integral_pixel;
                        //if (std::isnan(integral_pixel)) {
                        //std::cout << "Found Nan!" << std::endl;
                        //     integral_pixel = static_cast<T> (0.0f);
                        // }
                        integral_pixel += left_value + top_value - top_left_value;
                        //std::cout << "integral pixel s(x,y) = " << integral_pixel
                        //        << " s(x-1,y) = " << left_value << " s(x,y-1) = " << top_value
                        //        << " s(x-1,y-1) = " << top_left_value << std::endl;
                    }
                    integral_image_lu_ptr++;
                }
            }

            template<typename T>
            T getSum(cv::Mat iImg, cv::Rect2i& rect) const {
                //                int offset_tlc = rect.y * width + rect.x;
                //                int offset_trc = offset_tlc + rect.width+1;
                //                int offset_blc = offset_tlc + (rect.height+1) * width;
                //                int offset_brc = offset_blc + rect.width+1;
                //                return iImg[offset_brc] + iImg[offset_tlc] - iImg[offset_trc] - iImg[offset_blc];
                //return iImg.at<T>(rect.y, rect.x) + iImg.at<T>(rect.y + rect.height + 1, rect.x + rect.width + 1) -
                //        iImg.at<T>(rect.y + rect.height + 1, rect.x) - iImg.at<T>(rect.y, rect.x + rect.width + 1);
                return iImg.at<T>(rect.y, rect.x) + iImg.at<T>(rect.y + rect.height, rect.x + rect.width) -
                        iImg.at<T>(rect.y + rect.height, rect.x) - iImg.at<T>(rect.y, rect.x + rect.width);
            }


#define GETSUM(src, type, A, B, C, D) ((type *)src.data)[D] + ((type *)src.data)[A] - \
                ((type *)src.data)[B] - ((type *)src.data)[C]                

            void computeMat_LUT() {
                cv::Mat MtM(3, 3, CV_32F);
                float *mtm_ptr;
                int numPts = winSize.area();

                // sliding window on integral image 
                // the integral images have one more row and column than the source image
                ImageWindow imWin(one_div_z.size(), winSize);
                cv::Point2i *ibegin = imWin.begin(), *iend = imWin.end();
                for (int y = ibegin->y; y < iend->y - 1; ++y) {
                    imWin.centerOn(ibegin->x, y);
                    int key = y * width + ibegin->x;
                    for (int x = ibegin->x; x < iend->x - 1; ++x, ++key) {
                        mtm_ptr = MtM.ptr<float>(0, 0);
                        *mtm_ptr++ = GETSUM(tan_theta_x_sq, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                        *mtm_ptr++ = GETSUM(tan_theta_xy, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                        *mtm_ptr++ = GETSUM(tan_theta_x, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                        *mtm_ptr++ = MtM.at<float>(0, 1);
                        *mtm_ptr++ = GETSUM(tan_theta_y_sq, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                        *mtm_ptr++ = GETSUM(tan_theta_y, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                        *mtm_ptr++ = MtM.at<float>(0, 2);
                        *mtm_ptr++ = MtM.at<float>(1, 2);
                        *mtm_ptr++ = numPts;
                        cv::Mat iMtM = MtM.inv();
                        matMap.insert({key, iMtM});
                        imWin.shiftRight();
                    }
                }

                dz_dx = cv::Mat::zeros(height, width, CV_32F);
                dz_dy = cv::Mat::zeros(height, width, CV_32F);
                d2z_dx2 = cv::Mat::zeros(height, width, CV_32F);
                d2z_dy2 = cv::Mat::zeros(height, width, CV_32F);
                d2z_dxdy = cv::Mat::zeros(height, width, CV_32F);
            }

            void initialize(int _width, int _height, float _cx, float _cy, float _inv_f,
                    cv::Size _winSize) {
                width = _width;
                height = _height;
                setWindowSize(_winSize);
                cv::Mat _tan_theta_x(height, width, CV_32F);
                cv::Mat _tan_theta_y(height, width, CV_32F);
                ntan_theta_x = cv::Mat::zeros(height, width, CV_32F);
                ntan_theta_y = cv::Mat::zeros(height, width, CV_32F);
                cv::Mat _tan_theta_xy(height, width, CV_32F);
                cv::Mat _tan_theta_x_sq(height, width, CV_32F);
                cv::Mat _tan_theta_y_sq(height, width, CV_32F);
                _tan_theta_x_div_z = cv::Mat::zeros(height, width, CV_32F);
                _tan_theta_y_div_z = cv::Mat::zeros(height, width, CV_32F);
                _one_div_z = cv::Mat::zeros(height, width, CV_32F);
                _one_div_z_sq = cv::Mat::zeros(height, width, CV_32F);
                _ones_valid_mask = cv::Mat::zeros(height, width, CV_32F);

                numValidPts = cv::Mat::zeros(height + 1, width + 1, CV_32F);
                tan_theta_x_div_z = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                tan_theta_y_div_z = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                one_div_z = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                one_div_z_sq = cv::Mat::zeros(height + 1, width + 1, CV_64F);

                double ttxval, ttyval;
                for (int r = 0; r < height; ++r) {
                    for (int c = 0; c < width; ++c) {
                        ttxval = (c - _cx) * _inv_f;
                        ttyval = (r - _cy) * _inv_f;
                        _tan_theta_x.at<float>(r, c) = ttxval;
                        _tan_theta_y.at<float>(r, c) = ttyval;
                        ntan_theta_x.at<float>(r, c) = ttxval;
                        ntan_theta_y.at<float>(r, c) = ttyval;
                        _tan_theta_xy.at<float>(r, c) = ttxval*ttyval;
                        //_tan_theta_x_sq.at<float>(r, c) = ttxval*ttxval;
                        //_tan_theta_y_sq.at<float>(r, c) = ttyval*ttyval;
                    }
                }
                //tan_theta_x = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                //tan_theta_y = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                //tan_theta_x_sq = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                //tan_theta_y_sq = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                //tan_theta_xy = cv::Mat::zeros(height + 1, width + 1, CV_64F);
                //computeIntegralImage<double>(_tan_theta_x, tan_theta_x);
                //computeIntegralImage<double>(_tan_theta_y, tan_theta_y);
                //computeIntegralImage<double>(_tan_theta_xy, tan_theta_xy);
                //computeIntegralImage<double>(_tan_theta_x_sq, tan_theta_x_sq);
                //computeIntegralImage<double>(_tan_theta_y_sq, tan_theta_y_sq);
                cv::integral(_tan_theta_x, tan_theta_x, tan_theta_x_sq, CV_64F, CV_64F);
                cv::integral(_tan_theta_y, tan_theta_y, tan_theta_y_sq, CV_64F, CV_64F);
                cv::integral(_tan_theta_xy, tan_theta_xy, CV_64F);
#ifdef CACHE_INVERSE
                computeMat_LUT();
#endif
                std::cout << "Computed integral images of (tax_x, tan_y) for "
                        << width << "x" << height << " depth images." << std::endl;
            }

            void computeCurvatureFiniteDiff_Impl(const cv::Mat& depth, cv::Mat& normals); 
            
            void plucker(const cv::Mat& depth, cv::Mat& mean_curvature, cv::Mat& axes);

            void operator()(cv::InputArray depth_in, cv::OutputArray normals_out) {
                Mat depth_ori = depth_in.getMat();

                // Get the normals
                normals_out.create(depth_ori.size(), CV_32FC3);
                if (depth_in.empty())
                    return;

                Mat normals = normals_out.getMat();
                computeExplicit_Impl(depth_ori, normals);
            }

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

            void computeImplicit_Impl(const cv::Mat& depth, cv::Mat & normals) {
                const float *depth_ptr = depth.ptr<float>(0, 0);
                float *_ones_valid_mask_ptr = _ones_valid_mask.ptr<float>(0, 0);
                float *_tan_theta_x_div_z_ptr = _tan_theta_x_div_z.ptr<float>(0, 0);
                float *_tan_theta_y_div_z_ptr = _tan_theta_y_div_z.ptr<float>(0, 0);
                float *_one_div_z_ptr = _one_div_z.ptr<float>(0, 0);
                float *_one_div_z_sq_ptr = _one_div_z_sq.ptr<float>(0, 0);
                float *ntan_theta_x_ptr = ntan_theta_x.ptr<float>(0, 0);
                float *ntan_theta_y_ptr = ntan_theta_y.ptr<float>(0, 0);
                for (int pixNum = 0; pixNum < depth.rows * depth.cols; ++pixNum) {
                    if (std::isnan(depth_ptr[pixNum])) {
                        _one_div_z_ptr[pixNum] = 0;
                        _ones_valid_mask_ptr[pixNum] = 0;

                    } else {
                        _one_div_z_ptr[pixNum] = 1.0f / depth_ptr[pixNum];
                        _ones_valid_mask_ptr[pixNum] = 1;
                    }
                    _tan_theta_x_div_z_ptr[pixNum] = ntan_theta_x_ptr[pixNum] * _one_div_z_ptr[pixNum];
                    _tan_theta_y_div_z_ptr[pixNum] = ntan_theta_y_ptr[pixNum] * _one_div_z_ptr[pixNum];
                    _one_div_z_sq_ptr[pixNum] = _one_div_z_ptr[pixNum] * _one_div_z_ptr[pixNum];
                }
                //computeIntegralImage<double>(_tan_theta_x_div_z, tan_theta_x_div_z);
                //computeIntegralImage<double>(_tan_theta_y_div_z, tan_theta_y_div_z);
                //computeIntegralImage<double>(_one_div_z, one_div_z);
                //computeIntegralImage<double>(_one_div_z_sq, one_div_z_sq);
                //computeIntegralImage<float>(_ones_valid_mask, numValidPts);
                cv::integral(_tan_theta_x_div_z, tan_theta_x_div_z, CV_64F);
                cv::integral(_tan_theta_y_div_z, tan_theta_y_div_z, CV_64F);
                cv::integral(_one_div_z, one_div_z, one_div_z_sq, CV_64F, CV_64F);
                cv::integral(_ones_valid_mask, numValidPts, CV_32F);

                int numPts = winSize.width * winSize.height;
                int numValidWinPts = 0;
                cv::Mat sVals(4, 4, CV_64F);
                double *s_ptr, *coeffs_ptr;
                cv::Mat eVecs = cv::Mat::zeros(4, 4, CV_64F);
                cv::Mat eVals = cv::Mat::zeros(4, 1, CV_64F);
                //cv::Mat coeffs = cv::Mat(eVecs, Rect(0, 3, eVecs.cols, 1));
                cv::Vec3f *normptr;
                int numcomp = 0;

                // sliding window on integral image 
                // the integral images have one more row and column than the source image
                ImageWindow imWin(one_div_z.size(), winSize);
                cv::Point2i *ibegin = imWin.begin(), *iend = imWin.end();
                for (int y = ibegin->y; y < iend->y - 1; ++y) {
                    imWin.centerOn(ibegin->x, y);
                    normptr = normals.ptr<cv::Vec3f>(y, ibegin->x);
                    int key = y * width + ibegin->x;
                    for (int x = ibegin->x; x < iend->x - 1; ++x, ++key) {
                        if (!std::isnan(depth_ptr[key])) {
                            numValidWinPts = GETSUM(numValidPts, float, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                            if (numValidWinPts == numPts) {
                                s_ptr = (double *) sVals.data;
                                *s_ptr++ = GETSUM(tan_theta_x_sq, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = GETSUM(tan_theta_xy, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = GETSUM(tan_theta_x, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = GETSUM(tan_theta_x_div_z, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = sVals.at<double>(0, 1);
                                *s_ptr++ = GETSUM(tan_theta_y_sq, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = GETSUM(tan_theta_y, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = GETSUM(tan_theta_y_div_z, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = sVals.at<double>(0, 2);
                                *s_ptr++ = sVals.at<double>(1, 2);
                                *s_ptr++ = numPts;
                                *s_ptr++ = GETSUM(one_div_z, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                *s_ptr++ = sVals.at<double>(0, 3);
                                *s_ptr++ = sVals.at<double>(1, 3);
                                *s_ptr++ = sVals.at<double>(2, 3);
                                *s_ptr++ = GETSUM(one_div_z_sq, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                //std::cout << "svals = " << sVals << std::endl;
                                cv::eigen(sVals, eVals, eVecs);
                                //std::cout << "evecs = " << eVecs << std::endl;
                                //std::cout << "evals = " << eVals << std::endl;
                                cv::Mat coeffs = cv::Mat(eVecs, Rect(0, 3, eVecs.cols, 1));
                                coeffs_ptr = coeffs.ptr<double>(0);
                                if (coeffs_ptr[2] > 0) {
                                    for (int dim = 0; dim < 4; ++dim) {
                                        coeffs_ptr[dim] = -coeffs_ptr[dim];
                                    }
                                }
                                float normf = 0;
                                for (int dim = 0; dim < 3; ++dim) {
                                    normf += (coeffs_ptr[dim])*(coeffs_ptr[dim]);
                                }
                                normf = sqrt(normf);
                                coeffs /= normf;
                                //std::cout << "normf = " << normf << std::endl;
                                //std::cout << "coeffs = " << coeffs << std::endl;
                                float error = sqrt(eVals.at<double>(3, 0)) / (normf * numPts);
                                cv::Vec3f& normVal = *normptr;
                                normVal[0] = coeffs_ptr[0];
                                normVal[1] = coeffs_ptr[1];
                                normVal[2] = coeffs_ptr[2];
                                //std::cout << "normval = " << normals.at<cv::Vec3f>(winCenter.y, winCenter.x) << std::endl;
                                numcomp++;
                            } // numValidPts = windowArea 
                        } // depth at wincenter != nan
                        imWin.shiftRight();
                        normptr++;
                    }
                }
            }

            void computeExplicit_Impl(const cv::Mat& depth, cv::Mat & normals) {
                const float *depth_ptr = depth.ptr<float>(0, 0);
                float *_one_div_z_ptr = _one_div_z.ptr<float>(0, 0);
                float *_ones_valid_mask_ptr = _ones_valid_mask.ptr<float>(0, 0);
                float *_tan_theta_x_div_z_ptr = _tan_theta_x_div_z.ptr<float>(0, 0);
                float *_tan_theta_y_div_z_ptr = _tan_theta_y_div_z.ptr<float>(0, 0);
                float *ntan_theta_x_ptr = ntan_theta_x.ptr<float>(0, 0);
                float *ntan_theta_y_ptr = ntan_theta_y.ptr<float>(0, 0);
                for (int pixNum = 0; pixNum < depth.rows * depth.cols; ++pixNum) {
                    if (std::isnan(depth_ptr[pixNum])) {
                        _one_div_z_ptr[pixNum] = 0;
                        _ones_valid_mask_ptr[pixNum] = 0;
                    } else {
                        _one_div_z_ptr[pixNum] = 1.0f / depth_ptr[pixNum];
                        _ones_valid_mask_ptr[pixNum] = 1;
                    }
                    _tan_theta_x_div_z_ptr[pixNum] = ntan_theta_x_ptr[pixNum] * _one_div_z_ptr[pixNum];
                    _tan_theta_y_div_z_ptr[pixNum] = ntan_theta_y_ptr[pixNum] * _one_div_z_ptr[pixNum];
                }
                cv::integral(_tan_theta_x_div_z, tan_theta_x_div_z, CV_64F);
                cv::integral(_tan_theta_y_div_z, tan_theta_y_div_z, CV_64F);
                cv::integral(_one_div_z, one_div_z, CV_64F);
                cv::integral(_ones_valid_mask, numValidPts, CV_32F);

                int numPts = winSize.width * winSize.height;
                int numValidWinPts = 0;
                cv::Mat MtM(3, 3, CV_32F);
                cv::Mat iMtM(3, 3, CV_32F);
                cv::Mat Mtb(3, 1, CV_32F);
                cv::Mat sol(3, 1, CV_32F);
                cv::Mat coeffs(1, 4, CV_32F);
                float *mtm_ptr, *mtb_ptr, *imtm_ptr, *sol_ptr, *coeffs_ptr;
                imtm_ptr = iMtM.ptr<float>(0, 0);
                mtb_ptr = Mtb.ptr<float>(0);
                sol_ptr = sol.ptr<float>(0);
                coeffs_ptr = coeffs.ptr<float>(0);
                cv::Vec3f *normptr;
                int numcomp = 0;

                // sliding window on integral image 
                // the integral images have one more row and column than the source image
                ImageWindow imWin(one_div_z.size(), winSize);
                cv::Point2i *ibegin = imWin.begin(), *iend = imWin.end();
                for (int y = ibegin->y; y < iend->y - 1; ++y) {
                    imWin.centerOn(ibegin->x, y);
                    normptr = normals.ptr<cv::Vec3f>(y, ibegin->x);
                    int key = y * width + ibegin->x;
                    for (int x = ibegin->x; x < iend->x - 1; ++x, ++key) {
                        if (!std::isnan(depth_ptr[key])) {
                            numValidWinPts = GETSUM(numValidPts, float, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                            if (numValidWinPts == numPts) {
#ifndef CACHE_INVERSE
                                mtm_ptr = MtM.ptr<float>(0, 0);
                                *mtm_ptr++ = GETSUM(tan_theta_x_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                                *mtm_ptr++ = GETSUM(tan_theta_xy, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                                *mtm_ptr++ = GETSUM(tan_theta_x, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                                *mtm_ptr++ = MtM.at<float>(0, 1);
                                *mtm_ptr++ = GETSUM(tan_theta_y_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                                *mtm_ptr++ = GETSUM(tan_theta_y, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                                *mtm_ptr++ = MtM.at<float>(0, 2);
                                *mtm_ptr++ = MtM.at<float>(1, 2);
                                *mtm_ptr++ = numPts;
                                iMtM = MtM.inv();
                                imtm_ptr = iMtM.ptr<float>(0, 0);
#else
                                imtm_ptr = matMap[key].ptr<float>(0, 0);
                                //if (iMtM.empty()) {
                                //    std::cout << "Key for matrix inverse not found!" << std::endl;
                                //}
#endif                            
                                //std::cout << "MtM = " << MtM << std::endl;
                                //std::cout << "iMtM" << winCenter << " = " << iMtM << std::endl;
                                //std::cout << "iMtM2 = " << iMtM2 << std::endl;
                                mtb_ptr[0] = GETSUM(tan_theta_x_div_z, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                mtb_ptr[1] = GETSUM(tan_theta_y_div_z, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);
                                mtb_ptr[2] = GETSUM(one_div_z, double, imWin.tlc, imWin.trc, imWin.blc, imWin.brc);

                                //cv::Mat sol = iMtM*Mtb;
                                sol_ptr[0] = imtm_ptr[0] * mtb_ptr[0] + imtm_ptr[1] * mtb_ptr[1] + imtm_ptr[2] * mtb_ptr[2];
                                sol_ptr[1] = imtm_ptr[3] * mtb_ptr[0] + imtm_ptr[4] * mtb_ptr[1] + imtm_ptr[5] * mtb_ptr[2];
                                sol_ptr[2] = imtm_ptr[6] * mtb_ptr[0] + imtm_ptr[7] * mtb_ptr[1] + imtm_ptr[8] * mtb_ptr[2];

                                float normf = 0;
                                for (int dim = 0; dim < 2; ++dim) {
                                    coeffs_ptr[dim] = sol_ptr[dim];
                                    normf += coeffs_ptr[dim] * coeffs_ptr[dim];
                                }
                                coeffs_ptr[3] = coeffs_ptr[2];
                                coeffs_ptr[2] = 1.0f;
                                for (int dim = 0; dim < 4; ++dim) {
                                    coeffs_ptr[dim] = -coeffs_ptr[dim];
                                }
                                normf += 1;
                                normf = sqrt(normf);
                                coeffs /= normf;
                                //std::cout << "normf = " << normf << std::endl;
                                //std::cout << "coeffs = " << coeffs << std::endl;
                                //float error = sqrt(eVals.at<double>(3, 0)) / (normf * numPts);
                                cv::Vec3f& normVal = *normptr;
                                normVal[0] = coeffs_ptr[0];
                                normVal[1] = coeffs_ptr[1];
                                normVal[2] = coeffs_ptr[2];
                                //std::cout << "normval = " << normals.at<cv::Vec3f>(winCenter.y, winCenter.x) << std::endl;
                                numcomp++;
                            } // numValidPts = windowArea 
                        } // depth at wincenter != nan
                        imWin.shiftRight();
                        normptr++;
                    }
                }
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
                int normalWinSize = iImgs.getWindowSize().width;
                int normalMethod = RgbdNormals::RGBD_NORMALS_METHOD_FALS; // 7.0 fps
                //int normalMethod = RgbdNormals::RGBD_NORMALS_METHOD_LINEMOD;
                //int normalMethod = RgbdNormals::RGBD_NORMALS_METHOD_SRI; // 1.14 fps
                //                cv::Ptr<RgbdNormals> normalsComputer;
                //                normalsComputer = makePtr<RgbdNormals>(getDepth().rows,
                //                        getDepth().cols,
                //                        getDepth().depth(),
                //                        getCameraMatrix(),
                //                        normalWinSize,
                //                        normalMethod);
                //                setNormalsComputer(normalsComputer);
                //
                //                cv::Mat normals(getDepth().size(), CV_32FC3);
                //                cv::Mat points(getDepth().size(), CV_32FC3);
                //                for (int r = 0; r < getDepth().rows; ++r) {
                //                    for (int c = 0; c < getDepth().cols; ++c) {
                //                        const float& depth = zptr[r * zstep + c];
                //                        points.at<cv::Point3f>(r, c).x = (c - cx) * inv_f*depth;
                //                        points.at<cv::Point3f>(r, c).y = (r - cy) * inv_f*depth;
                //                        points.at<cv::Point3f>(r, c).z = depth;
                //                    }
                //                }
                //                (*normalsComputer)(points, normals);

                //                                cv::Mat normals2(getDepth().size(), CV_32FC3);
                //                                iImgs.computeImplicit_Impl(getDepth(), normals2);
                cv::Mat normals3(getDepth().size(), CV_32FC3);
                iImgs.computeExplicit_Impl(getDepth(), normals3);
                iImgs.computeCurvatureFiniteDiff_Impl(getDepth(), normals3);
                //                                cv::Point2i tlc(315, 235);
                //                cv::Rect roi(tlc.x, tlc.y, width - 2 * tlc.x, height - 2 * tlc.y);
                //                cv::Point2i winCenter;
                //                cv::Vec3f *normptr, *normptr2, *normptr3;
                //                for (winCenter.y = roi.y; winCenter.y < roi.y + roi.height; ++winCenter.y) {
                //                    normptr = normals.ptr<cv::Vec3f>(winCenter.y, roi.x);
                //                    normptr2 = normals2.ptr<cv::Vec3f>(winCenter.y, roi.x);
                //                    normptr3 = normals3.ptr<cv::Vec3f>(winCenter.y, roi.x);
                //                    for (winCenter.x = roi.x; winCenter.x < roi.x + roi.width; ++winCenter.x) {
                //                        std::cout << " normals_fals(" << winCenter.x << ","
                //                                << winCenter.y << ") = " << *normptr << std::endl;
                //                        std::cout << " normals_impl(" << winCenter.x << ","
                //                                << winCenter.y << ") = " << *normptr2 << std::endl;
                //                        std::cout << " normals_expl(" << winCenter.x << ","
                //                                << winCenter.y << ") = " << *normptr3 << std::endl;
                //                        std::cout << " normals diff_error(" << (*normptr - *normptr2) << ")" << std::endl;
                //                        normptr++;
                //                        normptr2++;
                //                        normptr3++;
                //                    }
                //                }

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

