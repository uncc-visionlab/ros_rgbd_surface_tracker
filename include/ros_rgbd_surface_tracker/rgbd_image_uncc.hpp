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

        // forward declaration for C++ compiler
        //class DepthIntegralImages;

        class DepthIntegralImages {
        private:
            cv::Mat ntan_theta_x, ntan_theta_y;
            cv::Mat tan_theta_x, tan_theta_y;
            cv::Mat tan_theta_xy, tan_theta_x_sq, tan_theta_y_sq;
            int width, height;

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

        public:
            typedef boost::shared_ptr<DepthIntegralImages> Ptr;

            int getWidth() {
                return width;
            }

            int getHeight() {
                return height;
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

            void initialize(int _width, int _height, float _cx, float _cy, float _inv_f) {
                width = _width;
                height = _height;
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
                std::cout << "Computed integral images of (tax_x, tan_y) for "
                        << width << "x" << height << " depth images." << std::endl;
            }

            void operator()(cv::InputArray depth_in, cv::OutputArray normals_out) {
                Mat depth_ori = depth_in.getMat();

                // Get the normals
                normals_out.create(depth_ori.size(), CV_32FC3);
                if (depth_in.empty())
                    return;

                Mat normals = normals_out.getMat();
                compute(depth_ori, normals);
            }

            void compute(const cv::Mat& depth, cv::Mat& normals) {
                //                cv::Mat test(3, 20, CV_32F);
                //                for (int r = 0; r < test.rows; ++r) {
                //                    for (int c = 0; c < test.cols; ++c) {
                //                        test.at<float>(r, c) = r * 5 + c;
                //                    }
                //                }
                //                cv::Mat itest(test.rows + 1, test.cols + 1, CV_64F);
                //                computeIntegralImage<double>(test, itest);
                //                cv::Mat itest2;
                //                cv::integral(test, itest2, CV_64F);
                //                std::cout << "test = [" << test << "]" << std::endl;
                //                std::cout << "itest = [" << itest << "]" << std::endl;
                //                std::cout << "itest2 = [" << itest2 << "]" << std::endl;
                //                cv::Rect ttrect(0, 1, 1, 1);
                //                std::cout << "sum1 = " << getSum<double>(itest, ttrect) << std::endl;
                //                std::cout << "test = " << itest.at<double>(2, 2) << std::endl;
                //                std::cout << "test2 = " << ((double*) itest.data)[2 * itest.cols + 2] << std::endl;
                //cv::Mat _tan_theta_x_div_z(depth.size(), CV_32F);
                //cv::Mat _tan_theta_y_div_z(depth.size(), CV_32F);
                //cv::Mat _one_div_z(depth.size(), CV_32F);
                //cv::Mat _one_div_z_sq(depth.size(), CV_32F);
                //cv::Mat _ones_valid_mask(depth.size(), CV_32F);

                //cv::Mat numValidPts(height + 1, width + 1, CV_32F);
                //cv::Mat tan_theta_x_div_z(height + 1, width + 1, CV_64F);
                //cv::Mat tan_theta_y_div_z(height + 1, width + 1, CV_64F);
                //cv::Mat one_div_z(height + 1, width + 1, CV_64F);
                //cv::Mat one_div_z_sq(height + 1, width + 1, CV_64F);
                float *depth_ptr = (float *) depth.data;
                float *_one_div_z_ptr = (float *) _one_div_z.data;
                float *_ones_valid_mask_ptr = (float *) _ones_valid_mask.data;
                float *_tan_theta_x_div_z_ptr = (float *) _tan_theta_x_div_z.data;
                float *_tan_theta_y_div_z_ptr = (float *) _tan_theta_y_div_z.data;
                float *_one_div_z_sq_ptr = (float *) _one_div_z_sq.data;
                float *ntan_theta_x_ptr = (float *) ntan_theta_x.data;
                float *ntan_theta_y_ptr = (float *) ntan_theta_y.data;
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

                cv::Rect2i trect(0, 0, width, height);
                //int badpointcount = getSum<int>((int *) numValidPts.data, trect);
                int badpointcount = getSum<float>(numValidPts, trect);
                //std::cout << badpointcount << " valid points in the RGBD image." << std::endl;

                cv::Size winSize(5, 5); // width, height
                cv::Size halfWinSize(winSize.width >> 1, winSize.height >> 1);
                //cv::Size MARGIN(winSize.width + 2, winSize.height + 2);
                cv::Size MARGIN(halfWinSize.width + 2, halfWinSize.height + 2);
                cv::Rect rect(MARGIN.width - halfWinSize.width - 1,
                        MARGIN.height - halfWinSize.height - 1,
                        winSize.width, winSize.height);
                int numPts = winSize.width * winSize.height;
                int numValidWinPts = 0;
                cv::Mat sVals(4, 4, CV_64F);
                double *s_ptr, *coeffs_ptr;
                cv::Point2i winCenter(MARGIN.width, MARGIN.height);
                cv::Vec3f* normptr;
                cv::Mat eVecs = cv::Mat::zeros(4, 4, CV_64F);
                cv::Mat eVals = cv::Mat::zeros(4, 1, CV_64F);
                //cv::Mat coeffs = cv::Mat(eVecs, Rect(0, 3, eVecs.cols, 1));
                //cv::Mat eVecs, eVals;
                int offset_tlc, offset_trc, offset_blc, offset_brc;
                cv::Point2i tlc2d(MARGIN.width - halfWinSize.width - 1,
                        MARGIN.height - halfWinSize.height - 1);

#define GETSUM(src, type, A, B, C, D) ((type *)src.data)[D] + ((type *)src.data)[A] - \
                ((type *)src.data)[B] - ((type *)src.data)[C]
                
                for (rect.y = tlc2d.y; rect.y < depth.rows - MARGIN.height; ++rect.y, ++winCenter.y) {
                    winCenter.x = MARGIN.width;
                    offset_tlc = rect.y * (width + 1) + tlc2d.x;
                    offset_trc = offset_tlc + rect.width;
                    offset_blc = offset_tlc + rect.height * (width + 1);
                    offset_brc = offset_blc + rect.width;
                    normptr = ((cv::Vec3f*)normals.data);
                    normptr += rect.y * width + tlc2d.x;
                    for (rect.x = tlc2d.x; rect.x < depth.cols - MARGIN.width; ++rect.x, ++winCenter.x) {
                        //numValidWinPts = getSum<float>(numValidPts, rect);
                        //if (!std::isnan(depth_ptr[winCenter.y * width + winCenter.x])) {
                        numValidWinPts = GETSUM(numValidPts, float, offset_tlc, offset_trc, offset_blc, offset_brc);
                        //if (abs(numValidWinPts - numValidWinPts2) > 0) {
                        // std::cout << "num valid = " << numValidWinPts
                        //  << " num valid2 = " << numValidWinPts2 << std::endl;
                        //}
                        if (numValidWinPts == numPts) {
                            s_ptr = (double *) sVals.data;
                            *s_ptr++ = GETSUM(tan_theta_x_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_xy, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_x, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_x_div_z, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = sVals.at<double>(0, 1);
                            *s_ptr++ = GETSUM(tan_theta_y_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_y, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_y_div_z, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = sVals.at<double>(0, 2);
                            *s_ptr++ = sVals.at<double>(1, 2);
                            *s_ptr++ = numPts;
                            *s_ptr++ = GETSUM(one_div_z, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = sVals.at<double>(0, 3);
                            *s_ptr++ = sVals.at<double>(1, 3);
                            *s_ptr++ = sVals.at<double>(2, 3);
                            *s_ptr++ = GETSUM(one_div_z_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            //std::cout << "svals = " << sVals << std::endl;
                            //directly use the buffer allocated by OpenCV
                            //Eigen::Map<Eigen::Matrix4d> eigenT((double *) sVals.data);
                            //es.compute(eigenT);
                            //std::cout << "The eigenvalues of A are:" << std::endl << es.eigenvalues() << std::endl;
                            //std::cout << "The matrix of eigenvectors, V, is:" << std::endl << es.eigenvectors() << std::endl;
                            cv::eigen(sVals, eVals, eVecs);
                            //std::cout << "evecs = " << eVecs << std::endl;
                            //std::cout << "evals = " << eVals << std::endl;
                            cv::Mat coeffs = cv::Mat(eVecs, Rect(0, 3, eVecs.cols, 1));
                            coeffs_ptr = (double *) coeffs.data;
                            //std::cout << "coeffs = " << coeffs << std::endl;
                            float normf = 0;
                            for (int dim = 0; dim < 3; ++dim) {
                                normf += (*coeffs_ptr)*(*coeffs_ptr);
                                coeffs_ptr++;
                            }
                            normf = sqrt(normf);
                            coeffs /= normf;
                            //std::cout << "normf = " << normf << std::endl;
                            //std::cout << "coeffs = " << coeffs << std::endl;
                            float error = sqrt(eVals.at<double>(3, 0)) / (normf * numPts);
                            (*normptr)[0] = ((double *) coeffs.data)[0];
                            (*normptr)[1] = ((double *) coeffs.data)[1];
                            (*normptr)[2] = ((double *) coeffs.data)[2];
                            normptr++;
                            //std::cout << "normval = " << normals.at<cv::Vec3f>(winCenter.y, winCenter.x) << std::endl;
                        }
                        offset_tlc++;
                        offset_trc++;
                        offset_blc++;
                        offset_brc++;
                        //}
                    }
                }
            }

            void compute2(const cv::Mat& depth, cv::Mat& normals) {
                float *depth_ptr = (float *) depth.data;
                float *_one_div_z_ptr = (float *) _one_div_z.data;
                float *_ones_valid_mask_ptr = (float *) _ones_valid_mask.data;
                float *_tan_theta_x_div_z_ptr = (float *) _tan_theta_x_div_z.data;
                float *_tan_theta_y_div_z_ptr = (float *) _tan_theta_y_div_z.data;
                float *ntan_theta_x_ptr = (float *) ntan_theta_x.data;
                float *ntan_theta_y_ptr = (float *) ntan_theta_y.data;
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

                cv::Rect2i trect(0, 0, width, height);
                //int badpointcount = getSum<int>((int *) numValidPts.data, trect);
                int badpointcount = getSum<float>(numValidPts, trect);
                //std::cout << badpointcount << " valid points in the RGBD image." << std::endl;

                cv::Size winSize(11, 11); // width, height
                cv::Size halfWinSize(winSize.width >> 1, winSize.height >> 1);
                //cv::Size MARGIN(winSize.width + 2, winSize.height + 2);
                cv::Size MARGIN(200, 200);
                cv::Rect rect(MARGIN.width - halfWinSize.width - 1,
                        MARGIN.height - halfWinSize.height - 1,
                        winSize.width, winSize.height);
                int numPts = winSize.width * winSize.height;
                int numValidWinPts = 0;
                cv::Mat sVals(4, 4, CV_64F);
                double *s_ptr, *coeffs_ptr;
                cv::Point2i winCenter(MARGIN.width, MARGIN.height);
                cv::Vec3f* normptr;
                cv::Mat eVecs = cv::Mat::zeros(4, 4, CV_64F);
                cv::Mat eVals = cv::Mat::zeros(4, 1, CV_64F);
                //cv::Mat coeffs = cv::Mat(eVecs, Rect(0, 3, eVecs.cols, 1));
                //cv::Mat eVecs, eVals;
                int offset_tlc, offset_trc, offset_blc, offset_brc;
                cv::Point2i tlc2d(MARGIN.width - halfWinSize.width - 1,
                        MARGIN.height - halfWinSize.height - 1);

#define GETSUM(src, type, A, B, C, D) ((type *)src.data)[D] + ((type *)src.data)[A] - \
                ((type *)src.data)[B] - ((type *)src.data)[C]

                Eigen::EigenSolver<Eigen::MatrixXd> es;
                for (rect.y = tlc2d.y; rect.y < depth.rows - MARGIN.height; ++rect.y, ++winCenter.y) {
                    winCenter.x = MARGIN.width + 1;
                    offset_tlc = rect.y * (width + 1) + tlc2d.x;
                    offset_trc = offset_tlc + rect.width;
                    offset_blc = offset_tlc + rect.height * (width + 1);
                    offset_brc = offset_blc + rect.width;
                    normptr = ((cv::Vec3f*)normals.data);
                    normptr += rect.y * width + tlc2d.x;
                    for (rect.x = tlc2d.x; rect.x < depth.cols - MARGIN.width; ++rect.x, ++winCenter.x) {
                        //numValidWinPts = getSum<float>(numValidPts, rect);
                        numValidWinPts = GETSUM(numValidPts, float, offset_tlc, offset_trc, offset_blc, offset_brc);
                        //if (abs(numValidWinPts - numValidWinPts2) > 0) {
                        // std::cout << "num valid = " << numValidWinPts
                        //  << " num valid2 = " << numValidWinPts2 << std::endl;
                        //}
                        if (numValidWinPts == numPts) {
                            s_ptr = (double *) sVals.data;
                            *s_ptr++ = GETSUM(tan_theta_x_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_xy, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_x, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_x_div_z, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = sVals.at<double>(0, 1);
                            *s_ptr++ = GETSUM(tan_theta_y_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_y, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = GETSUM(tan_theta_y_div_z, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = sVals.at<double>(0, 2);
                            *s_ptr++ = sVals.at<double>(1, 2);
                            *s_ptr++ = numPts;
                            *s_ptr++ = GETSUM(one_div_z, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            *s_ptr++ = sVals.at<double>(0, 3);
                            *s_ptr++ = sVals.at<double>(1, 3);
                            *s_ptr++ = sVals.at<double>(2, 3);
                            *s_ptr++ = GETSUM(one_div_z_sq, double, offset_tlc, offset_trc, offset_blc, offset_brc);
                            //std::cout << "svals = " << sVals << std::endl;
                            //directly use the buffer allocated by OpenCV
                            //Eigen::Map<Eigen::Matrix4d> eigenT((double *) sVals.data);
                            //es.compute(eigenT);
                            //std::cout << "The eigenvalues of A are:" << std::endl << es.eigenvalues() << std::endl;
                            //std::cout << "The matrix of eigenvectors, V, is:" << std::endl << es.eigenvectors() << std::endl;
                            cv::eigen(sVals, eVals, eVecs);
                            //std::cout << "evecs = " << eVecs << std::endl;
                            //std::cout << "evals = " << eVals << std::endl;
                            cv::Mat coeffs = cv::Mat(eVecs, Rect(0, 3, eVecs.cols, 1));
                            coeffs_ptr = (double *) coeffs.data;
                            //std::cout << "coeffs = " << coeffs << std::endl;
                            float normf = 0;
                            for (int dim = 0; dim < 3; ++dim) {
                                normf += (*coeffs_ptr)*(*coeffs_ptr);
                                coeffs_ptr++;
                            }
                            normf = sqrt(normf);
                            coeffs /= normf;
                            //std::cout << "normf = " << normf << std::endl;
                            //std::cout << "coeffs = " << coeffs << std::endl;
                            float error = sqrt(eVals.at<double>(3, 0)) / (normf * numPts);
                            (*normptr)[0] = ((double *) coeffs.data)[0];
                            (*normptr)[1] = ((double *) coeffs.data)[1];
                            (*normptr)[2] = ((double *) coeffs.data)[2];
                            normptr++;
                            //std::cout << "normval = " << normals.at<cv::Vec3f>(winCenter.y, winCenter.x) << std::endl;
                        }
                        offset_tlc++;
                        offset_trc++;
                        offset_blc++;
                        offset_brc++;
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
                if (!staticDataOK()) {
                    iImgs.initialize(width, height, cx, cy, inv_f);
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
                int normalWinSize = 7;
                int normalMethod = RgbdNormals::RGBD_NORMALS_METHOD_FALS; // 7.0 fps
                //int normalMethod = RgbdNormals::RGBD_NORMALS_METHOD_LINEMOD;
                //int normalMethod = RgbdNormals::RGBD_NORMALS_METHOD_SRI; // 1.14 fps
                cv::Ptr<RgbdNormals> normalsComputer;
                normalsComputer = makePtr<RgbdNormals>(getDepth().rows,
                        getDepth().cols,
                        getDepth().depth(),
                        getCameraMatrix(),
                        normalWinSize,
                        normalMethod);
                setNormalsComputer(normalsComputer);

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
                //(*normalsComputer)(points, normals);

                cv::Mat normals2(getDepth().size(), CV_32FC3);
                iImgs.compute(getDepth(), normals2);

                //                cv::Size winSize(11, 11); // width, height
                //                cv::Size halfWinSize(winSize.width >> 1, winSize.height >> 1);
                //                cv::Size MARGIN(310, 220);
                //                //cv::Size MARGIN(winSize.width + 2, winSize.height + 2);
                //                cv::Rect rect(MARGIN.width - halfWinSize.width - 1,
                //                        MARGIN.height - halfWinSize.height - 1,
                //                        winSize.width, winSize.height);
                //                cv::Point2i winCenter(MARGIN.width, MARGIN.height);
                //                cv::Point2i tlc2d(MARGIN.width - halfWinSize.width - 1,
                //                        MARGIN.height - halfWinSize.height - 1);
                //                for (rect.y = tlc2d.y; rect.y < getDepth().rows - MARGIN.height; ++rect.y, ++winCenter.y) {
                //                    winCenter.x = MARGIN.width;
                //                    for (rect.x = tlc2d.x; rect.x < getDepth().cols - MARGIN.width; ++rect.x, ++winCenter.x) {
                //                        std::cout << "normals1(" << winCenter.x << ","
                //                                << winCenter.y << ") = " <<
                //                                normals.at<cv::Vec3f>(winCenter.y, winCenter.x);
                //                        std::cout << " normals2(" << winCenter.x << ","
                //                                << winCenter.y << ") = " << normals2.at<cv::Vec3f>(winCenter.y, winCenter.x);
                //                        std::cout << " normals diff_error("
                //                                << (normals.at<cv::Vec3f>(winCenter.y, winCenter.x) - normals2.at<cv::Vec3f>(winCenter.y, winCenter.x))
                //                                << ")" << std::endl;
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

