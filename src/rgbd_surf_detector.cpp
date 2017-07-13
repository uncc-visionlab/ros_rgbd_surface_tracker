/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <vector>

#include <opencv2/core.hpp>

#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

namespace cv {
    namespace rgbd {

        void SurfaceDetector::detect(const cv::rgbd::RgbdImage& rgbd_img,
                std::vector<AlgebraicSurfacePatch>& geometries,
                cv::Mat mask) const {          
            rgbd_img.computeNormals();
        }


#define GETSUM(src, type, A, B, C, D) ((type *)src.data)[D] + ((type *)src.data)[A] - \
                ((type *)src.data)[B] - ((type *)src.data)[C]                

        void DepthIntegralImages::computeMat_LUT() {
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

        void DepthIntegralImages::initialize(int _width, int _height, float _cx, float _cy, float _inv_f,
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

        void DepthIntegralImages::operator()(cv::InputArray depth_in, cv::OutputArray normals_out) {
            Mat depth_ori = depth_in.getMat();

            // Get the normals
            normals_out.create(depth_ori.size(), CV_32FC3);
            if (depth_in.empty())
                return;

            Mat normals = normals_out.getMat();
            computeExplicit_Impl(depth_ori, normals);
        }

        void DepthIntegralImages::computeImplicit_Impl(const cv::Mat& depth, cv::Mat & normals) {
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

        void DepthIntegralImages::computeExplicit_Impl(const cv::Mat& depth, cv::Mat & normals) {
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
    } /* namespace rgbd */
} /* namespace cv */