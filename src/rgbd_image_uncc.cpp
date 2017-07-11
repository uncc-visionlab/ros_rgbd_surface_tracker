/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <opencv2/core/core.hpp>
namespace cv {
    namespace rgbd {

        // force construction of the static class member variable
        DepthIntegralImages RgbdImage::iImgs;

        bool RgbdImage::staticDataOK() {
            return cv::rgbd::RgbdImage::iImgs.getHeight() == height &&
                    cv::rgbd::RgbdImage::iImgs.getWidth() == width;
        }

        void DepthIntegralImages::plucker(cv::Mat& points, cv::Mat& normals,
                cv::Mat& axisVecs, cv::Mat& axisDirs) {
            static cv::Mat quadraticConstraints = cv::Mat::zeros(6, 6, CV_32F);
            quadraticConstraints.at<float>(0, 0) = 1.0f;
            quadraticConstraints.at<float>(1, 1) = 1.0f;
            quadraticConstraints.at<float>(2, 2) = 1.0f;
            cv::Mat pluckerScatter = cv::Mat::zeros(6, 6, CV_32F);
            float normf;
            cv::Point2i tlc(315, 235);
            cv::Rect roi(tlc.x, tlc.y, width - tlc.x, height - tlc.y);
            for (int y = roi.y; y < roi.y + roi.height; ++y) {
                cv::Vec3f* pt_ptr = points.ptr<cv::Vec3f>(y, roi.x);
                cv::Vec3f* norm_ptr = normals.ptr<cv::Vec3f>(y, roi.x);
                for (int x = roi.x; x < roi.x + roi.width; ++x, ++pt_ptr, ++norm_ptr) {
                    cv::Vec3f& pt = *pt_ptr;
                    cv::Vec3f& norm = *norm_ptr;
                    if (std::isnan(norm[0]) || std::isnan(pt[0])) {
                        continue;
                    }
                    cv::Vec6f pluckerLineVec(norm[0], norm[1], norm[2],
                            pt[1] * norm[2] - pt[2] * norm[1],
                            pt[2] * norm[0] - pt[0] * norm[2],
                            pt[0] * norm[1] - pt[1] * norm[2]);
                    //normf = 1.0f / std::sqrt(pluckerLineVec.dot(pluckerLineVec));
                    //pluckerLineVec *= normf;
                    // form upper triangular scatter matrix
                    float *pluckerScatter_ptr = pluckerScatter.ptr<float>(0, 0);
                    for (int row = 0; row < 6; ++row) {
                        pluckerScatter_ptr += row;
                        for (int col = row; col < 6; ++col, ++pluckerScatter_ptr) {
                            *pluckerScatter_ptr += pluckerLineVec[row] * pluckerLineVec[col];
                        }
                    }
                }
            }
            //std::cout << "pluckerScatter = " << pluckerScatter << std::endl;
            cv::completeSymm(pluckerScatter);
            //std::cout << "pluckerScatter = " << pluckerScatter << std::endl;
            cv::Mat ipluckerScatter = pluckerScatter.inv();
            cv::Mat ipluckerConstrained = ipluckerScatter*quadraticConstraints;
            cv::Mat U, W, Vt;
            cv::SVDecomp(ipluckerConstrained, W, U, Vt);
            //std::cout << "U = " << U << std::endl;
            //std::cout << "W = " << W << std::endl;
            //std::cout << "Vt = " << Vt << std::endl;
            cv::Vec6f pluckerAxisLine = cv::Mat(U, cv::Rect(0, 0, 1, U.cols));
            cv::Vec3f linePt(pluckerAxisLine[0], pluckerAxisLine[1], pluckerAxisLine[2]);
            cv::Vec3f lineDir(pluckerAxisLine[3], pluckerAxisLine[4], pluckerAxisLine[5]);
            //std::cout << "pt = " << linePt << " dir = " << lineDir << std::endl;
            lineDir = lineDir.cross(linePt);
            //std::cout << "pt = " << linePt << " dir = " << lineDir << std::endl;
            normf = 1.0f / std::sqrt(lineDir.dot(lineDir));
            lineDir *= normf;
            std::cout << "pt = " << linePt << " dir = " << lineDir << std::endl;
        }

        void DepthIntegralImages::computeCurvatureFiniteDiff_Impl(const cv::Mat& depth, cv::Mat& normals) {
            float *_ones_valid_mask_ptr = _ones_valid_mask.ptr<float>(1, 1);
            //float *d2z_dx2_ptr = d2z_dx2.ptr<float>(2, 2);
            //float *d2z_dy2_ptr = d2z_dy2.ptr<float>(2, 2);
            //float *_one_div_z_ptr = _one_div_z.ptr<float>(1, 1);
            float z_ratio_x, z_ratio_y, timeElapsed;
            int64 tickBefore, tickAfter;
            float const_NAN = std::numeric_limits<float>::quiet_NaN();
            tickBefore = cv::getTickCount();
            cv::Mat dx(height, width, CV_32F);
            cv::Mat dy(height, width, CV_32F);
            for (int y = 1; y < ntan_theta_x.rows - 1; ++y) {
                const float *depth_ptr = depth.ptr<float>(y, 1);
                float *dx_ptr = dx.ptr<float>(y, 1);
                float *dy_ptr = dy.ptr<float>(y, 1);
                float *dz_dx_ptr = dz_dx.ptr<float>(y, 1);
                float *dz_dy_ptr = dz_dy.ptr<float>(y, 1);
                float *ntan_theta_x_ptr = ntan_theta_x.ptr<float>(y, 1);
                float *ntan_theta_y_ptr = ntan_theta_y.ptr<float>(y, 1);
                for (int x = 1; x < ntan_theta_x.cols - 1; ++x) {
                    if (std::isnan(depth_ptr[x])) {
                        *dx_ptr = const_NAN;
                        *dy_ptr = const_NAN;
                        *dz_dx_ptr = const_NAN;
                        *dz_dy_ptr = const_NAN;
                    } else {
                        *dx_ptr = (ntan_theta_x_ptr[1] * depth_ptr[x + 1] - ntan_theta_x_ptr[-1] * depth_ptr[x - 1]);
                        *dz_dx_ptr = (depth_ptr[x + 1] - depth_ptr[x - 1]) / (*dx_ptr);
                        *dy_ptr = (ntan_theta_y_ptr[width] * depth_ptr[x + width] - ntan_theta_y_ptr[-width] * depth_ptr[x - width]);
                        *dz_dy_ptr = (depth_ptr[x + width] - depth_ptr[x - width]) / (*dy_ptr);
                    }
                    depth_ptr++;
                    ntan_theta_x_ptr++;
                    ntan_theta_y_ptr++;
                    dx_ptr++;
                    dy_ptr++;
                    dz_dx_ptr++;
                    dz_dy_ptr++;
                }
            }
            cv::Mat dx2(height, width, CV_32F);
            cv::Mat dy2(height, width, CV_32F);
            cv::Mat dxdy(height, width, CV_32F);
            for (int y = 2; y < ntan_theta_x.rows - 2; ++y) {
                float *dx_ptr = dx.ptr<float>(y, 2);
                float *dy_ptr = dy.ptr<float>(y, 2);
                float *dx2_ptr = dx2.ptr<float>(y, 2);
                float *dy2_ptr = dy2.ptr<float>(y, 2);
                //float *dxdy_ptr = dxdy.ptr<float>(y, 2);
                float *dz_dx_ptr = dz_dx.ptr<float>(y, 2);
                float *dz_dy_ptr = dz_dy.ptr<float>(y, 2);
                float *d2z_dx2_ptr = d2z_dx2.ptr<float>(y, 2);
                //float *d2z_dxdy_ptr = d2z_dx2.ptr<float>(y, 2);
                float *d2z_dy2_ptr = d2z_dy2.ptr<float>(y, 2);
                for (int x = 2; x < ntan_theta_x.cols - 2; ++x) {
                    if (std::isnan(dz_dx_ptr[0])) {
                        *dx2_ptr = const_NAN;
                        *dy2_ptr = const_NAN;
                        //*dxdy_ptr = const_NAN;
                        *d2z_dx2_ptr = const_NAN;
                        //*d2z_dxdy_ptr = const_NAN;
                        *d2z_dy2_ptr = const_NAN;
                    } else {
                        *dx2_ptr = (dx_ptr[1] - dx_ptr[-1]);
                        *d2z_dx2_ptr = (dz_dx_ptr[1] - dz_dx_ptr[-1]) / (*dx2_ptr);
                        *dy2_ptr = (dy_ptr[1] - dy_ptr[-1]);
                        *d2z_dy2_ptr = (dz_dy_ptr[width] - dz_dy_ptr[-width]) / (*dy2_ptr);
                        //float posCross = sqrt((dx_ptr[width + 1] - dx_ptr[-width - 1])*(dx_ptr[width + 1] - dx_ptr[-width - 1])
                        //        + (dy_ptr[width + 1] - dy_ptr[-width - 1])*(dy_ptr[width + 1] - dy_ptr[-width - 1]));
                        //float negCross = sqrt((dx_ptr[-width + 1] - dx_ptr[width - 1])*(dx_ptr[-width + 1] - dx_ptr[width - 1])
                        //        + (dy_ptr[width - 1] - dy_ptr[-width + 1])*(dy_ptr[width - 1] - dy_ptr[-width + 1]));
                        //*dxdy_ptr = posCross*negCross;
                        //*d2z_dxdy_ptr = (dz_dy_ptr[width + 1] - dz_dy_ptr[-width - 1]) / (*dxdy_ptr);
                    }
                    dx_ptr++;
                    dy_ptr++;
                    dx2_ptr++;
                    dy2_ptr++;
                    dz_dx_ptr++;
                    dz_dy_ptr++;
                    d2z_dx2_ptr++;
                    d2z_dy2_ptr++;
                }
            }
            //cv::Mat idz_dx, idz_dx_sq;
            //cv::integral(dz_dx, idz_dx, idz_dx_sq, CV_64F, CV_64F);
            tickAfter = cv::getTickCount();
            timeElapsed = (float) (tickAfter - tickBefore) / ((float) cv::getTickFrequency());
            std::cout << "Time elapsed 2 = " << timeElapsed << std::endl;
        }

    } /* namespace rgbd */
} /* namespace cv */

