/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
namespace cv {
    namespace rgbd {

        // force construction of the static class member variable
        DepthIntegralImages RgbdImage::iImgs;

        bool RgbdImage::staticDataOK() {
            return cv::rgbd::RgbdImage::iImgs.getHeight() == height &&
                    cv::rgbd::RgbdImage::iImgs.getWidth() == width;
        }

        void DepthIntegralImages::plucker(const cv::Mat& depth, cv::Mat& mean_curvature, cv::Mat& axes) {

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

