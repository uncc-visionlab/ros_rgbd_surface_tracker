/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <opencv2/core/core.hpp>

#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

namespace cv {
    namespace rgbd {

        // force construction of the static class member variable
        DepthIntegralImages RgbdImage::iImgs;

        bool RgbdImage::staticDataOK() {
            return cv::rgbd::RgbdImage::iImgs.getHeight() == height &&
                    cv::rgbd::RgbdImage::iImgs.getWidth() == width;
        }

        void RgbdImage::setNormalsComputer(cv::Ptr<RgbdNormals> _nc) {
            normalsComputer = _nc;
        }

        cv::Ptr<RgbdNormals> RgbdImage::getNormalsComputer(cv::Ptr<RgbdNormals> _nc) {
            return normalsComputer;
        }

        void RgbdImage::getPointCloud(cv::Mat& pts, cv::Mat& colors) {
            pts.create(getDepth().size(), CV_32FC3);
            colors.create(getDepth().size(), CV_8UC3);
            for (int r = 0; r < getDepth().rows; ++r) {
                const float *depth_ptr = getDepth().ptr<float>(r, 0);
                cv::Vec3f *pts_ptr = pts.ptr<cv::Vec3f>(r, 0);
                const uchar *rgb_ptr = getRGB().ptr<uchar>(r, 0);
                uchar *colors_ptr = colors.ptr<uchar>(r, 0);
                for (int c = 0; c < getDepth().cols; ++c) {
                    (*pts_ptr)[0] = (c - cx) * inv_f * (*depth_ptr);
                    (*pts_ptr)[1] = (r - cy) * inv_f * (*depth_ptr);
                    (*pts_ptr)[2] = *depth_ptr;
                    *colors_ptr++ = *rgb_ptr++;
                    *colors_ptr++ = *rgb_ptr++;
                    *colors_ptr++ = *rgb_ptr++;
                    pts_ptr++;
                    depth_ptr++;
                }
            }
        }

        bool RgbdImage::computeNormals() const {
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
            //iImgs.computeCurvatureFiniteDiff_Impl(getDepth(), normals3);
            //                cv::Mat axisVecs(1, 3, CV_32F);
            //                cv::Mat axisDirs(1, 3, CV_32F);
            //                iImgs.plucker(points, normals3, axisVecs, axisDirs);

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

        LineSegment2f RgbdImage::getVisibleLineSegment2f(Line3f& line3) const {
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

        void RgbdImage::fitImplicitPlaneLeastSquares(std::vector<Point3f>& data3, Plane3f& plane3, float& error,
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
            cv::Vec3f xP = linePt.cross(lineDir);
            //std::cout << "pt = " << linePt << " dir = " << lineDir << std::endl;
            normf = 1.0f / std::sqrt(xP.dot(xP));
            linePt *= normf;
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

