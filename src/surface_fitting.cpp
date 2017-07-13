/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <opencv2/core.hpp>

#include <ros_rgbd_surface_tracker/surface_fitting.hpp>
#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>

namespace cv {
    namespace uncc {

        void SurfaceFitting::fitExplicitPlaneLeastSquares(std::vector<Point3f>& data3, Plane3f& plane3, float& error,
                float& noise, int& inliers, int& outliers, int& invalid) const {
            int numPoints = data3.size();
            Mat _M = Mat::zeros(numPoints, 3, DataType<float>::type);
            Mat _Z = Mat::zeros(numPoints, 1, DataType<float>::type);
            float* M = _M.ptr<float>();
            float* Z = _Z.ptr<float>();
            noise = 0;
            for (int ptIdx = 0; ptIdx < data3.size(); ++ptIdx) {
                M[3 * ptIdx] = data3[ptIdx].x;
                M[3 * ptIdx + 1] = data3[ptIdx].y;
                M[3 * ptIdx + 2] = 1.0f;
                Z[ptIdx] = data3[ptIdx].z;
                //noise += getErrorStandardDeviation(data3[ptIdx].z);
            }
            Mat _MtM = _M.t() * _M;
            Mat _planeCoeffs = _MtM.inv() * _M.t() * _Z;
            float* planeCoeffs = _planeCoeffs.ptr<float>();
            Mat _error = _M * _planeCoeffs - _Z;
            //std::cout << "E_error=" << _error << std::endl;
            //std::cout << "M = " << M << std::endl;
            //std::cout << "Z = " << Z << std::endl;
            //std::cout << "eplaneCoeffs = " << _planeCoeffs << std::endl;           
            float* _z = _Z.ptr<float>();
            float* _err = _error.ptr<float>();
            error = inliers = outliers = invalid = 0;
            //        for (int ptIdx = 0; ptIdx < numPoints; ptIdx++) {
            //            if (isInlier(_err[ptIdx], _z[ptIdx])) {
            //                inliers++;
            //            } else {
            //                outliers++;
            //            }
            //            error += abs(_error.at<float>(ptIdx));
            //            //error += abs(_z[ptIdx]);
            //        }
            //            error /= numPoints;
            //            noise /= numPoints;
            plane3.x = planeCoeffs[0];
            plane3.y = planeCoeffs[1];
            plane3.z = -1;
            plane3.d = planeCoeffs[2];
            float normScale = 1.f / sqrt(plane3.x * plane3.x + plane3.y * plane3.y + plane3.z * plane3.z);
            plane3.scale(normScale);
            //std::cout << "eplaneCoeffs = " << plane3 << " error = " << error << std::endl;
            //std::cout.flush();
        }

        void SurfaceFitting::fitImplicitPlaneLeastSquares(std::vector<Point3f>& data3, Plane3f& plane3, float& error,
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
                //noise += getErrorStandardDeviation(data3[ptIdx].z);
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
            //        for (int ptIdx = 0; ptIdx < numPoints; ptIdx++) {
            //            if (isInlier(_err[ptIdx], data3[ptIdx].z)) {
            //                inliers++;
            //            } else {
            //                outliers++;
            //            }
            //            //                error += abs(_error.at<float>(ptIdx));
            //            error += abs(_err[ptIdx]);
            //        }
            //            error /= numPoints;
            //            noise /= numPoints;
            plane3.scale((plane3.z > 0) ? -1.f : 1.f);
            //std::cout << "iplaneCoeffs = " << plane3 << " error = " << error << std::endl;
            //std::cout.flush();
        }
    } /* namespace uncc */
} /* namespace cv */