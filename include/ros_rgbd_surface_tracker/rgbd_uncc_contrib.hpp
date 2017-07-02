/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rgbd_uncc_contrib.h
 * Author: arwillis
 *
 * Created on June 30, 2017, 10:01 AM
 */

#ifndef RGBD_UNCC_CONTRIB_H
#define RGBD_UNCC_CONTRIB_H

#ifdef __cplusplus

#include <limits>

#include <opencv2/core.hpp>

#include <boost/shared_ptr.hpp>

namespace cv {
    namespace rgbd {

        class RgbdSurfaceTracker {
        public:
            typedef boost::shared_ptr<RgbdSurfaceTracker> Ptr;

            RgbdSurfaceTracker() {
            };

            virtual ~RgbdSurfaceTracker() {
            };

            void setRGBCameraIntrinsics(cv::Mat &_cameraMatrix, cv::Size &_size) {
                cameraMatrix = _cameraMatrix;
                size = _size;
            }

            void setRGBCameraDistortion(cv::Mat &_distortionCoeffs) {
                distortionCoeffs = _distortionCoeffs;
            }

            void setDepthImage(cv::UMat &_rawDepth) {
                rawDepth = _rawDepth;
            }

            void setRGBImage(cv::UMat &_rawRGB) {
                rawDepth = _rawRGB;
            }
            void segmentDepth();
            // { std::cout << "Called segmentDepth()" << std::endl;}

//            void callback(cv::Mat _ocv_rgbframe, cv::Mat _ocv_depthframe_float,
//                    cv::Mat _rgb_distortionCoeffs, cv::Mat _rgb_cameraMatrix) {
//                cv::Size imSize(_ocv_rgbframe.cols, _ocv_rgbframe.rows);
//                rgbdSurfTracker.setRGBCameraDistortion(_rgb_distortionCoeffs);
//                rgbdSurfTracker.setRGBCameraIntrinsics(_rgb_cameraMatrix, imSize);
//                cv::UMat di = _ocv_depthframe_float.getUMat(cv::ACCESS_WRITE);
//                rgbdSurfTracker.setDepthImage(di);
//                cv::UMat rgb = _ocv_rgbframe.getUMat(cv::ACCESS_WRITE);
//                rgbdSurfTracker.setRGBImage(rgb);
//                rgbdSurfTracker.segmentDepth();
//            }
            void callback(cv::Mat _ocv_rgbframe, cv::Mat _ocv_depthframe_float,
                    cv::Mat _rgb_distortionCoeffs, cv::Mat _rgb_cameraMatrix) {
                cv::Size imSize(_ocv_rgbframe.cols, _ocv_rgbframe.rows);
                setRGBCameraDistortion(_rgb_distortionCoeffs);
                setRGBCameraIntrinsics(_rgb_cameraMatrix, imSize);
                cv::UMat di = _ocv_depthframe_float.getUMat(cv::ACCESS_WRITE);
                setDepthImage(di);
                cv::UMat rgb = _ocv_rgbframe.getUMat(cv::ACCESS_WRITE);
                setRGBImage(rgb);
                segmentDepth();
            }

        private:
            // -------------------------
            // Disabling default copy constructor and default
            // assignment operator.
            // -------------------------
            RgbdSurfaceTracker(const RgbdSurfaceTracker& ref);
            RgbdSurfaceTracker& operator=(const RgbdSurfaceTracker& ref);

            cv::Mat cameraMatrix, distortionCoeffs;
            cv::UMat rawDepth, rawRGB;
            cv::Size size;
        };
    } /* namespace rgbd */
} /* namespace cv */

#endif /* __cplusplus */
#endif /* RGBD_UNCC_CONTRIB_H */

