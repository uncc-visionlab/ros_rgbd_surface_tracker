/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */

#include <GL/gl.h>
//#include <GL/glu.h>
#include <GL/glut.h>

#include <opencv2/highgui.hpp>
#include <opencv2/rgbd.hpp>

#include <ros_rgbd_surface_tracker/AlgebraicSurface.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

extern int supermain(AlgebraicSurface<float>& surf, PlaneVisualizationData& vis_data,
        const Eigen::Matrix<float, 8, 3>& cube, float cubesize, float levelset);

namespace cv {
    namespace rgbd {

        // construct static class member for OpenGL based rendering of scene objects
        OpenGLRenderer RgbdSurfaceTracker::glDraw;

        void RgbdSurfaceTracker::callback(cv::Mat& _ocv_rgbframe, cv::Mat& _ocv_depthframe_float,
                cv::Mat& _rgb_distortionCoeffs, cv::Mat& _rgb_cameraMatrix) {
            float cx = _rgb_cameraMatrix.at<float>(0, 2);
            float cy = _rgb_cameraMatrix.at<float>(1, 2);
            float fx = _rgb_cameraMatrix.at<float>(0, 0);
            float fy = _rgb_cameraMatrix.at<float>(1, 1);
            cv::rgbd::RgbdImage rgbd_img(_ocv_rgbframe, _ocv_depthframe_float, cx, cy, fx);
            cv::Mat rgb_result = _ocv_rgbframe;
            segmentDepth(rgbd_img, rgb_result);
            cv::imshow("RGB Result", rgb_result);
            cv::waitKey(3);
        }

        void RgbdSurfaceTracker::segmentDepth(cv::rgbd::RgbdImage& rgbd_img, cv::Mat& rgb_result) {

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif
            if (!glDraw.initialized()) {
               glDraw.init(rgbd_img.getWidth(), rgbd_img.getHeight());
            }

            std::vector<cv::rgbd::AlgebraicSurfacePatch> surfletList;
            surfdetector.detect(rgbd_img, surfletList);

            std::vector<cv::rgbd::ObjectGeometry> geomList;
            surfdescriptor_extractor.compute(rgbd_img, surfletList, geomList);

            glDraw.setImage(rgbd_img.getRGB());
            glDraw.renderGeometries(geomList);

            std::vector<cv::DMatch> geometricMatches;
            surfmatcher.match(geomList, geomList, geometricMatches);

            //iterativeAlignment(rgbd_img, rgb_result);


#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      

        }
    } /* namespace rgbd */
} /* namespace cv */
