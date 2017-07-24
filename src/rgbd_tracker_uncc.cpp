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
        //OpenGLRenderAttributes RgbdSurfaceTracker::glAttr;
        int OpenGLRenderer::specialKey;
        OpenGLRenderAttributes OpenGLRenderer::attrs;


        std::map<SurfaceType, const char*> surfaceTypeToString = {
            {SurfaceType::UNKNOWN, "Uknown"},
            {SurfaceType::PLANE, "Plane"},
            {SurfaceType::EDGE, "Edge"},
            {SurfaceType::CORNER, "Corner"},
            {SurfaceType::BOX, "Box"}
        };

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
            bool offscreen_rendering = false;

            if (!glDraw.initialized()) {
                glDraw.init(rgbd_img.getWidth(), rgbd_img.getHeight(), offscreen_rendering);
                //glDraw.setAttributes(glAttr);
            }

            cv::Rect rectVal(290, 200, 640 - 2 * 290, 480 - 2 * 200);
            cv::rectangle(rgb_result, rectVal, cv::Scalar(0, 255, 0), 3);
            rgbd_img.computeNormals();

            std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> surfletPtrList;
            surfdetector.detect(rgbd_img, surfletPtrList, rgb_result);

            std::vector<cv::rgbd::ObjectGeometry> geomList;
            surfdescriptor_extractor.compute(rgbd_img, surfletPtrList, geomList, rgb_result);

            // OpenGL rendering
            glDraw.setImage(rgbd_img.getRGB());
            glDraw.initFrame();
            
            if (glDraw.attrs.showPointcloud || glDraw.attrs.showPointcloudNormals) {
                cv::Mat points, colors;
                rgbd_img.getPointCloud(points, colors);
                if (glDraw.attrs.showPointcloud) {
                    glDraw.renderPointCloud(points, colors);
                }
                if (glDraw.attrs.showPointcloudNormals) {
                    glDraw.renderPointCloudNormals(points, rgbd_img.getNormals());
                }
            }

            if (glDraw.attrs.showDetections) {             
                glDraw.renderGeometries(geomList);
            }

            std::vector<cv::DMatch> geometricMatches;
            surfmatcher.match(geomList, geomList, geometricMatches);

            //iterativeAlignment(rgbd_img, rgb_result);

            glDraw.clearObjects();
#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      

        }
    } /* namespace rgbd */
} /* namespace cv */
