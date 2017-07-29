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

#define BLOCKSIZE 32
#define MARGIN_X 50
#define MARGIN_Y 30

namespace cv {
    namespace rgbd {

        // construct static class member for OpenGL based rendering of scene objects
        OpenGLRenderer RgbdSurfaceTracker::glDraw;
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

        void testHypotheses(cv::rgbd::RgbdImage& rgbd_img,
                cv::QuadTree<sg::Plane<float>::Ptr>& quadtree,
                cv::Rect2i& roi, std::vector<cv::rgbd::ObjectGeometry>& geometry_list) {

            for (cv::rgbd::ObjectGeometry& geometry : geometry_list) {
                switch (geometry.getSurfaceType()) {
                    case SurfaceType::EDGE:
                        sg::Edge<float>::Ptr edge = boost::static_pointer_cast<sg::Edge<float>>(geometry.getShape());
                        cv::Point3f midpoint_edge = 0.5 * (edge->getPoint(edge->start) + edge->getPoint(edge->end));
                        cv::Point2f img_midpoint_edge = rgbd_img.project(midpoint_edge);
                        std::cout << "img_midpoint: " << img_midpoint_edge << std::endl;
                        //getBlockOfPoint(img_midpoint_edge, quadtree, roi);
                        break;
                }
            }
        }

        void RgbdSurfaceTracker::segmentDepth(cv::rgbd::RgbdImage& rgbd_img, cv::Mat& rgb_result) {

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif
            bool offscreen_rendering = false;

            if (!glDraw.initialized()) {
                glDraw.init(rgbd_img.getWidth(), rgbd_img.getHeight(), offscreen_rendering);
            }

            cv::Rect rectVal(290, 200, 640 - 2 * 290, 480 - 2 * 200);
            cv::rectangle(rgb_result, rectVal, cv::Scalar(0, 255, 0), 3);
            rgbd_img.computeNormals();

            cv::Rect roi(MARGIN_X, MARGIN_Y, rgbd_img.getWidth() - 2 * MARGIN_X, rgbd_img.getHeight() - 2 * MARGIN_Y);
            cv::Size imgSize(rgbd_img.getWidth(), rgbd_img.getHeight());
            cv::Size tileSize(BLOCKSIZE, BLOCKSIZE);
            cv::QuadTree<sg::Plane<float>::Ptr> quadTree( imgSize, tileSize, roi);

            std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> surfletPtrList;
            surfdetector.detect(rgbd_img, quadTree, surfletPtrList, rgb_result);

            std::vector<cv::rgbd::ObjectGeometry> geomList;
            surfdescriptor_extractor.compute(rgbd_img, surfletPtrList, geomList, rgb_result);

            testHypotheses(rgbd_img, quadTree, roi, geomList);

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
                    //glDraw.renderPointCloudNormals(points, rgbd_img.getNormals(),
                    //        0.2, 0.33, false);
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
