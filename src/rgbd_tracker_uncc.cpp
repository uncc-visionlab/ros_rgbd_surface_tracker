/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */

#include <GL/glut.h>
#include <GL/gl.h>

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

        void OpenGLRenderer::pushObject(std::string name, ObjectGeometry geom) {
            geomList.insert({name, geom});
        }

        void OpenGLRenderer::post() {
            glutPostRedisplay(); //to force "draw" function to be called
            //glutMainLoopEvent(); //freeglut function                
            glutMainLoop(); //freeglut function                
        }

        void OpenGLRenderer::renderGeometry(std::pair<std::string, ObjectGeometry> mapElement) {
            ObjectGeometry geom = mapElement.second;
            for (auto vert : geom.verts) {
                glVertex3f(vert[0], vert[1], vert[2]);
            }
        }

        void OpenGLRenderer::draw() {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgSeg.cols, imgSeg.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, imgSeg.data);

            glBegin(GL_POLYGON);
            //            glTexCoord2f(0.0f, 0.0f);
            //            glVertex2f(-1.0f, -1.0f);
            //            glTexCoord2f(1.0f, 0.0f);
            //            glVertex2f(1.0f, -1.0f);
            //            glTexCoord2f(1.0f, 1.0f);
            //            glVertex2f(1.0f, 1.0f);
            //            glTexCoord2f(0.0f, 1.0f);
            //            glVertex2f(-1.0f, 1.0f);
            for (auto geom : geomList) {
                renderGeometry(geom);
            }
            glTexCoord2f(0.0f, 0.0f);
            glVertex2f(-.50f, -.50f);
            glTexCoord2f(1.0f, 0.0f);
            glVertex2f(.50f, -.50f);
            glTexCoord2f(1.0f, 1.0f);
            glVertex2f(.50f, .50f);
            glTexCoord2f(0.0f, 1.0f);
            glVertex2f(-.50f, .50f);
            glEnd();

            glFlush();

            glutSwapBuffers();
        }

        int OpenGLRenderer::init(int width, int height) {
            //general initialization:
            char name[25] = "OpenGL Renderer\0";
            char **argv;
            int argc = 1;
            imgSeg.create(height, width, CV_8UC3);
            imgSeg = Scalar(0, 255, 0);
            argv = (char **) malloc(sizeof (char *));
            *argv = (char *) malloc(sizeof (char)*10);
            strcpy(*argv, name);

            glutInit(&argc, argv); //we cheat it ;P

            glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
            glutInitWindowPosition(800, 400);
            glutInitWindowSize(width, height);
            glutCreateWindow("OpenGL Rendered Segmentation");

            //glutDisplayFunc(draw);
            //glutIdleFunc(draw);

            //initializating the texture mapping
            GLuint mTexture;
            glGenTextures(1, &mTexture);
            glBindTexture(GL_TEXTURE_2D, mTexture);
            glEnable(GL_TEXTURE_2D);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

            glClearColor(1.0f, 1.0f, 1.0f, 1.0f); //color blanco de fondo
            glColor3f(0.0f, 0.0f, 0.0f);

            return 0;
        }

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
            } else {
                glDraw.draw();
                //glDraw.post();
            }
            rgbd_img.computeNormals(rgb_result);
            //iterativeAlignment(rgbd_img, rgb_result);


#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      

        }
    } /* namespace rgbd */
} /* namespace cv */
