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

        void SurfaceDetector::detect(const cv::rgbd::RgbdImage& rgbd_img,
                std::vector<AlgebraicSurfacePatch>& geometries,
                cv::Mat mask) const {
        }

        void SurfaceDescriptorExtractor::compute(const cv::rgbd::RgbdImage& rgbd_img,
                std::vector<AlgebraicSurfacePatch>& surflets,
                std::vector<ObjectGeometry>& geometries) const {

            ObjectGeometry geom;
            Box bb(cv::Vec3f(1, 1, 1),
                    Pose(cv::Vec3f(0, 0, 1), cv::Vec3f(0, 0, CV_PI / 4)));

            std::vector<cv::Vec3f> pts = bb.generateCoords();
            std::vector<cv::Vec3i> triIdxList = bb.generateCoordIndices();
            for (cv::Vec3i triIdxs : triIdxList) {
                geom.verts.push_back(pts[triIdxs[0]]);
                geom.verts.push_back(pts[triIdxs[1]]);
                geom.verts.push_back(pts[triIdxs[2]]);
            }
            std::vector<cv::Vec3f> colors = bb.generateColorCoords();
            std::vector<cv::Vec3i> triColorIdxList = bb.generateColorCoordIndices();
            for (cv::Vec3i triColorIdxs : triColorIdxList) {
                geom.colors.push_back(colors[triColorIdxs[0]]);
                geom.colors.push_back(colors[triColorIdxs[1]]);
                geom.colors.push_back(colors[triColorIdxs[2]]);
            }
            geometries.push_back(geom);
        }

        void SurfaceDescriptorMatcher::match(std::vector<ObjectGeometry> queryDescriptors,
                std::vector<ObjectGeometry> trainDescriptors,
                std::vector<cv::DMatch>& matches, cv::Mat mask) {

        }

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
            for (int idx = 0; idx < geom.verts.size(); ++idx) {
                cv::Vec3f vert = geom.verts[idx];
                cv::Vec3f color = geom.colors[idx];
                glColor3f(color[0], color[1], color[2]);
                glVertex3f(vert[0], vert[1], vert[2]);
            }
        }

        void OpenGLRenderer::draw() {
            static GLfloat xRotated = 0, yRotated = 0, zRotated = 0;
            xRotated += 1;
            zRotated += 2;
            zRotated += 1.5;
            glMatrixMode(GL_MODELVIEW);
            // clear the drawing buffer.
            //  Clear screen and Z-buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glLoadIdentity();
            //glTranslatef(0.0, 0.0, -10.5);
            glRotatef(xRotated, 1.0, 0.0, 0.0);
            // rotation about Y axis
            glRotatef(yRotated, 0.0, 1.0, 0.0);
            // rotation about Z axis
            glRotatef(zRotated, 0.0, 0.0, 1.0);

            glEnable(GL_TEXTURE_2D);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgSeg.cols, imgSeg.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, imgSeg.data);

            glBegin(GL_POLYGON);
            glTexCoord2f(0.0f, 1.0f);
            glVertex2f(-1.0f, -1.0f);
            glTexCoord2f(1.0f, 1.0f);
            glVertex2f(1.0f, -1.0f);
            glTexCoord2f(1.0f, 0.0f);
            glVertex2f(1.0f, 1.0f);

            glTexCoord2f(0.0f, 1.0f);
            glVertex2f(-1.0f, -1.0f);
            glTexCoord2f(1.0f, 0.0f);
            glVertex2f(1.0f, 1.0f);
            glTexCoord2f(0.0f, 0.0f);
            glVertex2f(-1.0f, 1.0f);
            glEnd();

            glDisable(GL_TEXTURE_2D);
            glBegin(GL_TRIANGLES);
            for (auto geom : geomList) {
                renderGeometry(geom);
            }
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
            glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
            glutInitWindowPosition(800, 400);
            glutInitWindowSize(width, height);
            glutCreateWindow("OpenGL Rendered Segmentation");
            //glutDisplayFunc(draw);
            //glutIdleFunc(draw);

            //initializating the texture mapping
            GLuint mTexture;
            glGenTextures(1, &mTexture);
            glBindTexture(GL_TEXTURE_2D, mTexture);
            glEnable(GL_DEPTH_TEST); // Enable depth testing for z-culling
            glDepthFunc(GL_LEQUAL); // Set the type of depth-test
            glShadeModel(GL_SMOOTH); // Enable smooth shading
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
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
            }

            rgbd_img.computeNormals(rgb_result);

            std::vector<cv::rgbd::AlgebraicSurfacePatch> surfletList;
            surfdetector.detect(rgbd_img, surfletList);

            std::vector<cv::rgbd::ObjectGeometry> geomList;
            surfdescriptor_extractor.compute(rgbd_img, surfletList, geomList);

            std::vector<cv::DMatch> geometricMatches;
            surfmatcher.match(geomList, geomList, geometricMatches);

            int idx = 0;
            for (cv::rgbd::ObjectGeometry geom : geomList) {
                std::string geomName("Box " + std::to_string(idx++));
                glDraw.pushObject(geomName, geom);
            }
            glDraw.setImage(rgbd_img.getRGB());
            glDraw.draw();
            glDraw.clearObjects();
            //iterativeAlignment(rgbd_img, rgb_result);


#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      

        }
    } /* namespace rgbd */
} /* namespace cv */
