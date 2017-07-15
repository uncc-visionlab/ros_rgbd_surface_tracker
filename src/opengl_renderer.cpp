/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */

#include <GL/glu.h>
#include <GL/glut.h>

#include <Eigen/Dense>

#include <ros_rgbd_surface_tracker/opengl_renderer.hpp>

namespace cv {
    namespace rgbd {

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
                cv::Vec3f normal = geom.normals[idx];
                cv::Vec3f color = geom.colors[idx];
                glColor3f(color[0], color[1], color[2]);
                glNormal3f(normal[0], normal[1], normal[2]);
                glVertex3f(vert[0], vert[1], vert[2]);
            }
        }

        void OpenGLRenderer::renderGeometries(std::vector<cv::rgbd::ObjectGeometry> geomList) {
            int idx = 0;
            for (cv::rgbd::ObjectGeometry geom : geomList) {
                std::string geomName("Box " + std::to_string(idx++));
                pushObject(geomName, geom);
            }
            draw();
            clearObjects();
        }

        void OpenGLRenderer::draw() {
            static GLfloat sign = 1, xRotated = 1, yRotated = 1, zRotated = 1;

            //glDisable(GL_LIGHTING);
            //glDepthMask(GL_FALSE);
            glMatrixMode(GL_MODELVIEW);

            //  Clear screen and Z-buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glEnable(GL_TEXTURE_2D);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgSeg.cols, imgSeg.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, imgSeg.data);

            glBegin(GL_POLYGON);
            glTexCoord2f(0.0f, 1.0f);
            glVertex3f(-1.0f, -1.0f, 10.0f);
            glTexCoord2f(1.0f, 1.0f);
            glVertex3f(1.0f, -1.0f, 10.0f);
            glTexCoord2f(1.0f, 0.0f);
            glVertex3f(1.0f, 1.0f, 10.0f);
            glTexCoord2f(0.0f, 1.0f);
            glVertex3f(-1.0f, -1.0f, 10.0f);
            glTexCoord2f(1.0f, 0.0f);
            glVertex3f(1.0f, 1.0f, 10.0f);
            glTexCoord2f(0.0f, 0.0f);
            glVertex3f(-1.0f, 1.0f, 10.0f);
            glEnd();

            //            glPushMatrix();

            glDisable(GL_TEXTURE_2D);
            //glEnable(GL_LIGHTING);

            //glLoadIdentity();
            //glTranslatef(xRotated, 0, 0);
            //glTranslatef(0, xRotated, 0);
            //glTranslatef(0, 0, xRotated);

            //glTranslatef(0.0, 0.0, -3.5);
            //glRotatef(xRotated, 1.0, 0.0, 0.0);
            //glTranslatef(0.0, 0.0, 3.5);
            // rotation about Y axis
            //glRotatef(yRotated, 0.0, 1.0, 0.0);
            // rotation about Z axis
            //glRotatef(zRotated, 0.0, 0.0, 1.0);

            glBegin(GL_TRIANGLES);
            for (auto geom : geomList) {
                renderGeometry(geom);
            }
            glEnd();
            //            glPopMatrix();

            glFlush();

            glutSwapBuffers();
        }

        int OpenGLRenderer::init(int width, int height) {
            int argc = 0;
            imgSeg.create(height, width, CV_8UC3);
            imgSeg = Scalar(0, 255, 0);


            glutInit(&argc, NULL);
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

            GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0}; /* Red diffuse light. */
            GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0}; /* Infinite light location. */

            /* Enable a single OpenGL light. */
                        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
                        glLightfv(GL_LIGHT0, GL_POSITION, light_position);
                        glEnable(GL_LIGHT0);
                        glEnable(GL_LIGHTING);
            /* Setup the view of the cube. */
            //            glMatrixMode(GL_PROJECTION);
            //            gluPerspective(/* field of view in degree */ 58.0,
            //                    /* aspect ratio */((float)width)/height,
            //                    /* Z near */ 0.10, /* Z far */ 100.0);
            //            GLfloat model[16]; 
            //            glGetFloatv(GL_PROJECTION_MATRIX, model); 
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            GLint viewport[4];
            glGetIntegerv(GL_VIEWPORT, &viewport[0]);
            Eigen::Matrix4d frustum;
            double fx = 567.8377685546875, fy = 567.8377685546875, skew = 0, cx = 319.5, cy = 239.5;
            int img_width = 640, img_height = 480;
            double near_clip = 0.01, far_clip = 3000;
            build_opengl_projection_for_intrinsics(frustum, &viewport[0],
                    fx, fy, skew, cx, cy, img_width, img_height, near_clip, far_clip);
            std::cout << "frustrum = " << frustum << std::endl;
            glLoadMatrixd(&frustum(0, 0));

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            gluLookAt(0.0, 0.0, 0.0, /* eye is at (0,0,5) */
                    0.0, 0.0, 1.0, /* center is at (0,0,0) */
                    0.0, 1.0, 0.); /* up is in negative Y direction */
            // looking down positive Z axis with Y axis pointing down, X axis pointing right
            cv::Mat cameraExtrinsics = (cv::Mat_<float>(4, 4) <<
                    1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, -1, 0,
                    0, 0, 0, 1);
            std::cout << "ext1 " << cameraExtrinsics << std::endl;
            //ros2opengl(cameraExtrinsics);
            std::cout << "ext2 " << cameraExtrinsics << std::endl;
            //glLoadMatrixf(cameraExtrinsics.ptr<float>(0, 0));
            GLfloat model[16];
            glGetFloatv(GL_MODELVIEW_MATRIX, model);
            cv::Mat model2(4, 4, CV_32F);
            float *m2ptr = model2.ptr<float>(0, 0);
            for (int i = 0; i < 16; ++i) {
                *m2ptr++ = model[i];
            }
            std::cout << "view = " << model2 << std::endl;
            /* Setup the view of the cube. */

            glMatrixMode(GL_MODELVIEW);
//                        gluLookAt(0.0, 0.0, 0.0, /* eye is at (0,0,5) */
//                                0.0, 0.0, 5.0, /* center is at (0,0,0) */
//                                0.0, -1.0, 0.); /* up is in positive Y direction */
            //            //gluPerspective (50.0*zoomFactor, (float)img_width/(float)img_height, near_clip, far_clip);
            //glLoadMatrixf(projectionMat.ptr<float>(0, 0)); // Projection
            //            glOrtho(clippingPlanes.at<float>(0,0), clippingPlanes.at<float>(0,1), 
            //                    clippingPlanes.at<float>(1,0), clippingPlanes.at<float>(1,0),
            //                    clippingPlanes.at<float>(2,0), clippingPlanes.at<float>(2,1));

            return 0;
        }

        void OpenGLRenderer::ros2opengl(cv::Mat& rotMat) {
            static cv::Mat flipYandZ = (cv::Mat_<float>(4, 4) <<
                    1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, -1, 0,
                    0, 0, 0, 1);
            rotMat = flipYandZ*rotMat;
        }

        /**
         @brief basic function to produce an OpenGL projection matrix and associated viewport parameters
         which match a given set of camera intrinsics. This is currently written for the Eigen linear
         algebra library, however it should be straightforward to port to any 4x4 matrix class.
         @param[out] frustum Eigen::Matrix4d projection matrix.  Eigen stores these matrices in column-major (i.e. OpenGL) order.
         @param[out] viewport 4-component OpenGL viewport values, as might be retrieved by glGetIntegerv( GL_VIEWPORT, &viewport[0] )
         @param[in]  alpha x-axis focal length, from camera intrinsic matrix
         @param[in]  alpha y-axis focal length, from camera intrinsic matrix
         @param[in]  skew  x and y axis skew, from camera intrinsic matrix
         @param[in]  u0 image origin x-coordinate, from camera intrinsic matrix
         @param[in]  v0 image origin y-coordinate, from camera intrinsic matrix
         @param[in]  img_width image width, in pixels
         @param[in]  img_height image height, in pixels
         @param[in]  near_clip near clipping plane z-location, can be set arbitrarily > 0, controls the mapping of z-coordinates for OpenGL
         @param[in]  far_clip  far clipping plane z-location, can be set arbitrarily > near_clip, controls the mapping of z-coordinate for OpenGL
         */
        void OpenGLRenderer::build_opengl_projection_for_intrinsics(Eigen::Matrix4d &frustum, int *viewport,
                double alpha, double beta, double skew, double u0, double v0,
                int img_width, int img_height, double near_clip, double far_clip) {

            // These parameters define the final viewport that is rendered into by
            // the camera.
            double L = 0;
            double R = img_width;
            double B = 0;
            double T = img_height;

            // near and far clipping planes, these only matter for the mapping from
            // world-space z-coordinate into the depth coordinate for OpenGL
            double N = near_clip;
            double F = far_clip;

            // set the viewport parameters
            viewport[0] = L;
            viewport[1] = B;
            viewport[2] = R - L;
            viewport[3] = T - B;

            // construct an orthographic matrix which maps from projected
            // coordinates to normalized device coordinates in the range
            // [-1, 1].  OpenGL then maps coordinates in NDC to the current
            // viewport
            Eigen::Matrix4d ortho = Eigen::Matrix4d::Zero();
            ortho(0, 0) = 2.0 / (R - L);
            ortho(0, 3) = -(R + L) / (R - L);
            ortho(1, 1) = -2.0 / (T - B);
            ortho(1, 3) = (T + B) / (T - B);
            ortho(2, 2) = -2.0 / (F - N);
            ortho(2, 3) = -(F + N) / (F - N);
            ortho(3, 3) = 1.0;

            // construct a projection matrix, this is identical to the 
            // projection matrix computed for the intrinsics, except an
            // additional row is inserted to map the z-coordinate to
            // OpenGL. 
            Eigen::Matrix4d tproj = Eigen::Matrix4d::Zero();
            tproj(0, 0) = alpha;
            tproj(0, 1) = skew;
            tproj(0, 2) = -u0;
            tproj(1, 1) = beta;
            tproj(1, 2) = -v0;
//            tproj(2, 2) = -(N + F);
//            tproj(2, 3) = -N*F;
//            tproj(3, 2) = 1.0;
                        tproj(2, 2) = (N + F);
                        tproj(2, 3) = N*F;
                        tproj(3, 2) = -1.0;

            // resulting OpenGL frustum is the product of the orthographic
            // mapping to normalized device coordinates and the augmented
            // camera intrinsic matrix
            frustum = ortho*tproj;
        }
    } /* namespace rgbd */
} /* namespace cv */
