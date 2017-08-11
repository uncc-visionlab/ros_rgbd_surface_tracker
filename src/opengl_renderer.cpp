/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */
#include <string>      /* printf, scanf, puts, NULL */

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include <GL/freeglut.h>
#include <GL/glext.h>

#include <Eigen/Dense>

#include <ros_rgbd_surface_tracker/rgbd_tracker_datatypes.hpp>
#include <ros_rgbd_surface_tracker/opengl_renderer.hpp>
#include <bits/stl_vector.h>

#define OFFSCREEN_W 640
#define OFFSCREEN_H 480

namespace cv {
    namespace rgbd {

        char fname[] = "output.pnm";
        
        void OpenGLRenderer::initFrame(void) {
            //  Clear screen and Z-buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            // get window input events
            //glutMainLoopEvent();
        }

        void OpenGLRenderer::callbackDisplay(void) {
            glPushMatrix();
            for (std::pair<std::string, ObjectGeometry> mapElement : geomList) {
                renderGeometry(mapElement);
            }
            glPopMatrix();
            //glFlush();
            glutSwapBuffers();

            // call on frame postRender()
            frameIndex++;
            // enable to view images when offscreen rendering is being used
            if (false && offscreen && frameIndex % 10 == 0) {
                saveWindow(fname);
            }
            glutMainLoopEvent();
        }

        void OpenGLRenderer::callbackIdle(void) {
            //glutPostRedisplay();
        }

        void OpenGLRenderer::callbackKeyboard(unsigned char key, int x, int y) {
            glMatrixMode(GL_MODELVIEW);
            
            switch (key) {
                case '=':
                    attrs.delta_budget_ms += 2;
                    break;
                case '-':
                    attrs.delta_budget_ms -= 2;
                    break;
                case 'a':
                    glTranslatef(+0.2f, 0.0f, 0.0f);
                    break;
                case 'w':
                    glTranslatef(0.0f, +0.2f, 0.0f);
                    break;
                case 'd':
                    glTranslatef(-0.2f, 0.0f, 0.0f);
                    break;
                case 's':
                    glTranslatef(0.0f, -0.2f, 0.0f);
                    break;
                case 'l':
                    attrs.showWireframe = !attrs.showWireframe;
                    break;
                case 'n':
                    attrs.showPointcloudNormals = !attrs.showPointcloudNormals;
                    break;
                case 'p':
                    attrs.showPointcloud = !attrs.showPointcloud;
                    break;
                case 'm':
                    attrs.showDetections = !attrs.showDetections;
                    break;
                case '\n':
                case '\r':
                    //vscroll();
                case 'R' - '@': /* ^R */
                    break;

                case 'S' - '@': /* ^S */
                    saveWindow(fname);
                    break;

                case '\x08': /* backspace */
                    //                    if (cursor) {
                    //                        --cursor;
                    //                        text[ 0 ][ cursor + 1 ] = 0;
                    //                        text[ 0 ][ cursor ] = '\n';
                    //                    }
                    break;

                case '\x1b':
                    exit(0);
                    break;

                default:
                    //add_char(key);
                    break;
            }
            //glutPostRedisplay();
        }

        void OpenGLRenderer::callbackMouseClick(int button, int state, int x, int y) {
            attrs.specialKey = glutGetModifiers();
            // if both a mouse button, and the ALT key, are pressed then
            // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
            if (state == GLUT_UP) return; // Disregard redundant GLUT_UP (MouseButtonRelease) events            
            if ((state == GLUT_DOWN)) { // &&
                //(attrs.specialKey == GLUT_ACTIVE_SHIFT)) {
                if (button == GLUT_LEFT_BUTTON) {
                    //std::cout << "Got Left Click";
                } else if (button == GLUT_MIDDLE_BUTTON) {
                    //std::cout << "Got Middle Click";
                } else if (button == GLUT_RIGHT_BUTTON) {
                    //std::cout << "Got Right Click";
                } else if (button == 3) { // Mouse wheel Scroll Up
                    //std::cout << "Got Mouse Scroll Up Click";
                    glTranslatef(0.0f, 0.0f, -0.1f);
                } else if (button == 4) { // Mouse wheel Scroll Down
                    //std::cout << "Got Mouse Scroll Down Click";
                    glTranslatef(0.0f, 0.0f, +0.1f);
                }
            }
        }

        void OpenGLRenderer::callbackMouseMotion(int x, int y) {
            // the ALT key was used in the previous function
            if (attrs.specialKey != GLUT_ACTIVE_ALT) {
                // setting red to be relative to the mouse
                // position inside the window
            }
        }

        void OpenGLRenderer::callbackPassiveMouseMotion(int x, int y) {
            // User must press the SHIFT key to change the
            // rotation in the X axis
            if (attrs.specialKey != GLUT_ACTIVE_SHIFT) {
                // setting the angle to be relative to the mouse
                // position inside the window
                //if (x < 0)
                //    angleX = 0.0;
                //else if (x > width)
                //    angleX = 180.0;
                //else
                //    angleX = 180.0 * ((float) x) / height;
            }
        }

        void OpenGLRenderer::callbackReshape(int w, int h) {

        }

        void OpenGLRenderer::writeRawPNM(const char *fname, char *pixels, int w, int h) {
            FILE *f;

            f = fopen(fname, "wb");
            if (!f)
                printf("Ouch!  Cannot create file.\n");
            else {
                int row;

                fprintf(f, "P6\n");
                fprintf(f, "# CREATOR: offscreen freeglut demo\n");
                fprintf(f, "%d %d\n", w, h);
                fprintf(f, "255\n");

                /*
                 * Write the rows in reverse order because OpenGL's 0th row
                 * is at the bottom.
                 */
                for (row = h; row; --row)
                    fwrite(pixels + ((row - 1) * w * 3), 1, 3 * w, f);

                fclose(f);
            }
        }

        void OpenGLRenderer::saveWindow(const char *file_name) {
            std::cout << "Saving frame to file: " << std::string(file_name) << "." << std::endl;
            int width = glutGet(GLUT_WINDOW_WIDTH);
            int height = glutGet(GLUT_WINDOW_HEIGHT);
            char pixels[3 * width * height];
            if (pixels) {
                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glReadPixels(0, 0, width, height,
                        GL_RGB, GL_UNSIGNED_BYTE, (GLvoid *) pixels);
                writeRawPNM(file_name, pixels, width, height);
            }
        }

        int OpenGLRenderer::init(int width, int height, bool _offscreen) {
            int argc = 0;
            imgSeg.create(height, width, CV_8UC3);
            imgSeg = Scalar(0, 255, 0);
            offscreen = _offscreen;

            glutInit(&argc, NULL);
            glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
            glutInitWindowPosition(800, 400);
            glutInitWindowSize(width, height);
            glutCreateWindow("Render Window");

            // initialize offscreen rendering framebuffer
            if (offscreen) {
                glutHideWindow();
                /*  Framebuffer */
                glGenFramebuffers(1, &fbo);
                glBindFramebuffer(GL_FRAMEBUFFER, fbo);

                /* Color renderbuffer. */
                glGenRenderbuffers(1, &color);
                glBindRenderbuffer(GL_RENDERBUFFER, color);
                /* Storage must be one of: */
                /* GL_RGBA4, GL_RGB565, GL_RGB5_A1, GL_DEPTH_COMPONENT16, GL_STENCIL_INDEX8. */
                glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB565, width, height);
                glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color);

                /* Depth render buffer. */
                glGenRenderbuffers(1, &depth);
                glBindRenderbuffer(GL_RENDERBUFFER, depth);
                glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);
                glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
                glReadBuffer(GL_COLOR_ATTACHMENT0);
            } else {
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                glutKeyboardFunc(callbackKeyboard);
                glutMouseFunc(callbackMouseClick);
                glutMotionFunc(callbackMouseMotion);
                glutPassiveMotionFunc(callbackPassiveMouseMotion);
                //glutDisplayFunc(callbackDisplay);
            }
            glutIdleFunc(callbackIdle);
            glutMainLoopEvent();

            //initializating the texture mapping
            GLuint mTexture;
            glGenTextures(1, &mTexture);
            glBindTexture(GL_TEXTURE_2D, mTexture);
            glEnable(GL_DEPTH_TEST); // Enable depth testing for z-culling
            glDepthFunc(GL_LEQUAL); // Set the type of depth-test
            glShadeModel(GL_SMOOTH); // Enable smooth shading

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glColor3f(0.0f, 0.0f, 0.0f);

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
            glMatrixMode(GL_MODELVIEW);
            glLineWidth(3.0);
            return 0;
        }

        void OpenGLRenderer::pushObject(std::string name, ObjectGeometry geom) {
            geomList.insert({name, geom});
        }

        void OpenGLRenderer::renderGeometry(std::pair<std::string, ObjectGeometry> mapElement) {
            ObjectGeometry geom = mapElement.second;
            if (geom.getSurfaceType() != cv::rgbd::EDGE &&
                    geom.getSurfaceType() != cv::rgbd::CORNER) {
                attrs.showWireframe ? glBegin(GL_LINE_LOOP) : glBegin(GL_TRIANGLES);
                for (int idx = 0; idx < geom.verts.size(); ++idx) {
                    cv::Vec3f vert = geom.verts[idx];
                    cv::Vec3f normal = geom.normals[idx];
                    cv::Vec3f color = geom.colors[idx];
                    glColor4f(color[0], color[1], color[2], 0.5f);
                    glNormal3f(normal[0], normal[1], normal[2]);
                    glVertex3f(vert[0], vert[1], vert[2]);
                }
                glEnd();
            } else { // Render EDGE and CORNER Object Geometries
                glBegin(GL_LINES);
                for (int idx = 0; idx < geom.verts.size(); ++idx) {
                    cv::Vec3f vert = geom.verts[idx];
                    cv::Vec3f normal = geom.normals[idx];
                    cv::Vec3f color = geom.colors[idx];
                    glColor3f(color[0], color[1], color[2]);
                    glNormal3f(normal[0], normal[1], normal[2]);
                    glVertex3f(vert[0], vert[1], vert[2]);
                }
                glEnd();
            }
        }

        void OpenGLRenderer::constructGeometries(
                const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                std::vector<ObjectGeometry>& geomVec) const {
            for (auto shapeType_it = query_shapeMap.begin();
                    shapeType_it != query_shapeMap.end(); ++shapeType_it) {
                for (auto shape_iter = (*shapeType_it).second.begin();
                        shape_iter != (*shapeType_it).second.end(); ++shape_iter) {
                    ObjectGeometry geom;
                    geom.setSurfaceType((*shapeType_it).first);
                    geom.setShape(*shape_iter);
                    sg::Shape& shape = **shape_iter;
                    //sg::Shape& shape = *geom.getShape();
                    std::vector<cv::Vec3f> pts = shape.generateCoords();
                    std::vector<int> triIdxList = shape.generateCoordIndices();
                    for (int triIdxs : triIdxList) {
                        geom.verts.push_back(pts[triIdxs]);
                    }
                    std::vector<cv::Vec3f> norms = shape.generateNormals();
                    std::vector<int> triNormalIdxList = shape.generateNormalCoordIndices();
                    for (int triNormalIdxs : triNormalIdxList) {
                        geom.normals.push_back(norms[triNormalIdxs]);
                    }
                    std::vector<cv::Vec3f> colors = shape.generateColorCoords();
                    std::vector<int> triColorIdxList = shape.generateColorCoordIndices();
                    for (int triColorIdxs : triColorIdxList) {
                        geom.colors.push_back(colors[triColorIdxs]);
                    }
                    geomVec.push_back(geom);
                }
            }
        }

        void OpenGLRenderer::renderGeometries(std::vector<ObjectGeometry> geomList) {
            int idx = 0;
            for (ObjectGeometry geom : geomList) {
                //                std::string geomName(std::string(surfaceTypeToString[geom.getSurfaceType()])
                //                        + " " + std::to_string(idx++));
                std::string geomName(std::string("test")
                        + " " + std::to_string(idx++));
                pushObject(geomName, geom);
            }
            callbackDisplay();
        }

        void OpenGLRenderer::renderPointCloudNormals(cv::Mat points, cv::Mat normals,
                float scale, float density, bool outwardPointing) const {

            scale = outwardPointing ? scale : -scale;
            int skip = (int) (1.0f / density);
            glBegin(GL_LINES);
            for (int y = 0; y < points.rows; y += skip) {
                cv::Vec3f *points_ptr = points.ptr<cv::Vec3f>(y, 0);
                cv::Vec3f *normals_ptr = normals.ptr<cv::Vec3f>(y, 0);
                for (int x = 0; x < points.cols; x += skip) {
                    if (!std::isnan((*points_ptr)[0])) {
                        glColor3f(1.0, 0.0, 0.0);
                        glVertex3f((*points_ptr)[0], (*points_ptr)[1], (*points_ptr)[2]);
                        glVertex3f((*points_ptr)[0] + scale * (*normals_ptr)[0],
                                (*points_ptr)[1] + scale * (*normals_ptr)[1],
                                (*points_ptr)[2] + scale * (*normals_ptr)[2]);
                    }
                    points_ptr += skip;
                    normals_ptr += skip;
                }
            }
            glEnd();
            glFlush();
            glutSwapBuffers();
        }

        void OpenGLRenderer::renderPointCloud(cv::Mat points, cv::Mat colors) {
            glPointSize(1.5);

            glBegin(GL_POINTS);
            for (int y = 0; y < points.rows; ++y) {
                cv::Vec3f *points_ptr = points.ptr<cv::Vec3f>(y, 0);
                uchar *colors_ptr = colors.ptr<uchar>(y, 0);
                for (int x = 0; x < points.cols; ++x) {
                    if (!std::isnan((*points_ptr)[0])) {
                        // MAKE POINT CLOUND TRANSPARENT
                        glColor4ub(colors_ptr[2], colors_ptr[1], colors_ptr[0], 80);
                        glVertex3d((*points_ptr)[0], (*points_ptr)[1], (*points_ptr)[2]);
                        //std::cout << "vert " << (*points_ptr) << " color "
                        //        << (int) colors_ptr[0] << "," << (int) colors_ptr[1] 
                        // << ", " << (int) colors_ptr[2] << std::endl;
                    }
                    points_ptr++;
                    colors_ptr += 3;
                }
            }
            glEnd();
            glFlush();

            glutSwapBuffers();

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
            ortho(0, 0) = -2.0 / (R - L);
            ortho(0, 3) = (R + L) / (R - L);
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
