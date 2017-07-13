/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */

//#include <GL/glu.h>
#include <GL/glut.h>

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
                cv::Vec3f color = geom.colors[idx];
                glColor3f(color[0], color[1], color[2]);
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
            static GLfloat xRotated = 0, yRotated = 0, zRotated = 0;
            xRotated += 1;
            zRotated += 2;
            zRotated += 1.5;

            glOrtho(0, 1, 0, 1, -1, 1);
            //glDisable(GL_DEPTH_TEST);
            //glDisable(GL_LIGHTING);
            //glDepthMask(GL_FALSE);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            // clear the drawing buffer.
            //  Clear screen and Z-buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glEnable(GL_TEXTURE_2D);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgSeg.cols, imgSeg.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, imgSeg.data);

            glBegin(GL_POLYGON);
            glTexCoord2f(0.0f, 1.0f);
            glVertex3f(-1.0f, -1.0f, 1.0f);
            glTexCoord2f(1.0f, 1.0f);
            glVertex3f(1.0f, -1.0f, 1.0f);
            glTexCoord2f(1.0f, 0.0f);
            glVertex3f(1.0f, 1.0f, 1.0f);

            glTexCoord2f(0.0f, 1.0f);
            glVertex3f(-1.0f, -1.0f, 1.0f);
            glTexCoord2f(1.0f, 0.0f);
            glVertex3f(1.0f, 1.0f, 1.0f);
            glTexCoord2f(0.0f, 0.0f);
            glVertex3f(-1.0f, 1.0f, 1.0f);
            glEnd();

            glPushMatrix();

            glDisable(GL_TEXTURE_2D);
            //glEnable(GL_LIGHTING);

            glLoadIdentity();

            //glTranslatef(0.0, 0.0, -10.5);
            glRotatef(xRotated, 1.0, 0.0, 0.0);
            // rotation about Y axis
            glRotatef(yRotated, 0.0, 1.0, 0.0);
            // rotation about Z axis
            glRotatef(zRotated, 0.0, 0.0, 1.0);

            glBegin(GL_TRIANGLES);
            for (auto geom : geomList) {
                renderGeometry(geom);
            }
            glEnd();
            glPopMatrix();

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
            //glEnable(GL_LIGHTING);
            //glEnable(GL_LIGHT0);

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
    } /* namespace rgbd */
} /* namespace cv */
