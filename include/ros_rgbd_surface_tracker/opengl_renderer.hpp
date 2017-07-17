/* 
 * File:   opengl_renderer.hpp
 * Author: arwillis
 *
 * Created on July 13, 2017, 1:20 PM
 */

#ifndef OPENGL_RENDERER_HPP
#define OPENGL_RENDERER_HPP

#ifdef __cplusplus

#include <string>
#include <unordered_map>

#include <GL/gl.h>

#include <opencv2/core.hpp>

//#include <ros_rgbd_surface_tracker/opengl_iothread.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_datatypes.hpp>

namespace cv {
    namespace rgbd {

        class OpenGLRenderer {
        private:
            cv::Mat imgSeg;
            std::unordered_map<std::string, ObjectGeometry> geomList;           
            GLuint color;
            GLuint depth;
            GLuint fbo;
        public:
            static int specialKey;

            OpenGLRenderer() {
            }

            virtual ~OpenGLRenderer() {
            };

            bool initialized() {
                return !imgSeg.empty();
            }

            int init(int width, int height, bool offscreen = false);

            static void callbackIdle(void);

            static void callbackKeyboard(unsigned char key, int x, int y);

            static void callbackMouseClick(int button, int state, int x, int y);
            
            static void callbackMouseMotion(int x, int y);

            static void callbackPassiveMouseMotion(int x, int y);

            static void callbackReshape(int w, int h);

            void callbackDisplay(void);

            static void saveWindow(const char *file_name);

            static void writeRawPNM(const char *fname, char *pixels, int w, int h);

            void reshape(GLsizei width, GLsizei height);

            void renderGeometries(std::vector<ObjectGeometry> geomList);

            void renderGeometry(std::pair<std::string, ObjectGeometry> mapElement);

            void renderPointCloud(cv::Mat points, cv::Mat colors);

            void pushObject(std::string name, ObjectGeometry geom);

            void setImage(const cv::Mat& image) {
                imgSeg = image;
            }

            void clearObjects() {
                geomList.clear();
            }

            void ros2opengl(cv::Mat& rotMat);

            void build_opengl_projection_for_intrinsics(Eigen::Matrix4d &frustum, int *viewport,
                    double alpha, double beta, double skew, double u0, double v0,
                    int img_width, int img_height, double near_clip, double far_clip);
        }; /* class OpenGLRenderer */

    } /* namespace rgbd */
} /* namespace cv */
#endif /* __cplusplus */

#endif /* OPENGL_RENDERER_HPP */

