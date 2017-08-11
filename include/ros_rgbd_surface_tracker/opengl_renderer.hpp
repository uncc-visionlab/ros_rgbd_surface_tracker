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

        class OpenGLRenderAttributes {
        public:
            bool showWireframe;
            bool showPointcloud;
            bool showPointcloudNormals;
            bool showDetections;
            int specialKey;
            int delta_budget_ms;

            OpenGLRenderAttributes() : showWireframe(false),
            showPointcloud(true),
            showPointcloudNormals(false),
            showDetections(true), delta_budget_ms(0) {
            }

        };

        class OpenGLRenderer {
        private:
            // texture map RGB image
            cv::Mat imgSeg;

            // list of geometries to render
            std::unordered_map<std::string, ObjectGeometry> geomList;

            bool offscreen;
            int frameIndex;
            // buffer references for offscreen rendering
            GLuint color;
            GLuint depth;
            GLuint fbo;
        public:
            static OpenGLRenderAttributes attrs;

            OpenGLRenderer() : offscreen(false), frameIndex(0) {
            }

            virtual ~OpenGLRenderer() {
            };

            bool initialized() {
                return !imgSeg.empty();
            }

            int init(int width, int height, bool offscreen = false);

            void initFrame(void);

            void setAttributes(OpenGLRenderAttributes& _attrs) {
                attrs = _attrs;
            }

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

            void constructGeometries(
                    const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                    std::vector<ObjectGeometry>& geomVec) const;

            void renderGeometries(std::vector<ObjectGeometry> geomList);

            void renderGeometry(std::pair<std::string, ObjectGeometry> mapElement);

            void renderPointCloud(cv::Mat points, cv::Mat colors);

            void renderPointCloudNormals(cv::Mat points, cv::Mat normals,
                    float scale = 0.01f, float density = 0.33f,
                    bool outwardPointing = true) const;

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

