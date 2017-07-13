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

#include <ros_rgbd_surface_tracker/rgbd_tracker_datatypes.hpp>

namespace cv {
    namespace rgbd {

        class OpenGLRenderer {
        private:
            cv::Mat imgSeg;
            std::unordered_map<std::string, ObjectGeometry> geomList;
        public:

            OpenGLRenderer() {
            }

            virtual ~OpenGLRenderer() {
            };

            bool initialized() {
                return !imgSeg.empty();
            }

            int init(int width, int height);
            void draw();
            void reshape(GLsizei width, GLsizei height);
            void renderGeometries(std::vector<cv::rgbd::ObjectGeometry> geomList);

            void renderGeometry(std::pair<std::string, ObjectGeometry> mapElement);
            void pushObject(std::string name, ObjectGeometry geom);

            void setImage(const cv::Mat& image) {
                imgSeg = image;
            }

            void clearObjects() {
                geomList.clear();
            }
            void post();
        }; /* class OpenGLRenderer */

    } /* namespace rgbd */
} /* namespace cv */
#endif /* __cplusplus */

#endif /* OPENGL_RENDERER_HPP */

