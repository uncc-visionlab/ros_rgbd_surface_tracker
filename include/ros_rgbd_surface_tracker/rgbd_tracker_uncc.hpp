/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rgbd_uncc_contrib.h
 * Author: arwillis
 *
 * Created on June 30, 2017, 10:01 AM
 */

#ifndef RGBD_UNCC_CONTRIB_HPP
#define RGBD_UNCC_CONTRIB_HPP

#ifdef __cplusplus

#include <limits>
#include <string>
#include <unordered_map>

#include <boost/shared_ptr.hpp>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>

#define PROFILE_CALLGRIND true

#ifdef PROFILE_CALLGRIND
#include <valgrind/callgrind.h>
#endif

namespace cv {
    namespace rgbd {

        class AlgebraicSurfacePatch {
        public:
            std::vector<cv::Vec3f> verts;
        }; /* class AlgebraicSurfacePatch */

        class ObjectGeometry {
        public:
            std::vector<cv::Vec3f> verts;
        }; /* class ObjectGeometry */

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

        class SurfaceDetector {
        public:
            void detect(const cv::rgbd::RgbdImage& rgbd_img,
                    std::vector<AlgebraicSurfacePatch>& surflets,
                    const cv::Mat mask = cv::Mat()) const;
        }; /* class SurfaceDetector */

        class SurfaceDescriptorExtractor {
        public:
            void compute(const cv::rgbd::RgbdImage& rgbd_img,
                    std::vector<AlgebraicSurfacePatch>& surflets,
                    std::vector<ObjectGeometry>& geometries) const;
        }; /* class SurfaceDescriptorExtractor */

        class SurfaceDescriptorMatcher {
        public:
            void match(std::vector<ObjectGeometry> queryDescriptors,
                    std::vector<ObjectGeometry> trainDescriptors,
                    std::vector<cv::DMatch>& matches, cv::Mat mask = cv::Mat());
        }; /* class SurfaceDescriptorMatcher */

        class RgbdSurfaceTracker {
        public:
            static OpenGLRenderer glDraw;
            typedef boost::shared_ptr<RgbdSurfaceTracker> Ptr;

            RgbdSurfaceTracker() {
            }

            virtual ~RgbdSurfaceTracker() {
            };

            void segmentDepth(cv::rgbd::RgbdImage& rgbd_img, cv::Mat& result);

            void callback(cv::Mat& _ocv_rgbframe, cv::Mat& _ocv_depthframe_float,
                    cv::Mat& _rgb_distortionCoeffs, cv::Mat& _rgb_cameraMatrix);

            PlaneVisualizationData *getPlaneVisualizationData() {
                return &vis_data;
            }

            void iterativeAlignment(cv::rgbd::RgbdImage& rgbd_img, cv::Mat& rgb_result);
        private:
            // -------------------------
            // Disabling default copy constructor and default
            // assignment operator.
            // -------------------------
            RgbdSurfaceTracker(const RgbdSurfaceTracker& ref);
            RgbdSurfaceTracker& operator=(const RgbdSurfaceTracker& ref);
            PlaneVisualizationData vis_data;
            SurfaceDetector surfdetector;
            SurfaceDescriptorExtractor surfdescriptor_extractor;
            SurfaceDescriptorMatcher surfmatcher;
        }; /* class RgbdSurfaceTracker */
    } /* namespace rgbd */
} /* namespace cv */

#endif /* __cplusplus */
#endif /* RGBD_UNCC_CONTRIB_H */

