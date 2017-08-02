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

#include <ros_rgbd_surface_tracker/opengl_renderer.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_datatypes.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>

//#define PROFILE_CALLGRIND

#ifdef PROFILE_CALLGRIND
#include <valgrind/callgrind.h>
#endif

namespace cv {
    namespace rgbd {

        class SurfaceDetector {
        public:
            void detect(const cv::rgbd::RgbdImage& rgbd_img,
                    cv::QuadTree<sg::Plane<float>::Ptr>& quadTree,
                    int timeBudget_ms, cv::Mat& rgb_result,
                    const cv::Mat mask = cv::Mat()) const;

            std::vector<cv::Point2i> findPlanes(ErrorSortedRectQueue& quadQueue,
                    const RgbdImage& rgbd_img,
                    QuadTreeLevel<sg::Plane<float>::Ptr>* quadTree,
                    Mat& img_labels, int& numLabels) const;

            std::vector<cv::Point2i> recursiveSubdivision(ErrorSortedRectQueue& quadQueue,
                    const RgbdImage& rgbd_img,
                    QuadTreeLevel<sg::Plane<float>::Ptr>* quadTree,
                    const std::vector<cv::Point2i>& subdivideTileVec,
                    Mat& img_labels, int& numLabels) const;

        }; /* class SurfaceDetector */

        class SurfaceDescriptorExtractor {
        public:
            void compute(const cv::rgbd::RgbdImage& rgbd_img,
                    cv::QuadTree<sg::Plane<float>::Ptr>* quadTree,
                    std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                    int timeBudget_ms, cv::Mat& rgb_result) const;
            
            void cluster(
                std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap);
            
        }; /* class SurfaceDescriptorExtractor */

        class SurfaceDescriptorMatcher {
        public:
            void match(std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                    std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&train_shapeMap,
                    std::vector<cv::DMatch>& matches, int timeBudget_ms,
                    cv::Mat& rgb_result, Pose camPose = Pose(), cv::Mat mask = cv::Mat());
        }; /* class SurfaceDescriptorMatcher */

        class RgbdSurfaceTracker {
        public:
            static OpenGLRenderer glDraw;
            typedef boost::shared_ptr<RgbdSurfaceTracker> Ptr;

            RgbdSurfaceTracker() {
            }

            virtual ~RgbdSurfaceTracker() {
            };

            // validate edge and corner features with geometric measurements center on feature location
            void setRealorVirtualFlag(cv::rgbd::RgbdImage& rgbd_img,
                    cv::QuadTree<sg::Plane<float>::Ptr>& quadTree,
                    std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&query_shapeMap,
                    cv::Mat& rgb_result);

            // erase edge and corner features that do not lie within the image boundary
            bool setIsRealFlag(const sg::Shape::Ptr shape,
                const cv::rgbd::RgbdImage& rgbd_img, const cv::Size& tileDims,
                const cv::rgbd::SurfaceType& shapeType, cv::Mat& rgb_result) const;

            void clusterDetections(std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>& query_shapeMap,
                    cv::Mat& rgb_result);

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

