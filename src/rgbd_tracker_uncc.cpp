/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>      /* printf, scanf, puts, NULL */
#include <unordered_set>

#include <boost/bind.hpp>

#include <GL/gl.h>
//#include <GL/glu.h>
#include <GL/glut.h>

#include <opencv2/highgui.hpp>
#include <opencv2/rgbd.hpp>

#include <ros_rgbd_surface_tracker/AlgebraicSurface.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

extern int supermain(AlgebraicSurface<float>& surf, PlaneVisualizationData& vis_data,
        const Eigen::Matrix<float, 8, 3>& cube, float cubesize, float levelset);

#define BLOCKSIZE 48
#define MARGIN_X 50
#define MARGIN_Y 30

namespace cv {
    namespace rgbd {

        // construct static class member for OpenGL based rendering of scene objects
        OpenGLRenderer RgbdSurfaceTracker::glDraw;
        OpenGLRenderAttributes OpenGLRenderer::attrs;

        std::map<SurfaceType, const char*> surfaceTypeToString = {
            {SurfaceType::UNKNOWN, "Uknown"},
            {SurfaceType::PLANE, "Plane"},
            {SurfaceType::EDGE, "Edge"},
            {SurfaceType::CORNER, "Corner"},
            {SurfaceType::BOX, "Box"}
        };

        std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>> train_shapeMap;

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

        void testHypotheses(cv::rgbd::RgbdImage& rgbd_img,
                cv::QuadTree<sg::Plane<float>::Ptr>& quadtree,
                cv::Rect2i& roi, std::vector<cv::rgbd::ObjectGeometry>& geometry_list) {

            for (cv::rgbd::ObjectGeometry& geometry : geometry_list) {
                switch (geometry.getSurfaceType()) {
                    case SurfaceType::EDGE:
                        sg::Edge<float>::Ptr edge = boost::static_pointer_cast<sg::Edge<float>>(geometry.getShape());
                        cv::Point3f midpoint_edge = 0.5 * (edge->getPoint(edge->start) + edge->getPoint(edge->end));
                        cv::Point2f img_midpoint_edge = rgbd_img.project(midpoint_edge);
                        std::cout << "img_midpoint: " << img_midpoint_edge << std::endl;
                        //getBlockOfPoint(img_midpoint_edge, quadtree, roi);
                        break;
                }
            }
        }

        bool RgbdSurfaceTracker::setIsRealFlag(const sg::Shape::Ptr shape,
                const cv::rgbd::RgbdImage& rgbd_img, const cv::Size& tileDims,
                const cv::rgbd::SurfaceType& shapeType) const {
            cv::Vec3f position;
            shape->getPose().getTranslation(position);
            cv::Point2f projPt = rgbd_img.project(position);
            cv::Rect tile((int) projPt.x - (tileDims.width >> 1),
                    (int) projPt.y - (tileDims.height >> 1),
                    tileDims.width, tileDims.height);
            if (tile.x < 0 || tile.y < 0 ||
                    tile.x + tile.width >= rgbd_img.getWidth() ||
                    tile.y + tile.height >= rgbd_img.getHeight()) {
                //std::cout << "Feature is too close to image boundary to analyze... deleting!" << std::endl;
                //std::cout << "tile = " << tile << std::endl;
                // delete -> feature position at the periphery of image 
                // (tile data exceeds image dimensions)
                return true;
            }
            int numSamples = (tile.width * tile.height) / 2;
            std::vector<cv::Point3f> tile_data;
            tile_data.reserve(numSamples);
            rgbd_img.getTileData_Uniform(tile.x, tile.y,
                    tile.width, tile.height, tile_data, numSamples);
            if (tile_data.size() > (numSamples >> 2)) {
                sg::Corner<float>::Ptr cornerPtr;
                sg::Edge<float>::Ptr edgePtr;
                switch (shapeType) {
                    case SurfaceType::CORNER:
                        cornerPtr = boost::static_pointer_cast<sg::Corner<float>>(shape);
                        cornerPtr->setIsRealFlag(tile_data);
                        break;
                    case SurfaceType::EDGE:
                        edgePtr = boost::static_pointer_cast<sg::Edge<float>>(shape);
                        edgePtr->setIsRealFlag(tile_data);
                        break;
                    default:
                        std::cout << "RgbdSurfaceTracker::setIsRealFlag() -> should never execute!" << std::endl;
                        break;
                }
            } else {
                // delete -> not enough data to confirm edge / corner structure
                return true;
            }
            return false; // keep this shape and we have set it's isReal() flag
        }

        void RgbdSurfaceTracker::setRealorVirtualFlag(cv::rgbd::RgbdImage& rgbd_img,
                cv::QuadTree<sg::Plane<float>::Ptr>& quadTree,
                std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&query_shapeMap, cv::Mat & rgb_result) {
            //sg::Corner<float>::Ptr cornerPtr;
            //sg::Edge<float>::Ptr edgePtr;
            //cv::Vec3f position;

            cv::Size tileDims(BLOCKSIZE, BLOCKSIZE);

            for (auto shapeType_iter = query_shapeMap.begin();
                    shapeType_iter != query_shapeMap.end(); ++shapeType_iter) {
                if ((*shapeType_iter).first == SurfaceType::EDGE ||
                        (*shapeType_iter).first == SurfaceType::CORNER) {
                    std::vector<sg::Shape::Ptr>& shapeVec = (*shapeType_iter).second;
                    cv::rgbd::SurfaceType shapeType = (*shapeType_iter).first;

                    // erase features too close to image boundary for analysis
                    shapeVec.erase(std::remove_if(shapeVec.begin(), shapeVec.end(),
                            boost::bind(&RgbdSurfaceTracker::setIsRealFlag, this, _1,
                            boost::ref(rgbd_img), boost::ref(tileDims), boost::ref(shapeType))),
                            shapeVec.end());

//                    for (auto shape_iter = shapeVec.begin();
//                            shape_iter != shapeVec.end(); ++shape_iter) {
//                        sg::Shape::Ptr& shape = (*shape_iter);
//                        shape->getPose().getTranslation(position);
//                        cv::Point2f projPt = rgbd_img.project(position);
//                        cv::Rect tile((int) projPt.x - (tileDims.width >> 1),
//                                (int) projPt.y - (tileDims.height >> 1),
//                                tileDims.width, tileDims.height);
//                        int numSamples = (tile.width * tile.height) / 2;
//                        std::vector<Point3f> tile_data;
//                        tile_data.reserve(numSamples);
//                        rgbd_img.getTileData_Uniform(tile.x, tile.y,
//                                tile.width, tile.height, tile_data, numSamples);
//
//                        if (tile_data.size() > (numSamples >> 2)) {
//                            switch (shapeType) {
//                                case SurfaceType::CORNER:
//                                    cornerPtr = boost::static_pointer_cast<sg::Corner<float>>(*shape_iter);
//                                    cornerPtr->setIsRealFlag(tile_data);
//                                    break;
//                                case SurfaceType::EDGE:
//                                    edgePtr = boost::static_pointer_cast<sg::Edge<float>>(*shape_iter);
//                                    edgePtr->setIsRealFlag(tile_data);
//                                    break;
//                                default:
//                                    break;
//                            }
//                        }
//                    }
                }
            }
        }

        void RgbdSurfaceTracker::clusterDetections(std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&query_shapeMap,
                cv::Mat & rgb_result) {
            std::vector< std::vector < sg::Plane<float>::Ptr>> planeClusters;
            std::vector< std::vector < sg::Edge<float>::Ptr>> edgeClusters;
            std::vector< std::vector < sg::Corner<float>::Ptr>> cornerClusters;
            std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>> clustered_query_shapeMap;
            sg::Corner<float>::Ptr cornerPtr;
            sg::Edge<float>::Ptr edgePtr;
            sg::Plane<float>::Ptr planePtr;
            cv::Vec3f position;
            bool done = false;

            for (auto shapeType_iter = query_shapeMap.begin();
                    shapeType_iter != query_shapeMap.end(); ++shapeType_iter) {
                for (auto shape_iter = (*shapeType_iter).second.begin();
                        shape_iter != (*shapeType_iter).second.end(); ++shape_iter) {
                    switch ((*shapeType_iter).first) {
                        case SurfaceType::CORNER:
                            cornerPtr = boost::static_pointer_cast<sg::Corner<float>>(*shape_iter);
                            break;
                        case SurfaceType::EDGE:
                            edgePtr = boost::static_pointer_cast<sg::Edge<float>>(*shape_iter);
                            edgePtr->getPose().getTranslation(position);
                            break;
                        case SurfaceType::PLANE:
                        default:
                            planePtr = boost::static_pointer_cast<sg::Plane<float>>(*shape_iter);
                            planePtr->getPose().getTranslation(position);
                            break;
                    }
                }
            }
        }

        void RgbdSurfaceTracker::segmentDepth(cv::rgbd::RgbdImage& rgbd_img, cv::Mat & rgb_result) {

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif
            bool offscreen_rendering = false;

            if (!glDraw.initialized()) {
                glDraw.init(rgbd_img.getWidth(), rgbd_img.getHeight(), offscreen_rendering);
            }

            cv::Rect rectVal(290, 200, 640 - 2 * 290, 480 - 2 * 200);
            cv::rectangle(rgb_result, rectVal, cv::Scalar(0, 255, 0), 3);
            //rgbd_img.computeNormals();

            cv::Rect roi(MARGIN_X, MARGIN_Y, rgbd_img.getWidth() - 2 * MARGIN_X, rgbd_img.getHeight() - 2 * MARGIN_Y);
            cv::Size imgSize(rgbd_img.getWidth(), rgbd_img.getHeight());
            cv::Size tileSize(BLOCKSIZE, BLOCKSIZE);
            cv::QuadTree<sg::Plane<float>::Ptr> quadTree(imgSize, tileSize, roi);

            int detector_timeBudget_ms = 15;
            surfdetector.detect(rgbd_img, quadTree, detector_timeBudget_ms, rgb_result);

            int descriptor_timeBudget_ms = 20;
            std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>> query_shapeMap;
            surfdescriptor_extractor.compute(rgbd_img, &quadTree, query_shapeMap,
                    descriptor_timeBudget_ms, rgb_result);

            // Validate higher-order detections:
            // We want to discriminate between virtual/real corner detections and virtual/real edge detections
            // TODO: Implement setRealorVirtualFlag(quadTree, query_shapeMap)
            // This function will visit edges and corners:
            // For each edge:
            // 1. Sample data from the depth image tile centered on the edge midpoint
            // 2. Plug tile data into the quadric surface for the edge to estimate surface fit error
            //    (plug into both surfaces and take min(abs(f1(pt)),abs(f2(pt))))
            // 3. If points violate initial plane fitting criteria set the edge as virtual
            // For each corner:
            // 1. Sample data from the depth image tile centered on the edge midpoint
            // 2. Plug tile data into the cubic surface for the corner to estimate surface fit error
            //    (plug into the three surfaces and take min(abs(f1(pt)),abs(f2(pt),abs(f3(pt))))
            // 3. If points violate initial plane fitting criteria set the the corner as virtual
            //
            // We may want to keep virtual corners as they may be useful for odometry / loop closure
            setRealorVirtualFlag(rgbd_img, quadTree, query_shapeMap, rgb_result);            
            
            // RENDER DETECTED SHAPE GRAMMAR GEOMETRIES
            std::vector<ObjectGeometry> detected_geometries;
            glDraw.constructGeometries(query_shapeMap, detected_geometries);
            // OpenGL rendering
            //glDraw.setImage(rgbd_img.getRGB());
            glDraw.setImage(rgb_result);
            glDraw.initFrame();
            if (glDraw.attrs.showPointcloud || glDraw.attrs.showPointcloudNormals) {
                cv::Mat points, colors;
                rgbd_img.getPointCloud(points, colors);
                if (glDraw.attrs.showPointcloud) {
                    glDraw.renderPointCloud(points, colors);
                }
                if (glDraw.attrs.showPointcloudNormals) {
                    //glDraw.renderPointCloudNormals(points, rgbd_img.getNormals(),
                    //        0.2, 0.33, false);
                    glDraw.renderPointCloudNormals(points, rgbd_img.getNormals());
                }
            }
            if (glDraw.attrs.showDetections) {
                glDraw.renderGeometries(detected_geometries);
            }
            glDraw.clearObjects();

            // Cluster detections:
            // Merge multiple detections of the same structure
            // TODO: Implement clusterDetections(query_shapeMap)
            // This function visits all features for matching, e.g., planes, edges and corners,
            // and clusters the detections into groups and produces a smaller collection of
            // features for integration with the "map"
            clusterDetections(query_shapeMap, rgb_result);

            // Match structures detected in this image with structures of the map
            // Compute the map between detected structures and existing structures
            // TODO: Implement match(query_shapeMap)
            // This function visits all features for matching, e.g., planes, edges and corners,
            // and clusters the detections into groups and produces a smaller collection of
            // features for integration with the "map"
            int descriptor_matching_timeBudget_ms = 20;
            std::vector<cv::DMatch> geometricMatches;
            std::vector<int> noMatches;
            surfmatcher.match(query_shapeMap, train_shapeMap, geometricMatches,
                    descriptor_matching_timeBudget_ms, rgb_result);

            // Structures without a match are inserted to the map
            // addToMap(unmatched_
            // Structures having a match are used to solve the following problems:
            // 1. Localization -> (Range-based Odometry / Pose-change estimation)
            // TODO: computePoseChange();
            // 2. Mapping -> Structure estimation
            // TODO: updateMap(query_shapeMap, train_shapeMap);

            //iterativeAlignment(rgbd_img, rgb_result);

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      

        }
    } /* namespace rgbd */
} /* namespace cv */
