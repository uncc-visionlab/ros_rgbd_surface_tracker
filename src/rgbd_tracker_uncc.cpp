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
#include <clustering/dkm.hpp> // simple k-means clustering

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

        bool RgbdSurfaceTracker::filterShape(const sg::Shape::Ptr shape,
                const cv::rgbd::RgbdImage& rgbd_img,
                const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                const cv::Size& tileDims,
                const cv::rgbd::SurfaceType& shapeType, cv::Mat& rgb_result) const {
//            static float ISREAL_THRESHOLD = 0.022;
            static float ISREAL_THRESHOLD = 0.01;

            cv::Vec3f position;
            shape->getPose().getTranslation(position);
            cv::Vec2f projPt = rgbd_img.project(position);
            cv::Rect tile((int) projPt[0] - (tileDims.width >> 1),
                    (int) projPt[1] - (tileDims.height >> 1),
                    tileDims.width, tileDims.height);
            if (tile.x < 0 || tile.y < 0 ||
                    tile.x + tile.width >= rgbd_img.getWidth() ||
                    tile.y + tile.height >= rgbd_img.getHeight()) {
                //std::cout << "Feature is too close to image boundary to analyze... deleting!" << std::endl;
                //std::cout << "tile = " << tile << std::endl;
                // delete -> feature position at the periphery of image 
                // (tile data exceeds image dimensions)
                // erase features too close to image boundary for analysis
                return true;
            }
            //            int numSamples = (tile.width * tile.height) / 2;
            //            std::vector<cv::Point3f> tile_data;
            //            tile_data.reserve(numSamples);
            //            rgbd_img.getTileData_Uniform(tile.x, tile.y,
            //                    tile.width, tile.height, tile_data, numSamples);
            //            if (tile_data.size() > (numSamples >> 2)) {
            cv::Matx33f coordVecs;
            sg::Corner<float>::Ptr cornerPtr;
            sg::Edge<float>::Ptr edgePtr;

            cv::Rect testTile;
            int idx;
            std::unordered_set<sg::Plane<float>::Ptr> planeSet;
            cv::Vec3f axes2[3];
            cv::Vec3f planePos3[3];
            cv::Vec2f planePos2[3];
            cv::Vec2f dirVec[3];
            cv::Vec2f tileCtr[3];
            cv::Vec2f avgTileCtr;
            cv::Point3f ctrpt;
            float error, planeerror;
            int samples_per_tile = 100;
            std::vector<cv::Point3f> tile_data;
            tile_data.reserve(samples_per_tile);

            cv::rectangle(rgb_result, tile, cv::Scalar(0, 255, 0), 1);

            switch (shapeType) {
                case SurfaceType::CORNER:
                    cornerPtr = boost::static_pointer_cast<sg::Corner<float>>(shape);
                    cornerPtr->getPlanes(planeSet);
                    if (false) { // use vector to plane positions directly
                        idx = 0;
                        for (auto plane_iter = planeSet.begin(); plane_iter != planeSet.end(); ++plane_iter, ++idx) {
                            (*plane_iter)->getPose().getTranslation(planePos3[idx]);
                            planePos2[idx] = rgbd_img.project(planePos3[idx]);
                            dirVec[idx] = planePos2[idx] - projPt;
                            dirVec[idx] *= 1.0 / std::sqrt(dirVec[idx].dot(dirVec[idx]));
                            tileCtr[idx][0] = projPt[0] + 0.4 * dirVec[idx][0] * tileDims.width;
                            tileCtr[idx][1] = projPt[1] + 0.4 * dirVec[idx][1] * tileDims.height;
                        }
                    } else { // use local approximately perpendicular coordinate system
                        coordVecs = cornerPtr->getNonOrthogonalCoordinateSystem();
                        //std::cout << "coordVecs = " << coordVecs << std::endl;
                        for (idx = 0; idx < 3; ++idx) {
                            axes2[idx][0] = coordVecs(idx, 0);
                            axes2[idx][1] = coordVecs(idx, 1);
                            axes2[idx][2] = coordVecs(idx, 2);
                        }
                        for (idx = 0; idx < 3; ++idx) {
                            planePos3[idx] = position + 0.05 * axes2[idx] + 0.05 * axes2[(idx + 1) % 3];
                            planePos2[idx] = rgbd_img.project(planePos3[idx]);
                            dirVec[idx] = planePos2[idx] - projPt;
                            dirVec[idx] *= 1.0 / std::sqrt(dirVec[idx].dot(dirVec[idx]));
                            tileCtr[idx][0] = projPt[0] + 0.4 * dirVec[idx][0] * tileDims.width;
                            tileCtr[idx][1] = projPt[1] + 0.4 * dirVec[idx][1] * tileDims.height;
                        }
                    }
                    avgTileCtr = (tileCtr[0] + tileCtr[1] + tileCtr[2]) / 3.0;
                    avgTileCtr -= projPt;
                    error = 0;
                    for (idx = 0; idx < 3; ++idx) {
                        testTile.x = tileCtr[idx][0] - avgTileCtr[0] - 0.10 * std::abs(dirVec[idx][0] * tileDims.width);
                        testTile.y = tileCtr[idx][1] - avgTileCtr[1] - 0.10 * std::abs(dirVec[idx][1] * tileDims.height);
                        testTile.width = 0.2 * tileDims.width;
                        testTile.height = 0.2 * tileDims.height;

                        cv::rectangle(rgb_result, testTile, cv::Scalar(255, 255, 0), 1);

                        // TODO: get classification from quadTree and apply
                        // code A) 
                        // sg::Plane<float>::Ptr fitPlane = quadTree->getObject(tileCtr[idx][0] - avgTileCtr[0], tileCtr[idx][1] - avgTileCtr[1]);
                        // if (fitPlane && !fitPlane.approxEquals(plane[(idx + 2) % 3]) {
                        // cornerPtr->setReal(false);
                        // return true;
                        // } else {
                        
                        tile_data.clear();
                        rgbd_img.getTileData_Uniform(testTile.x, testTile.y, testTile.width, testTile.height, tile_data, samples_per_tile);
                        
                        if (tile_data.size() > 0.75*samples_per_tile) {
                            
                            float last_plane_error = std::numeric_limits<float>::infinity();
                            
                            for (const sg::Plane<float>::Ptr& plane : planeSet) {
                                
                                planeerror = 0;
                                for (const cv::Point3f& pt : tile_data) {
                                    planeerror += std::pow<float>(plane->evaluate(pt), 2.0);
                                }
                                
                                planeerror = std::min(planeerror, last_plane_error);
                                last_plane_error = planeerror;
                            }
                            
                        } else {
                            cornerPtr->setReal(false);
                            cv::rectangle(rgb_result, tile, (cornerPtr->isReal()) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255), 1);
                            return true; // delete
                        }
                        
//                        rgbd_img.getPoint3f(tileCtr[idx][0] - avgTileCtr[0], tileCtr[idx][1] - avgTileCtr[1], ctrpt);
//                        if (std::isnan(ctrpt.z)) {
//                            cornerPtr->setReal(false);
//                            return true; // delete
//                            //return false; // keep
//                        }
//                        ctrpt = (cv::Vec3f) ctrpt - position;
//                        // out of plane error --> plane index is (idx+2)%3
//                        planeerror = std::abs(ctrpt.dot(axes2[(idx + 2) % 3]));
                        //std::cout << "plane error = " << planeerror << std::endl;
                        error = std::max(planeerror, error);
                    }
                    // code A)
                    // cornerPtr->setReal(true);
                    // return false;
                    //std::cout << "error = " << error << std::endl;
                    cornerPtr->setReal(error < ISREAL_THRESHOLD);
                    //return false;                        
                    return !cornerPtr->isReal();
                    break;
                    
                    
                case SurfaceType::EDGE:
                    edgePtr = boost::static_pointer_cast<sg::Edge<float>>(shape);
                    edgePtr->getPlanes(planeSet);
                    if (false) { // use vector to plane positions directly
                        idx = 0;
                        for (auto plane_iter = planeSet.begin(); plane_iter != planeSet.end(); ++plane_iter, ++idx) {
                            (*plane_iter)->getPose().getTranslation(planePos3[idx]);
                            planePos2[idx] = rgbd_img.project(planePos3[idx]);
                            dirVec[idx] = planePos2[idx] - projPt;
                            dirVec[idx] *= 1.0 / std::sqrt(dirVec[idx].dot(dirVec[idx]));
                            tileCtr[idx][0] = projPt[0] + 0.33 * dirVec[idx][0] * tileDims.width;
                            tileCtr[idx][1] = projPt[1] + 0.33 * dirVec[idx][1] * tileDims.height;
                        }
                    } else { // use local approximately perpendicular coordinate system
                        coordVecs = edgePtr->getNonOrthogonalCoordinateSystem();
                        //std::cout << "coordVecs = " << coordVecs << std::endl;
                        for (idx = 0; idx < 2; ++idx) {
                            axes2[idx][0] = coordVecs(idx, 0);
                            axes2[idx][1] = coordVecs(idx, 1);
                            axes2[idx][2] = coordVecs(idx, 2);
                            planePos3[idx] = position + 0.05 * axes2[idx];
                            planePos2[idx] = rgbd_img.project(planePos3[idx]);
                            dirVec[idx] = planePos2[idx] - projPt;
                            dirVec[idx] *= 1.0 / std::sqrt(dirVec[idx].dot(dirVec[idx]));
                            tileCtr[idx][0] = projPt[0] + 0.33 * dirVec[idx][0] * tileDims.width;
                            tileCtr[idx][1] = projPt[1] + 0.33 * dirVec[idx][1] * tileDims.height;
                        }
                    }
                    avgTileCtr = 0.5 * (tileCtr[0] + tileCtr[1]);
                    avgTileCtr -= projPt;
                    
                    error = 0;
                    for (idx = 0; idx < 2; ++idx) {
                        testTile.x = tileCtr[idx][0] - avgTileCtr[0] - 0.15 * std::abs(dirVec[idx][0] * tileDims.width);
                        testTile.y = tileCtr[idx][1] - avgTileCtr[1] - 0.15 * std::abs(dirVec[idx][1] * tileDims.height);
                        testTile.width = 0.3 * tileDims.width;
                        testTile.height = 0.3 * tileDims.height;

                        cv::rectangle(rgb_result, testTile,
                                (idx == 0) ? cv::Scalar(255, 255, 0) : cv::Scalar(0, 255, 255), 1);

                        // TODO: get classification from quadTree and apply
                        // code A) 
                        // sg::Plane<float>::Ptr fitPlane = quadTree->getObject(tileCtr[idx][0] - avgTileCtr[0], tileCtr[idx][1] - avgTileCtr[1]);
                        // if (fitplane && !fitPlane.approxEquals(plane[(idx + 1) % 2]) {
                        // edgePtr->setReal(false);
                        // return true;
                        // } else {
                        
                        tile_data.clear();
                        rgbd_img.getTileData_Uniform(testTile.x, testTile.y, testTile.width, testTile.height, tile_data, samples_per_tile);
                        
                        if (tile_data.size() > 0.75*samples_per_tile) {
                            
                            float last_plane_error = std::numeric_limits<float>::infinity();
                            
                            for (const sg::Plane<float>::Ptr& plane : planeSet) {
                                
                                planeerror = 0;
                                for (const cv::Point3f& pt : tile_data) {
                                    planeerror += std::pow<float>(plane->evaluate(pt), 2.0);
                                }
                                
                                planeerror = std::min(planeerror, last_plane_error);
                                last_plane_error = planeerror;
                            }
                            
                        } else {
                            edgePtr->setReal(false);
                            cv::rectangle(rgb_result, tile, (edgePtr->isReal()) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255), 1);
                            return true; // delete
                        }
                        
//                        rgbd_img.getPoint3f(tileCtr[idx][0] - avgTileCtr[0], tileCtr[idx][1] - avgTileCtr[1], ctrpt);
//                        if (std::isnan(ctrpt.z)) {
//                            edgePtr->setReal(false);
//                            return true; // delete
//                            //return false; // keep
//                        }
//                        ctrpt = (cv::Vec3f) ctrpt - position;
//                        // out of plane error --> plane index is (idx+1)%2
//                        planeerror = std::abs(ctrpt.dot(axes2[(idx + 1) % 2]));

//                        std::string rect_txt = "err=" + std::to_string(planeerror);
//                        cv::putText(rgb_result, rect_txt, testTile.tl() + cv::Point2i(0, -5), 0, .3, Scalar::all(255), 1, 8);
                        error = std::max(planeerror, error);
                    }
//                    cv::putText(rgb_result, std::string("err=" + std::to_string(planeerror)),
//                            tile.tl() + cv::Point2i(0, tile.height + 5), 0, .3, Scalar::all(255), 1, 8);
                    edgePtr->setReal(error < ISREAL_THRESHOLD);
                    cv::rectangle(rgb_result, tile, (edgePtr->isReal()) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255), 1);
                    return !edgePtr->isReal();
                    break;
                    
                    
                default:
                    std::cout << "RgbdSurfaceTracker::filterShape() -> should never execute!" << std::endl;
                    break;
            }
            //            } else {
            //                // delete -> not enough data to confirm edge / corner structure
            //                return true;
            //            }
            return false; // keep this shape and we have set it's isReal() flag
        }
        
        void RgbdSurfaceTracker::filterDetections(const cv::rgbd::RgbdImage& rgbd_img,
                const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&query_shapeMap,
                cv::Mat& rgb_result) {

            cv::Size tileDims(BLOCKSIZE, BLOCKSIZE);

            for (auto shapeType_iter = query_shapeMap.begin();
                    shapeType_iter != query_shapeMap.end(); ++shapeType_iter) {
                cv::rgbd::SurfaceType shapeType = (*shapeType_iter).first;
                if (shapeType == SurfaceType::EDGE || shapeType == SurfaceType::CORNER) {
                    std::vector<sg::Shape::Ptr>& shapeVec = (*shapeType_iter).second;

                    // erase features too close to image boundary for analysis
                    shapeVec.erase(std::remove_if(shapeVec.begin(), shapeVec.end(),
                            boost::bind(&RgbdSurfaceTracker::filterShape, this, _1,
                            boost::ref(rgbd_img), boost::ref(quadTree),
                            boost::ref(tileDims), boost::ref(shapeType),
                            boost::ref(rgb_result))),
                            shapeVec.end());
                }
            }
        }

        void RgbdSurfaceTracker::clusterDetections(
                std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
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

            for (std::pair<const SurfaceType, std::vector < sg::Shape::Ptr>>&element : query_shapeMap) {

                std::size_t k = 1;
                std::vector<std::array<float, 3 >> kmeans_data;
                kmeans_data.reserve(element.second.size());
                std::tuple<std::vector<std::array<float, 3 >>, std::vector < uint32_t>> kmeans_result;

                switch (element.first) {
                    case SurfaceType::CORNER:
                        for (sg::Shape::Ptr shape_ptr : element.second) {
                            sg::Corner<float>::Ptr corner = boost::static_pointer_cast<sg::Corner<float>>(shape_ptr);
                            corner->getPose().getTranslation(position);
                            //std::cout << "corner position: " << position << "\n";
                            kmeans_data.emplace_back<std::array<float, 3 >> ({position(0), position(1), position(2)});

                        }

                        if (kmeans_data.size() > 1) {
                            kmeans_result = dkm::kmeans_lloyd(kmeans_data, k);
                            std::cout << "corners detected: " << kmeans_data.size() << ", clusters: " << std::get<0>(kmeans_result).size() << "\n";
                            std::cout << "cluster results:\n";
                            for (std::array<float, 3> cluster : std::get<0>(kmeans_result)) {
                                for (float dim : cluster) {
                                    std::cout << dim << " ";
                                }
                                std::cout << std::endl;
                            }
                        }
                        break;

                    case SurfaceType::EDGE:

                        for (sg::Shape::Ptr shape_ptr : element.second) {
                            sg::Edge<float>::Ptr edge = boost::static_pointer_cast<sg::Edge<float>>(shape_ptr);
                            edge->getPose().getTranslation(position);
                            kmeans_data.emplace_back<std::array<float, 3 >> ({position(0), position(1), position(2)});
                        }
                        break;

                    case SurfaceType::PLANE:

                        for (sg::Shape::Ptr shape_ptr : element.second) {
                            sg::Plane<float>::Ptr plane = boost::static_pointer_cast<sg::Plane<float>>(shape_ptr);
                            plane->getPose().getTranslation(position);
                            kmeans_data.emplace_back<std::array<float, 3 >> ({position(0), position(1), position(2)});
                        }

                    default:
                        break;
                }
            }
        }

        bool RgbdSurfaceTracker::estimateDeltaPose(
                const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&train_shapeMap,
                const std::vector<cv::rgbd::ShapeMatch>& matches,
                Pose& delta_pose_estimate) {

            sg::Corner<float>::Ptr cornerPtr;
            sg::Edge<float>::Ptr edgePtr;
            sg::Plane<float>::Ptr fixedPlanePtr, movingPlanePtr;
            std::vector<cv::Plane3f::Ptr> fixedPlanes;
            std::vector<cv::Plane3f::Ptr> movingPlanes;
            for (auto match_iter = matches.begin(); match_iter != matches.end(); ++match_iter) {
                const cv::rgbd::ShapeMatch& match = *match_iter;
                sg::Shape::Ptr movingShape = match.query_shape;
                sg::Shape::Ptr fixedShape = match.train_shape;
                if (match.surfaceType == cv::rgbd::PLANE) {
                    fixedPlanePtr = boost::static_pointer_cast<sg::Plane<float>>(movingShape);
                    movingPlanePtr = boost::static_pointer_cast<sg::Plane<float>>(fixedShape);
                    fixedPlanes.push_back(fixedPlanePtr);
                    movingPlanes.push_back(movingPlanePtr);
                }
            }
            cv::Mat cvTransform(4, 4, CV_32FC1);
            //directly use the buffer allocated by OpenCV
            Eigen::Map<Eigen::Matrix4f> eigenTransformMap(cvTransform.ptr<float>(0, 0));
            Eigen::Matrix4f eigenTransform = Eigen::Matrix4f::Zero(4, 4);
            if (movingPlanes.size() < 3) {
                std::cout << "RgbdSurfaceTracker::estimateDeltaPose -> Not enough plane matches ("
                        << movingPlanes.size() << ") cannot estimate pose change." << std::endl;
                return false;
            }
            int solutionHasReflection = planeListAlignmentCV(movingPlanes,
                    fixedPlanes, eigenTransform);
            if (solutionHasReflection) {
                std::cout << "RgbdSurfaceTracker::estimateDeltaPose -> Solution has a reflection..."
                        << " discarding pose change estimate." << std::endl;
                return false;
            }
            std::cout << "eigen mat = " << eigenTransform << std::endl;
            eigenTransformMap = eigenTransform;
            //std::cout << "eigen mat mapped = " << eigenTransformMap << std::endl;
            cv::transpose(cvTransform, cvTransform);
            cv::Vec3f tVec = cvTransform(cv::Rect(3, 0, 1, 3));
            cv::Vec3f rVec;
            cv::Mat rotMat = cvTransform(cv::Rect(0, 0, 3, 3));
            cv::Rodrigues(rotMat, rVec);
            //cv::transpose(rotMat, rotMat);
            std::cout << "R = " << rotMat << std::endl;
            std::cout << "t = " << tVec << std::endl;
            delta_pose_estimate.set(tVec, rVec);
            return true;
        }

        void RgbdSurfaceTracker::segmentDepth(cv::rgbd::RgbdImage& rgbd_img, cv::Mat & rgb_result) {

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif
            bool offscreen_rendering = false;

            if (!glDraw.initialized()) {
                glDraw.init(rgbd_img.getWidth(), rgbd_img.getHeight(), offscreen_rendering);
            }

            //cv::Rect rectVal(290, 200, 640 - 2 * 290, 480 - 2 * 200);
            //cv::rectangle(rgb_result, rectVal, cv::Scalar(0, 255, 0), 3);
            //rgbd_img.computeNormals();

            cv::Rect roi(MARGIN_X, MARGIN_Y, rgbd_img.getWidth() - 2 * MARGIN_X, rgbd_img.getHeight() - 2 * MARGIN_Y);
            cv::Size imgSize(rgbd_img.getWidth(), rgbd_img.getHeight());
            cv::Size tileSize(BLOCKSIZE, BLOCKSIZE);
            cv::QuadTree<sg::Plane<float>::Ptr>::Ptr quadTree(new QuadTree<sg::Plane<float>::Ptr>(imgSize, tileSize, roi));

            int detector_timeBudget_ms = 15;
            surfdetector.detect(rgbd_img, quadTree, detector_timeBudget_ms, rgb_result);

            int descriptor_timeBudget_ms = 20;

            cv::rgbd::ShapeMap::Ptr query_shapeMapPtr = cv::rgbd::ShapeMap::create();
            cv::rgbd::ShapeMap& query_shapeMap = *query_shapeMapPtr;

            surfdescriptor_extractor.compute(rgbd_img, quadTree, query_shapeMap,
                    descriptor_timeBudget_ms, rgb_result);

            // Validate higher-order detections:
            // We want to discriminate between virtual/real corner detections and virtual/real edge detections
            // TODO: Implement filterDetections(quadTree, query_shapeMap)
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
            filterDetections(rgbd_img, quadTree, query_shapeMap, rgb_result);

            // Cluster detections:
            // Merge multiple detections of the same structure
            // TODO: Implement clusterDetections(query_shapeMap)
            // This function visits all features for matching, e.g., planes, edges and corners,
            // and clusters the detections into groups and produces a smaller collection of
            // features for integration with the "map"
            //clusterDetections(query_shapeMap, rgb_result);

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

            // Match structures detected in this image with structures of the map
            // Compute the map between detected structures and existing structures
            std::vector<sg::Shape::Ptr> newShapes;

            // Code here ensures the new image has enough information to
            // be used for matching. If we use it, then we must be sure that we
            // can match to this map in both the current the next frame.
            if (quadTree->getData().size() < 50) {
                return; // not enough information to use this image
            }
            
            if (prev_quadTree && train_shapeMapPtr) {
                cv::rgbd::ShapeMap& train_shapeMap = *train_shapeMapPtr;
                int descriptor_matching_timeBudget_ms = 20;
                std::vector<cv::rgbd::ShapeMatch> shapeMatches;
                surfmatcher.match(quadTree, query_shapeMap,
                        prev_quadTree, train_shapeMap,
                        shapeMatches, newShapes,
                        descriptor_matching_timeBudget_ms, rgb_result);
                // Structures having a match are used to solve the following problems:
                // 1. Localization -> (Range-based Odometry / Pose-change estimation)
                if (!estimateDeltaPose(query_shapeMap, train_shapeMap, shapeMatches, delta_pose_estimate)) {
                    std::cout << "Could not estimate pose change from provided shape matches!" << std::endl;
                    return;
                } else { // use pose estimate to update map
                    cv::Matx44f pose = global_pose_estimate.getTransform();
                    cv::Matx44f deltaPose = delta_pose_estimate.getTransform();
                    cv::Matx44f new_pose = pose*deltaPose;
                    // TODO: implement set function for pose
                    //global_pose_estimate.set(newPose);
                    // Mapping -> Structure estimation
                    //updateMap(query_shapeMap, train_shapeMap);
                    world_map->update(quadTree, query_shapeMap,
                            prev_quadTree, train_shapeMap,
                            shapeMatches, global_pose_estimate);
                }
            }
            //addToMap(unmatched)
            world_map->insert(newShapes, global_pose_estimate);

            prev_quadTree = quadTree;
            train_shapeMapPtr = query_shapeMapPtr;
            //iterativeAlignment(rgbd_img, rgb_result);

#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif      
        }

        void ShapeMap::update(const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                const std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& prev_quadTree,
                const std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&train_shapeMap,
                std::vector<cv::rgbd::ShapeMatch>& matches, Pose global_pose) {
        }

        void ShapeMap::insert(std::vector<sg::Shape::Ptr>& newShapes, Pose global_pose) {

        }

    } /* namespace rgbd */
} /* namespace cv */
