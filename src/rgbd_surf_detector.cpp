/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <vector>

#include <opencv2/core.hpp>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

namespace cv {
    namespace rgbd {

        void populateGeometries(const cv::rgbd::RgbdImage& rgbd_img,
                QuadTreeLevel<sg::Plane<float>::Ptr>* quadTree,
                std::vector<AlgebraicSurfacePatch::Ptr>& geometries) {
            std::unordered_map<int, sg::Plane<float>::Ptr> data = quadTree->getData();
            for (auto it = data.begin(); it != data.end(); ++it) {
                //int key = it->first;
                //int qX, qY;
                //quadTree->keyToXY(key, qX, qY);
                //std::cout << "Quad(" << qX << ", " << qY << ") = " << ((!planeA) ? "null" : (*planeA).toString()) << std::endl;
                sg::Plane<float>::Ptr& planeA = it->second;
                if (planeA) {
                    AlgebraicSurfacePatch::Ptr surfPatch = AlgebraicSurfacePatch::create(planeA, rgbd_img);
                    if (planeA->avgError() < 0.0025 * surfPatch->getAverageDepth()) {
                        //std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA << std::endl;
                        geometries.push_back(surfPatch);
                    }
                }
            }
        }

        void SurfaceDetector::detect(const cv::rgbd::RgbdImage& rgbd_img,
                cv::QuadTree<sg::Plane<float>::Ptr>& quadTree,
                std::vector<AlgebraicSurfacePatch::Ptr>& geometries,
                cv::Mat& rgb_result, cv::Mat mask) const {
            int numLabels = 1;
            cv::ErrorSortedRectQueue quadQueue;
            std::vector<sg::Plane<float>::Ptr> planeList;
            cv::Mat img_L;

            bool timeBudgetExpired = false;
            int timeBudget_ms = 10;
            int64 timeBudgetTicks = timeBudget_ms * cv::getTickFrequency() / 1000;
            int64 ticksNow, ticksStart = cv::getTickCount();

            std::vector<Point2i> recurseTileVec = findPlanes(quadQueue, rgbd_img, &quadTree, img_L, numLabels);
            populateGeometries(rgbd_img, &quadTree, geometries);

            ticksNow = cv::getTickCount();
            timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;

            int level = quadTree.getLevel();
            while (recurseTileVec.size() > 0 && !timeBudgetExpired) {
                QuadTreeLevel<sg::Plane<float>::Ptr>* nextLevel_ptr = quadTree.getQuadTreeLevel(++level);
                if (!nextLevel_ptr) {
                    break;
                }
                recurseTileVec = recursiveSubdivision(quadQueue, rgbd_img, nextLevel_ptr, recurseTileVec, img_L, numLabels);
                populateGeometries(rgbd_img, nextLevel_ptr, geometries);

                ticksNow = cv::getTickCount();
                timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;
            }
            std::cout << "quadtree time used" << ((double) (ticksNow - ticksStart) / cv::getTickFrequency())
                    << " allocated time " << ((double) timeBudgetTicks / cv::getTickFrequency()) << std::endl;
        }

        std::vector<cv::Point2i> SurfaceDetector::findPlanes(ErrorSortedRectQueue& quadQueue,
                const RgbdImage& rgbd_img,
                QuadTreeLevel<sg::Plane<float>::Ptr>* quadTree,
                Mat& img_labels, int& numLabels) const {
            std::vector<cv::Point2i> recurseTileVec;
            cv::Plane3f plane3;
            RectWithError quad;
            cv::Size tileDims = quadTree->getTileDims();
            for (int y_tile = 0; y_tile < tileDims.height; ++y_tile) {
                for (int x_tile = 0; x_tile < tileDims.width; ++x_tile) {
                    quadTree->setRect(x_tile, y_tile, quad);
                    std::vector<cv::Point3f> tile_data; // data for each tile in array of vectors
                    int numSamples = std::max((quad.width * quad.height) / 2, 3);
                    tile_data.reserve(numSamples);
                    float avgDepth = 0.25f * (rgbd_img.getDepth(quad.x, quad.y) +
                            rgbd_img.getDepth(quad.x + quad.width, quad.y) +
                            rgbd_img.getDepth(quad.x + quad.width, quad.y + quad.height) +
                            rgbd_img.getDepth(quad.x, quad.y + quad.height));

                    rgbd_img.getTileData_Uniform(quad.x, quad.y,
                            quad.width, quad.height, tile_data, numSamples);
                    if (tile_data.size() > (numSamples >> 2)) {
                        rgbd_img.fitImplicitPlaneLeastSquares(tile_data, plane3, quad.error,
                                quad.noise, quad.inliers, quad.outliers, quad.invalid);
                        //sg::Plane<float>::Ptr& cBlock = quadTree.get(x_tile, y_tile);
                        if (quad.error / (quad.inliers + quad.outliers) < 0.0025 * avgDepth) {
                            //std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA << std::endl;
                            cv::TesselatedPlane3f tplane(plane3, numLabels++);
                            tplane.addQuad(quad);
                            sg::Plane<float>::Ptr planePtr(new sg::Plane<float>(tplane));
                            quadTree->insert_or_assign(x_tile, y_tile, planePtr);
                            RectWithError newQuad = quad.clone();
                            quad.clearStatistics();
                            planePtr->addQuad(newQuad); // error and support already known
                            quadQueue.push(newQuad);
                        } else {
                            // mark for subdivision by placing a null shared pointer in the quadtree
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 0));
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 0));
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 1));
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 1));
                        }
                    } else {
                        // not enough samples found in tile -> stop recursions
                    }
                }
            }
            return recurseTileVec;
        }

        std::vector<cv::Point2i> SurfaceDetector::recursiveSubdivision(ErrorSortedRectQueue& quadQueue,
                const RgbdImage& rgbd_img,
                QuadTreeLevel<sg::Plane<float>::Ptr>* quadTree,
                std::vector<cv::Point2i>& subdivideTileVec,
                Mat& img_labels, int& numLabels) const {
            std::vector<cv::Point2i> recurseTileVec;
            cv::Plane3f plane3;
            RectWithError quad;
            for (cv::Point2i tileIdx : subdivideTileVec) {
                quadTree->setRect(tileIdx.x, tileIdx.y, quad);
                std::vector<cv::Point3f> tile_data; // data for each tile in array of vectors
                int numSamples = (quad.width * quad.height) / 2;
                tile_data.reserve(numSamples);
                float avgDepth = 0.25f * (rgbd_img.getDepth(quad.x, quad.y) +
                        rgbd_img.getDepth(quad.x + quad.width, quad.y) +
                        rgbd_img.getDepth(quad.x + quad.width, quad.y + quad.height) +
                        rgbd_img.getDepth(quad.x, quad.y + quad.height));

                rgbd_img.getTileData_Uniform(quad.x, quad.y,
                        quad.width, quad.height, tile_data, numSamples);
                if (tile_data.size() > (numSamples >> 2)) {
                    rgbd_img.fitImplicitPlaneLeastSquares(tile_data, plane3, quad.error,
                            quad.noise, quad.inliers, quad.outliers, quad.invalid);
                    if (quad.error / (quad.inliers + quad.outliers) < 0.0025 * avgDepth) {
                        //std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA << std::endl;
                        cv::TesselatedPlane3f tplane(plane3, numLabels++);
                        tplane.addQuad(quad);
                        sg::Plane<float>::Ptr planePtr(new sg::Plane<float>(tplane));
                        quadTree->insert_or_assign(tileIdx.x, tileIdx.y, planePtr);
                        RectWithError newQuad = quad.clone();
                        quad.clearStatistics();
                        planePtr->addQuad(newQuad); // error and support already known
                        quadQueue.push(newQuad);
                    } else {
                        // mark for subdivision by placing a null shared pointer in the quadtree
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 0, (tileIdx.y << 1) + 0));
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 1, (tileIdx.y << 1) + 0));
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 0, (tileIdx.y << 1) + 1));
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 1, (tileIdx.y << 1) + 1));
                    }
                } else {
                    // not enough samples found in tile -> stop recursions
                }
            }
            return recurseTileVec;
        }

    } /* namespace rgbd */
} /* namespace cv */