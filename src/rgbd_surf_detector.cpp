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

        void SurfaceDetector::detect(const cv::rgbd::RgbdImage& rgbd_img,
                cv::QuadTree<sg::Plane<float>::Ptr>& quadTree,
                std::vector<AlgebraicSurfacePatch::Ptr>& geometries,
                cv::Mat& rgb_result, cv::Mat mask) const {
            int numLabels = 1;
            cv::ErrorSortedRectQueue quadQueue;
            std::vector<sg::Plane<float>::Ptr> planeList;
            cv::Mat img_L;
            findPlanes(quadQueue, rgbd_img, quadTree, img_L, numLabels);
            std::unordered_map<int, sg::Plane<float>::Ptr> data = quadTree.getData();
            for (auto it = data.begin(); it != data.end(); ++it) {
                int key = it->first;
                sg::Plane<float>::Ptr& planeA = it->second;
                int qX, qY;
                quadTree.keyToXY(key, qX, qY);
                std::cout << "Quad(" << qX << ", " << qY << ") = " << ((!planeA) ? "null" : (*planeA).toString()) << std::endl;
                if (planeA) {
                    AlgebraicSurfacePatch::Ptr surfPatch = AlgebraicSurfacePatch::create(planeA, rgbd_img);
                    if (planeA->avgError() < 0.0025 * surfPatch->getAverageDepth()) {
                        //std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA << std::endl;
                        geometries.push_back(surfPatch);
                    }
                }
            }
        }

        void SurfaceDetector::findPlanes(ErrorSortedRectQueue& quadQueue,
                const RgbdImage& rgbd_proc,
                QuadTreeLevel<sg::Plane<float>::Ptr>& quadTree,
                Mat& img_labels, int& numLabels) const {
            int numSubdivisions = 0;
            bool timeBudgetExpired = false;
            cv::Plane3f plane3;
            sg::Plane<float>::Ptr nullPtr;
            RectWithError quad;
            cv::Size tileDims = quadTree.getTileDims();            
            for (int y_tile = 0; y_tile < tileDims.height; ++y_tile) {
                for (int x_tile = 0; x_tile < tileDims.width; ++x_tile) {
                    quadTree.setRect(x_tile, y_tile, quad);
                    std::vector<cv::Point3f> tile_data; // data for each tile in array of vectors
                    int numSamples = (quad.width * quad.height) / 2;
                    tile_data.reserve(numSamples);
                    float avgDepth = 0.25f * (rgbd_proc.getDepth(quad.x, quad.y) +
                            rgbd_proc.getDepth(quad.x + quad.width, quad.y) +
                            rgbd_proc.getDepth(quad.x + quad.width, quad.y + quad.height) +
                            rgbd_proc.getDepth(quad.x, quad.y + quad.height));

                    rgbd_proc.getTileData_Uniform(quad.x, quad.y,
                            quad.width, quad.height, tile_data, numSamples);
                    if (tile_data.size() > numSamples >> 2) {
                        rgbd_proc.fitImplicitPlaneLeastSquares(tile_data, plane3, quad.error,
                                quad.noise, quad.inliers, quad.outliers, quad.invalid);
                        //sg::Plane<float>::Ptr& cBlock = quadTree.get(x_tile, y_tile);
                        if (quad.error / (quad.inliers + quad.outliers) < 0.0025 * avgDepth) {
                            //std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA << std::endl;
                            cv::TesselatedPlane3f tplane(plane3, numLabels++);
                            tplane.addQuad(quad);
                            sg::Plane<float>::Ptr planePtr(new sg::Plane<float>(tplane));
                            quadTree.insert_or_assign(x_tile, y_tile, planePtr);
                            RectWithError newQuad = quad.clone();
                            quad.clearStatistics();
                            planePtr->addQuad(newQuad); // error and support already known
                            quadQueue.push(newQuad);
                        } else {
                            // mark for subdivision by placing a null shared pointer in the quadtree
                            quadTree.insert_or_assign(x_tile, y_tile, nullPtr);  
                            numSubdivisions++;
                        }
                    } else {
                        // not enough samples found in tile stop recursions
                    }
                }
            }
            if (numSubdivisions > 0 && timeBudgetExpired) {
                //QuadTreeLevel<sg::Plane<float>::Ptr>& nextLevel = quadTree.getQuadTreeLevel(quadTree.getLevel()+1);
            }
        }

    } /* namespace rgbd */
} /* namespace cv */