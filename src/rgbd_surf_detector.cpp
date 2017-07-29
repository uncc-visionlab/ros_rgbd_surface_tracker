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
                sg::Plane<float>::Ptr& planeA = it->second;
                AlgebraicSurfacePatch::Ptr surfPatch = AlgebraicSurfacePatch::create(planeA, rgbd_img);
                if (planeA->avgError() < 0.0025 * surfPatch->getAverageDepth()) {
                    //std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA << std::endl;
                    geometries.push_back(surfPatch);
                }
            }
        }

        void SurfaceDetector::findPlanes(ErrorSortedRectQueue& quadQueue,
                const RgbdImage& rgbd_proc,
                QuadTree<sg::Plane<float>::Ptr>& quadTree,
                Mat& img_labels, int& numLabels) const {
            // block analyzed must lie on quadtree grid and inside the ROI
            Plane3f plane3;

            RectWithError quad;
            cv::Size tileDims = quadTree.getTileDims();
            for (int y_tile = 0; y_tile < tileDims.height; ++y_tile) {
                for (int x_tile = 0; x_tile < tileDims.width; ++x_tile) {
                    quadTree.setRect(x_tile, y_tile, quad);
                    std::vector<cv::Point3f> tile_data; // data for each tile in array of vectors
                    int numSamples = (quad.width * quad.height) / 2;
                    tile_data.reserve(numSamples);

                    rgbd_proc.getTileData_Uniform(quad.x, quad.y,
                            quad.width, quad.height, tile_data, numSamples);
                    if (tile_data.size() > numSamples >> 2) {
                        rgbd_proc.fitImplicitPlaneLeastSquares(tile_data, plane3, quad.error,
                                quad.noise, quad.inliers, quad.outliers, quad.invalid);
                        cv::TesselatedPlane3f tplane(plane3, numLabels++);
                        tplane.addQuad(quad);
                        sg::Plane<float>::Ptr& cBlock = quadTree.get(x_tile, y_tile);
                        cBlock = sg::Plane<float>::Ptr(new sg::Plane<float>(tplane));

                        RectWithError newQuad = quad.clone();
                        quad.clearStatistics();
                        cBlock->addQuad(newQuad); // error and support already known
                        quadQueue.push(newQuad);
                    } else {

                    }
                }
            }
        }

    } /* namespace rgbd */
} /* namespace cv */