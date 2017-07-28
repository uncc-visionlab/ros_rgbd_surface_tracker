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
                cv::QuadPyramid<cv::TesselatedPlane3f::Ptr>& quadTree,
                cv::Rect roi,
                std::vector<AlgebraicSurfacePatch::Ptr>& geometries,
                cv::Mat& rgb_result, cv::Mat mask) const {

            cv::Size imSize(rgbd_img.getDepth().size());
            //            int blockSize = BLOCKSIZE;
                        int numLabels = 1;
            //            Rect roi(MARGIN_X, MARGIN_Y, imSize.width - 2 * MARGIN_X, imSize.height - 2 * MARGIN_Y);
            //            int xBlocks = (int) cvFloor((float) roi.width / blockSize);
            //            int yBlocks = (int) cvFloor((float) roi.height / blockSize);
            //            //std::cout << "(xBlocks, yBlocks) = (" << xBlocks << ", " << yBlocks << ")" << std::endl;
            //            cv::QuadPyramid<cv::TesselatedPlane3f::Ptr> quadTree(xBlocks, yBlocks, blockSize);
            cv::ErrorSortedRectQueue quadQueue;
            std::vector<cv::TesselatedPlane3f::Ptr> planeList;
            cv::Mat img_L;
            findPlanes(roi, quadQueue, planeList, rgbd_img, quadTree, img_L, numLabels);
            for (int indexA = 0; indexA < planeList.size(); indexA++) {
                TesselatedPlane3f::Ptr& planeA = planeList[indexA];
                AlgebraicSurfacePatch::Ptr surfPatch = AlgebraicSurfacePatch::create(planeA, rgbd_img);
                if (planeA->avgError() < 0.0025 * surfPatch->getAverageDepth()) {
                    //std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA << std::endl;
                    geometries.push_back(surfPatch);
                }
            }
        }

        void SurfaceDetector::findPlanes(const cv::Rect& roi,
                ErrorSortedRectQueue& quadQueue,
                std::vector<TesselatedPlane3f::Ptr>& planeList,
                const RgbdImage& rgbd_proc,
                QuadPyramid<TesselatedPlane3f::Ptr>& quadTree,
                Mat& img_labels, int& numLabels) const {
            // block analyzed must lie on quadtree grid and inside the ROI
            TesselatedPlane3f::Ptr topBlock, leftBlock;
            Plane3f plane3;
            std::vector<TesselatedPlane3f::Ptr>& planeArray = quadTree.getObjects();
            int blockSize = quadTree.blockSize;
            int numPoints = (blockSize * blockSize) >> 2;

            //rgbd_proc.randPt.initializePatterns(blockSize);

            int x0 = (roi.x / blockSize) * blockSize, y0 = (roi.y / blockSize) * blockSize;
            int xBlock = x0 / blockSize, yBlock = y0 / blockSize;
            int xBlocks = roi.width / blockSize, yBlocks = roi.height / blockSize;
            RectWithError quad;
            quad.width = quad.height = blockSize;
            for (; yBlock < yBlocks; y0 += blockSize, ++yBlock) {
                for (x0 = (roi.x / blockSize) * blockSize, xBlock = x0 / blockSize;
                        xBlock < xBlocks; x0 += blockSize, ++xBlock) {
                    //std::cout << "processing tile " << Point2i(xBlock, yBlock) << " of " << Point2i(xBlocks, yBlocks) << std::endl;
                    // if returns false  no fit is possible
                    // if returns true, plane3, error and noise have analysis values
                    //if (rgbd_proc.fitPlaneSimple(x0, y0, blockSize, plane3, error, noise)) {
                    //RectWithError quad(x0, y0, blockSize, blockSize);
                    quad.x = x0;
                    quad.y = y0;
                    std::vector<cv::Point3f> tile_data; // data for each tile in array of vectors
                    int numSamples = (quad.width * quad.height) / 2;
                    tile_data.reserve(numSamples);

                    rgbd_proc.getTileData_Uniform(quad.x, quad.y,
                            quad.width, quad.height,
                            tile_data, numSamples);
                    if (tile_data.size() > numSamples >> 2) {
                        rgbd_proc.fitImplicitPlaneLeastSquares(tile_data, plane3, quad.error,
                                quad.noise, quad.inliers, quad.outliers, quad.invalid);
                        TesselatedPlane3f::Ptr& cBlock = planeArray[yBlock * xBlocks + xBlock];
                        cBlock = TesselatedPlane3f::Ptr(
                                new TesselatedPlane3f(plane3, numLabels++));
                        planeList.push_back(cBlock);
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