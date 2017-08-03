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
                cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                int timeBudget_ms, cv::Mat& rgb_result, cv::Mat mask) const {
            cv::ErrorSortedRectQueue quadQueue;
            cv::Mat img_L;
            int numLabels = 1;

            bool timeBudgetExpired = false;
            int64 timeBudgetTicks = timeBudget_ms * cv::getTickFrequency() / 1000;
            int64 ticksNow, ticksStart = cv::getTickCount();

            std::vector<Point2i> recurseTileVec = findPlanes(quadQueue, rgbd_img, quadTree, img_L, numLabels);
            //populateGeometries(rgbd_img, &quadTree, geometries);

            ticksNow = cv::getTickCount();
            timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;

            int level = quadTree->getLevel();
            while (recurseTileVec.size() > 0 && !timeBudgetExpired) {
                QuadTreeLevel<sg::Plane<float>::Ptr>* nextLevel_ptr = quadTree->getQuadTreeLevel(++level);
                if (!nextLevel_ptr) {
                    break;
                }
                recurseTileVec = recursiveSubdivision(quadQueue, rgbd_img, nextLevel_ptr, recurseTileVec, img_L, numLabels);
                //populateGeometries(rgbd_img, nextLevel_ptr, geometries);

                ticksNow = cv::getTickCount();
                timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;
            }
            std::cout << "Detector time used = " << ((double) (ticksNow - ticksStart) / cv::getTickFrequency())
                    << " sec time allocated = " << ((double) timeBudgetTicks / cv::getTickFrequency()) << " sec" << std::endl;
        }

        void setPlaneUVCoords(sg::Plane<float>::Ptr plane_ptr, const RgbdImage& rgbdImg, cv::Rect& imgTile) {
            cv::Point3f pts[4];
            rgbdImg.getPoint3f(imgTile.x, imgTile.y, pts[0]);
            rgbdImg.getPoint3f(imgTile.x + imgTile.width, imgTile.y, pts[1]);
            rgbdImg.getPoint3f(imgTile.x + imgTile.width, imgTile.y + imgTile.height, pts[2]);
            rgbdImg.getPoint3f(imgTile.x, imgTile.y + imgTile.height, pts[3]);
            std::vector<cv::Vec2f> plane_uv_coords;
            cv::Vec3f meanPos = 0.25 * (pts[0] + pts[1] + pts[2] + pts[3]);
            cv::Point3f conv_error;
            for (cv::Point3f pt : pts) {
                cv::Point2f uv_coord = plane_ptr->xyzToUV(pt);
                plane_uv_coords.push_back(uv_coord);
                // TODO Resolve circumstances when this conversion can have large error & fix/triage.
                if (false) { // likely cause is fit error calculation, inliers & outliers
                    cv::Point3f xyz3 = plane_ptr->uvToXYZ(uv_coord);
                    conv_error = xyz3 - pt;
                    float conv_error_lensq = conv_error.dot(conv_error);
                    if (conv_error_lensq > 0.05) {
                        std::cout << "plane = " << plane_ptr->toString() << std::endl;
                        std::cout << "xyz = " << pt << "-> uv = " << uv_coord << std::endl;
                        std::cout << "uv = " << uv_coord << "-> xyz = " << xyz3 << std::endl;
                        std::cout << "Error = " << std::sqrt(conv_error_lensq) << std::endl;
                        uv_coord = plane_ptr->xyzToUV(pt);
                        xyz3 = plane_ptr->uvToXYZ(uv_coord);
                    }
                }
            }
            plane_ptr->addCoords(plane_uv_coords);

            // create texture coordinates
            std::vector<cv::Vec2f> plane_uv_texcoords(4);
            plane_uv_texcoords[0] = cv::Vec2f(((float) imgTile.x) / rgbdImg.getWidth(),
                    1.0f - ((float) imgTile.y / (float) rgbdImg.getHeight()));
            plane_uv_texcoords[1] = cv::Vec2f(((float) imgTile.x + imgTile.width) / rgbdImg.getWidth(),
                    1.0f - ((float) imgTile.y / (float) rgbdImg.getHeight()));
            plane_uv_texcoords[2] = cv::Vec2f(((float) imgTile.x + imgTile.width) / rgbdImg.getWidth(),
                    1.0f - ((float) imgTile.y + imgTile.height) / (float) rgbdImg.getHeight());
            plane_uv_texcoords[3] = cv::Vec2f(((float) imgTile.x) / rgbdImg.getWidth(),
                    1.0f - ((float) imgTile.y + imgTile.height) / (float) rgbdImg.getHeight());
            plane_ptr->addTexCoords(plane_uv_texcoords);
            plane_ptr->setPose(Pose(meanPos, cv::Vec3f(plane_ptr->x, plane_ptr->y, plane_ptr->z)));
        }

        std::vector<cv::Point2i> SurfaceDetector::findPlanes(ErrorSortedRectQueue& quadQueue,
                const RgbdImage& rgbd_img,
                const QuadTreeLevel<sg::Plane<float>::Ptr>::Ptr& quadTree,
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
                        if (quad.error / (quad.inliers + quad.outliers) < 0.0025 * avgDepth) {
                            //&& plane3.d > 0 && abs(plane3.x) < 0.99 && abs(plane3.y) < 0.99) {
                            //std::cout << "areaA " << planeA->area() << " errorA = " 
                            //<< planeA->avgError() << " planeA = " << *planeA << std::endl;
                            cv::TesselatedPlane3f tplane(plane3, numLabels++);
                            tplane.addQuad(quad);
                            sg::Plane<float>::Ptr planePtr(new sg::Plane<float>(tplane));
                            setPlaneUVCoords(planePtr, rgbd_img, quad);
                            quadTree->insert_or_assign(x_tile, y_tile, planePtr);
                            RectWithError newQuad = quad.clone();
                            quad.clearStatistics();
                            planePtr->addQuad(newQuad); // error and support already known
                            quadQueue.push(newQuad);
                        } else {
                            // mark tile coordinates for subdivision
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
                const std::vector<cv::Point2i>& subdivideTileVec,
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
                        //&& plane3.d > 0 && abs(plane3.x) < 0.99 && abs(plane3.y) < 0.99) {
                        //std::cout << "areaA " << planeA->area() << " errorA = " 
                        //<< planeA->avgError() << " planeA = " << *planeA << std::endl;
                        cv::TesselatedPlane3f tplane(plane3, numLabels++);
                        tplane.addQuad(quad);
                        sg::Plane<float>::Ptr planePtr(new sg::Plane<float>(tplane));
                        setPlaneUVCoords(planePtr, rgbd_img, quad);
                        quadTree->insert_or_assign(tileIdx.x, tileIdx.y, planePtr);
                        RectWithError newQuad = quad.clone();
                        quad.clearStatistics();
                        planePtr->addQuad(newQuad); // error and support already known
                        quadQueue.push(newQuad);
                    } else {
                        // mark tile coordinates for subdivision
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