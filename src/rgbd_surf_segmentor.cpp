/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rgbd_surf_segmentor.cpp
 * Author: jzhang72
 *
 * Created on April 12, 2021, 10:54 AM
 */

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

#define PI 3.14159

namespace cv {
    namespace rgbd {
        float PLANE_AVG_FIT_ERROR_THRESHOLD_QUADTREE_SPLIT = 0.005;
        float PLANE_MAX_FIT_ERROR_THRESHOLD_QUADTREE_SPLIT = 0.130;
        int MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT = 2 * 2;

        void SurfaceSegmentor::detect(const cv::rgbd::RgbdImage& rgbd_img,
                cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                int timeBudget_ms, cv::Mat& rgb_result, cv::Mat& img_L) const {
            cv::ErrorSortedRectQueue quadQueue;
            //cv::Mat img_L = cv::Mat::zeros(rgbd_img.getHeight(), rgbd_img.getWidth() , CV_32SC1);
            int numLabels = 1;
            int new_label = 1;

            bool timeBudgetExpired = false;
            int64 timeBudgetTicks = timeBudget_ms * cv::getTickFrequency() / 100;
            int64 ticksNow, ticksStart = cv::getTickCount();

            std::vector<Point2i> recurseTileVec = mergePlanes(quadQueue, rgbd_img, quadTree, img_L, numLabels, new_label);
            //populateGeometries(rgbd_img, &quadTree, geometries);

            ticksNow = cv::getTickCount();
            timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;

            int level = quadTree->getLevel();
            while (recurseTileVec.size() > 0 && !timeBudgetExpired) {
                QuadTreeLevel<sg::Plane<float>::Ptr>* nextLevel_ptr = quadTree->getQuadTreeLevel(++level);
                if (!nextLevel_ptr) {
                    break;
                }
                recurseTileVec = recursiveMergeSubdivision(quadQueue, rgbd_img, nextLevel_ptr, recurseTileVec, img_L, numLabels, quadTree, new_label);
                //populateGeometries(rgbd_img, nextLevel_ptr, geometries);

                ticksNow = cv::getTickCount();
                timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;
            }
//            cv::Mat segLabelImg(img_L.rows, img_L.cols, CV_8UC3);
//            int golden_angle = (int) (180 * (3 - sqrt(5))) % 360;
//            cv::Mat label_hue(img_L.rows, img_L.cols, CV_8UC1);
//            for (int i = 0; i < img_L.rows; i++) {
//                for (int j = 0; j < img_L.cols; j++) {
//                    label_hue.at<uchar>(i, j) = img_L.at<int>(i, j) * golden_angle; // each label will be nearly maximally different in hue
//                }
//            }
//            cv::Mat blank(label_hue.rows, label_hue.cols, CV_8UC1, cv::Scalar(255));
//            std::vector<cv::Mat> hsvChannels = {label_hue, blank, blank};
//            cv::Mat hsvImage(label_hue.rows, label_hue.cols, CV_8UC3);
//            cv::merge(hsvChannels, hsvImage);
//            cv::cvtColor(hsvImage, segLabelImg, cv::COLOR_HSV2BGR);
//            cv::imwrite("SegLabels.jpg", segLabelImg);
            std::cout << "QuadTree level = " << level << std::endl;
            std::cout << "Detector time used = " << ((double) (ticksNow - ticksStart) / cv::getTickFrequency())
                    << " sec time allocated = " << ((double) timeBudgetTicks / cv::getTickFrequency()) << " sec" << std::endl;
        }

        bool setPlaneUVCoords2(sg::Plane<float>::Ptr plane_ptr, const RgbdImage& rgbdImg, cv::RectWithError& imgTile) {
            cv::Point3f pts[4];
            rgbdImg.getPoint3f(imgTile.x, imgTile.y, pts[0]);
            rgbdImg.getPoint3f(imgTile.x + imgTile.width, imgTile.y, pts[1]);
            rgbdImg.getPoint3f(imgTile.x + imgTile.width, imgTile.y + imgTile.height, pts[2]);
            rgbdImg.getPoint3f(imgTile.x, imgTile.y + imgTile.height, pts[3]);
            std::vector<cv::Vec2f> plane_uv_coords;
            cv::Vec3f meanPos = 0.25 * (pts[0] + pts[1] + pts[2] + pts[3]);
            cv::Point3f conv_error;
            int numUV = 0;
            for (cv::Point3f pt : pts) {
                cv::Point2f uv_coord = plane_ptr->xyzToUV(pt);
                plane_uv_coords.push_back(uv_coord);
                numUV++;
                // TODO: This filter should not be necessary if other criteria were more appropriate
                // improve PLANE_FIT_ERROR_THRESHOLD
                // improve checkTilePlanar() 
                cv::Point3f xyz3 = plane_ptr->uvToXYZ(uv_coord);
                conv_error = xyz3 - pt;
                float conv_error_lensq = conv_error.dot(conv_error);
                if (conv_error_lensq > 0.005) { // corner is > 5mm out of the plane -> reject the plane
                    return false; // does not look square-like in UV coords! -> delete
                }
                if (numUV >= 2) {
                    float sidelength = cv::norm(plane_uv_coords[numUV - 1] - plane_uv_coords[numUV - 2]);
                    if (sidelength > 0.4) {
                        return false;
                    }
                }
                // TODO: Resolve circumstances when this conversion can have large error & fix/triage.
                if (false) { // likely cause is fit error calculation, inliers & outliers
                    cv::Point3f xyz3 = plane_ptr->uvToXYZ(uv_coord);
                    conv_error = xyz3 - pt;
                    std::cout << "rect  = " << imgTile << std::endl;
                    float conv_error_lensq = conv_error.dot(conv_error);
                    std::cout << "plane = " << plane_ptr->toString() << std::endl;
                    std::cout << "xyz = " << pt << "-> uv = " << uv_coord << std::endl;
                    std::cout << "uv = " << uv_coord << "-> xyz = " << xyz3 << std::endl;
                    std::cout << "Error = " << std::sqrt(conv_error_lensq) << std::endl;
                    if (conv_error_lensq > 0.05) {
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
            cv::Vec3f planePt2meanPos = meanPos - (cv::Vec3f) plane_ptr->getClosestPointToOrigin();
            cv::Vec3f inPlanePt = meanPos - planePt2meanPos.dot(*plane_ptr) * (cv::Vec3f)(*plane_ptr);
            //std::cout << "inPlanePt " << inPlanePt << " meanPos " << meanPos << " eval(inPlanePt) " << plane_ptr->evaluate(inPlanePt) << std::endl;
            plane_ptr->setPose(Pose(inPlanePt, cv::Vec3f(plane_ptr->x, plane_ptr->y, plane_ptr->z)));
            //plane_ptr->setPose(Pose(meanPos, cv::Vec3f(plane_ptr->x, plane_ptr->y, plane_ptr->z)));
            return true;
        }

        bool checkTilePlanar2(const RgbdImage& rgbd_img, const cv::RectWithError& quad,
                float &avgDepth, std::vector<cv::Point2i>& recurseTileVec, int x_tile, int y_tile) {
            cv::Vec3f pts[5];
            rgbd_img.getPoint3f(quad.x, quad.y, pts[0][0], pts[0][1], pts[0][2]);
            rgbd_img.getPoint3f(quad.x + quad.width, quad.y, pts[1][0], pts[1][1], pts[1][2]);
            rgbd_img.getPoint3f(quad.x + quad.width, quad.y + quad.height, pts[2][0], pts[2][1], pts[2][2]);
            rgbd_img.getPoint3f(quad.x, quad.y + quad.height, pts[3][0], pts[3][1], pts[3][2]);
            rgbd_img.getPoint3f((quad.x + quad.width) >> 1, (quad.y + quad.height) >> 1, pts[4][0], pts[4][1], pts[4][2]);
            avgDepth = 0.25f * (pts[0][2] + pts[1][2] + pts[2][2] + pts[3][2]);
            //avgDepth = 0.20f * (pts[0][2] + pts[1][2] + pts[2][2] + pts[3][2] + pts[4][2]);
            if (std::isnan(avgDepth)) {
                if (!std::isnan(pts[0][2]) && !std::isnan(pts[4][2])) {
                    recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 0));
                }
                if (!std::isnan(pts[1][2]) && !std::isnan(pts[4][2])) {
                    recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 0));
                }
                if (!std::isnan(pts[3][2]) && !std::isnan(pts[4][2])) {
                    recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 1));
                }
                if (!std::isnan(pts[2][2]) && !std::isnan(pts[4][2])) {
                    recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 1));
                }
                return false;
            }
            //cv::Vec3f curvature_vec = 0.25f * (pts[0] + pts[1] + pts[2] + pts[3]) - pts[4];
            //float curvature_est = std::sqrt(curvature_vec.dot(curvature_vec));
            //std::cout << "avgDepth = " << avgDepth << " curvature = " << curvature_est << std::endl;
            //if (std::abs(curvature_est) > 0.5f) {
            //return false;
            //}
            return true;
        }

        std::vector<cv::Point2i> SurfaceSegmentor::mergePlanes(ErrorSortedRectQueue& quadQueue,
                const RgbdImage& rgbd_img,
                const QuadTreeLevel<sg::Plane<float>::Ptr>::Ptr& quadTree,
                Mat& img_labels, int& numLabels,
                int& new_label) const {
            std::vector<cv::Point2i> recurseTileVec;
            cv::Plane3f plane3;
            RectWithError quad;
            float avgDepth = 0.0;
            cv::Size tileDims = quadTree->getTileDims();
            for (int y_tile = 0; y_tile < tileDims.height; ++y_tile) {
                for (int x_tile = 0; x_tile < tileDims.width; ++x_tile) {
                    quadTree->setRect(x_tile, y_tile, quad);
                    //if (checkTilePlanar2(rgbd_img, quad, avgDepth, recurseTileVec, x_tile, y_tile) || 
                    //        (quad.area() <= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT)) {
                    if(1) {
                        std::vector<cv::Point3f> tile_data; // data for each tile in array of vectors
                        int numSamples = std::max((quad.width * quad.height) / 2, 3);
                        tile_data.reserve(numSamples);
                        rgbd_img.getTileData_Uniform(quad.x, quad.y,
                                quad.width, quad.height, tile_data, numSamples);
                        if (tile_data.size() > (numSamples >> 2)) {
                            cv::FitStatistics stats;
                            std::tie(plane3, stats) = rgbd_img.fitPlaneImplicitLeastSquaresWithStats(tile_data);
                            //std::tie(plane3, stats) = rgbd_img.fitPlaneExplicitLeastSquaresWithStats(tile_data);
                            quad.error = stats.error;
                            quad.noise = stats.noise;
                            quad.inliers = stats.inliers;
                            quad.outliers = stats.outliers;
                            quad.invalid = stats.invalid;
                            for (int num = 1; num < tile_data.size(); num++) {
                                avgDepth = avgDepth + tile_data[num].z;
                            }
                            avgDepth /= tile_data.size();
                            float scalef = (avgDepth < 2.5) ? 2.0f : 1.0f;
                            if (quad.error < 0.0014*avgDepth*avgDepth || //PLANE_AVG_FIT_ERROR_THRESHOLD_QUADTREE_SPLIT * avgDepth * scalef ||
                                    (quad.area() <= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT)) {
                                //&& plane3.d > 0 && abs(plane3.x) < 0.99 && abs(plane3.y) < 0.99) {
                                //std::cout << "areaA " << planeA->area() << " errorA = " 
                                //<< planeA->avgError() << " planeA = " << *planeA << std::endl;
                                cv::TesselatedPlane3f tplane(plane3, numLabels++);
                                tplane.addQuad(quad);
                                sg::Plane<float>::Ptr planePtr(new sg::Plane<float>(tplane));
                                bool checkPlanar = setPlaneUVCoords2(planePtr, rgbd_img, quad);
                                checkPlanar = true;
                                if ((checkPlanar && quad.area() > MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT) ||
                                        (quad.area() <= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT)) {
                                    // accept a block
                                    float curCoeffs[4] = {planePtr->x, planePtr->y, planePtr->z, planePtr->d};
                                    //cv::Mat curCoeffsMat(3, 1, CV_32FC1, curCoeffs);
                                    bool foundLabel = false;
                                    int label = -1;
                                    int level_index = quadTree->getLevel(); // only one level in the first search
                                    //std::cout << "FindTile level index = " << level_index << std::endl;
                                    for (int search_level_index = 0; search_level_index <= level_index; search_level_index++) {
                                        //std::cout << "FindTile search level = " << search_level_index << std::endl;
                                        QuadTreeLevel<sg::Plane<float>::Ptr>* search_level_ptr = quadTree->getQuadTreeLevel(search_level_index);
                                        int search_level_numBlocks = search_level_ptr->numElements(); // get all blocks from current level in the search_level_ptr
                                        if (search_level_numBlocks != 0) {
                                            std::unordered_map<int, sg::Plane<float>::Ptr> search_level_data = search_level_ptr->getData();
                                            std::unordered_map<int, sg::Plane<float>::Ptr>::iterator p;
                                            //for (int searchBlockIdx = 1; searchBlockIdx < search_level_numBlocks; searchBlockIdx++) {
                                            for (p = search_level_data.begin(); p != search_level_data.end(); p++) {
                                                int search_block_pos_x, search_block_pos_y;
                                                // find the tlc of current searchBlock to make its label accessible to be used later
                                                search_level_ptr->keyToXY(p->first, search_block_pos_x, search_block_pos_y);
                                                cv::Point2i tlc, brc;
                                                search_level_ptr->getCorners(search_block_pos_x, search_block_pos_y, tlc, brc);
                                                //std::cout << "block pos = " << tlc.x << ", " << tlc.y << std::endl;
                                                sg::Plane<float>::Ptr search_block_ptr = p->second;
                                                float searchCoeffs[4] = {search_block_ptr->x, search_block_ptr->y, search_block_ptr->z, search_block_ptr->d};
                                                //cv::Mat searchCoeffsMat(3, 1, CV_32FC1, searchCoeffs);
                                                float coff_err = searchCoeffs[0]*curCoeffs[0] + searchCoeffs[1]*curCoeffs[1] + searchCoeffs[2]*curCoeffs[2];
                                                float ang_err = abs(asin(1 - coff_err))*180/PI; // get the ang error of the fitted planes from two blocks
                                                //std::cout << "angular error = " << ang_err << std::endl;
                                                float d_err = abs(searchCoeffs[3] - curCoeffs[3]);
                                                if (ang_err < 5 && d_err < 0.05*scalef) {
                                                    foundLabel = true;
                                                    // extract the label of current searchBlock for label assignment later
                                                    label = img_labels.at<int>(tlc.y, tlc.x); // set labels to current block in current level (need to modify the search_level_ptr level)
                                                    // assign the accpeted label to the the latest accpted block in the img_labels
                                                    for (int col = quad.x; col < quad.x+quad.width; col++) {
                                                        for (int row = quad.y; row < quad.y+quad.height; row++) {
                                                            img_labels.at<int>(row, col) = label;
                                                        }
                                                    }
                                                    //std::cout << "current label = " << label << std::endl;
                                                    break;
                                                }
                                            }
                                        }
                                        if (foundLabel == true) {
                                            break;
                                        }
                                    }
                                    // no merge was found
                                    if (foundLabel == false) {
                                        for (int col = x_tile; col < quad.x+quad.width; col++) {
                                            for (int row = y_tile; row < quad.y+quad.height; row++) {
                                                img_labels.at<int>(row, col) = new_label;
                                            }
                                        }                                   
                                        new_label += 1;
                                    }
                                    quadTree->insert_or_assign(x_tile, y_tile, planePtr);
                                    RectWithError newQuad = quad.clone();
                                    quad.clearStatistics();
                                    planePtr->addQuad(newQuad); // error and support already known
                                    quadQueue.push(newQuad);
                                }
                            } else {
                                if (quad.area() >= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT << 2) {
                                    // mark tile coordinates for subdivision
                                    //std::cout << "recurse on quad dims = (" << quad.width << ", " << quad.height << ")" << std::endl;
                                    recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 0));
                                    recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 0));
                                    recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 1));
                                    recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 1));
                                }
                            }
                        } else {
                            // not enough samples found in tile -> stop recursions
                        }
                    } else { // one or more tile corners is a NaN depth measurement -> subdivide
                        if (quad.area() >= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT << 2) {
                            // mark tile coordinates for subdivision
                            //std::cout << "recurse on quad dims = (" << quad.width << ", " << quad.height << ")" << std::endl;
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 0));
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 0));
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 0, (y_tile << 1) + 1));
                            recurseTileVec.push_back(Point2i((x_tile << 1) + 1, (y_tile << 1) + 1));
                        }
                    }
                } // loop over tile columns
            } // loop over tile rows
            return recurseTileVec;
        }

        std::vector<cv::Point2i> SurfaceSegmentor::recursiveMergeSubdivision(ErrorSortedRectQueue& quadQueue,
                const RgbdImage& rgbd_img,
                QuadTreeLevel<sg::Plane<float>::Ptr>* quadTree,
                const std::vector<cv::Point2i>& subdivideTileVec,
                Mat& img_labels, int& numLabels,
                const QuadTreeLevel<sg::Plane<float>::Ptr>::Ptr& full_quadTree,
                int& new_label) const {
            std::vector<cv::Point2i> recurseTileVec;
            cv::Plane3f plane3;
            RectWithError quad;
            float avgDepth = 0.0;
            for (cv::Point2i tileIdx : subdivideTileVec) {
                quadTree->setRect(tileIdx.x, tileIdx.y, quad);
                //if (checkTilePlanar2(rgbd_img, quad, avgDepth, recurseTileVec, tileIdx.x, tileIdx.y) || 
                //            (quad.area() <= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT)) {
                if(1) {
                    std::vector<cv::Point3f> tile_data; // data for each tile in array of vectors
                    int numSamples = std::max((quad.width * quad.height) / 2, 3);
                    tile_data.reserve(numSamples);
                    rgbd_img.getTileData_Uniform(quad.x, quad.y,
                            quad.width, quad.height, tile_data, numSamples);
                    if (tile_data.size() > (numSamples >> 2)) {
                        cv::FitStatistics stats;
                        //std::tie(plane3, stats) = rgbd_img.fitPlaneImplicitLeastSquaresWithStats(tile_data);
                        std::tie(plane3, stats) = rgbd_img.fitPlaneExplicitLeastSquaresWithStats(tile_data);
                        quad.error = stats.error;
                        quad.noise = stats.noise;
                        quad.inliers = stats.inliers;
                        quad.outliers = stats.outliers;
                        quad.invalid = stats.invalid;
                        for (int num = 1; num < tile_data.size(); num++) {
                            avgDepth = avgDepth + tile_data[num].z;
                        }
                        avgDepth /= tile_data.size();
                        float scalef = (avgDepth < 2.5) ? 2.0f : 1.0f;
                        if (quad.error < 0.0014*avgDepth*avgDepth || // PLANE_AVG_FIT_ERROR_THRESHOLD_QUADTREE_SPLIT * avgDepth * scalef || 
                            (quad.area() <= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT)) {
                            //&& plane3.d > 0 && abs(plane3.x) < 0.99 && abs(plane3.y) < 0.99) {
                            //std::cout << "areaA " << planeA->area() << " errorA = " 
                            //<< planeA->avgError() << " planeA = " << *planeA << std::endl;
                            cv::TesselatedPlane3f tplane(plane3, numLabels++);
                            tplane.addQuad(quad);
                            sg::Plane<float>::Ptr planePtr(new sg::Plane<float>(tplane));
                            bool checkPlanar = setPlaneUVCoords2(planePtr, rgbd_img, quad);
                            checkPlanar = true;
                            if (checkPlanar && quad.area() > MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT || 
                                    (quad.area() <= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT)) {
                                // accept a block
                                float curCoeffs[4] = {planePtr->x, planePtr->y, planePtr->z, planePtr->d};
                                //cv::Mat curCoeffsMat(3, 1, CV_32FC1, curCoeffs);
                                bool foundLabel = false;
                                int label = -1;
                                int level_index = quadTree->getLevel();
                                //std::cout << "RecurseTile level index = " << level_index << std::endl;
                                for (int search_level_index = 0; search_level_index <= level_index; search_level_index++) {
                                    //std::cout << "RecurseTile search level = " << search_level_index << std::endl;
                                    QuadTreeLevel<sg::Plane<float>::Ptr>* search_level_ptr = full_quadTree->getQuadTreeLevel(search_level_index);
                                    int search_level_numBlocks = search_level_ptr->numElements(); // get all blocks from current level in the search_level_ptr
                                    if (search_level_numBlocks != 0) {
                                        std::unordered_map<int, sg::Plane<float>::Ptr> search_level_data = search_level_ptr->getData();
                                        std::unordered_map<int, sg::Plane<float>::Ptr>::iterator p;
                                        //for (int searchBlockIdx = 1; searchBlockIdx < search_level_numBlocks; searchBlockIdx++) {
                                        for (p = search_level_data.begin(); p != search_level_data.end(); p++) {
                                            //std::cout << "block pos key = " << p->first << std::endl;
                                            int search_block_pos_x, search_block_pos_y;
                                            // find the tlc of current searchBlock to make its label accessible to be used later
                                            search_level_ptr->keyToXY(p->first, search_block_pos_x, search_block_pos_y);
                                            cv::Point2i tlc, brc;
                                            search_level_ptr->getCorners(search_block_pos_x, search_block_pos_y, tlc, brc);
                                            //std::cout << "block pos = " << tlc.x << ", " << tlc.y << std::endl;
                                            sg::Plane<float>::Ptr search_block_ptr = p->second;
                                            float searchCoeffs[4] = {search_block_ptr->x, search_block_ptr->y, search_block_ptr->z, search_block_ptr->d};
                                            //cv::Mat searchCoeffsMat(3, 1, CV_32FC1, searchCoeffs);
                                            float coff_err = searchCoeffs[0]*curCoeffs[0] + searchCoeffs[1]*curCoeffs[1] + searchCoeffs[2]*curCoeffs[2];
                                            float ang_err = abs(asin(1 - coff_err))*180/PI; // get the ang error of the fitted planes from two blocks
                                            //std::cout << "angular error = " << ang_err << std::endl;
                                            float d_err = abs(searchCoeffs[3] - curCoeffs[3]);
                                            if (ang_err < 5 && d_err < 0.05*scalef) {
                                                foundLabel = true;
                                                // extract the label of current searchBlock for label assignment later
                                                label = img_labels.at<int>(tlc.y, tlc.x); // set labels to current block in current level (need to modify the search_level_ptr level)
                                                // assign the accpeted label to the the latest accpted block in the img_labels
                                                for (int col = quad.x; col < quad.x+quad.width; col++) {
                                                    for (int row = quad.y; row < quad.y+quad.height; row++) {
                                                        img_labels.at<int>(row, col) = label;
                                                    }
                                                }
                                                //std::cout << "current label = " << label << std::endl;
                                                break;
                                            }
                                        }   
                                    }
                                    if (foundLabel == true) {
                                            break;
                                    }
                                }
                                // no merge was found
                                if (foundLabel == false) {
                                    for (int col = quad.x; col < quad.x+quad.width; col++) {
                                        for (int row = quad.y; row < quad.y+quad.height; row++) {
                                            img_labels.at<int>(row, col) = new_label;
                                        }
                                    }                                   
                                    new_label += 1;
                                }
                                quadTree->insert_or_assign(tileIdx.x, tileIdx.y, planePtr);
                                RectWithError newQuad = quad.clone();
                                quad.clearStatistics();
                                planePtr->addQuad(newQuad); // error and support already known
                                quadQueue.push(newQuad);
                            }
                        } else { // tile error exceeds threshold -> subdivide                            
                            if (quad.area() >= (MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT << 2)) {
                                //std::cout << "recurse on quad dims = (" << quad.width << ", " << quad.height << ")" << std::endl;
                                recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 0, (tileIdx.y << 1) + 0));
                                recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 1, (tileIdx.y << 1) + 0));
                                recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 0, (tileIdx.y << 1) + 1));
                                recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 1, (tileIdx.y << 1) + 1));
                            }
                        }
                    } else {
                        // not enough samples found in tile -> stop recursions
                    }
                } else { // one or more tile corners is a NaN depth measurement -> subdivide
                    if (quad.area() >= MIN_QUAD_PIXEL_AREA_QUADTREE_SPLIT << 2) {
                        // mark tile coordinates for subdivision
                        //std::cout << "recurse on quad dims = (" << quad.width << ", " << quad.height << ")" << std::endl;
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 0, (tileIdx.y << 1) + 0));
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 1, (tileIdx.y << 1) + 0));
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 0, (tileIdx.y << 1) + 1));
                        recurseTileVec.push_back(Point2i((tileIdx.x << 1) + 1, (tileIdx.y << 1) + 1));
                    }
                }
            } // loop over subdivision tiles
            return recurseTileVec;
        }

    } /* namespace rgbd */
} /* namespace cv */

