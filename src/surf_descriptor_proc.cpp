/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>
#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>

namespace cv {
    namespace rgbd {

        void SurfaceDescriptorExtractor::compute(const cv::rgbd::RgbdImage& rgbd_img,
                cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>&query_shapeMap,
                int timeBudget_ms, cv::Mat& rgb_result) const {

            bool timeBudgetExpired = false;
            int64 timeBudgetTicks = timeBudget_ms * cv::getTickFrequency() / 1000;
            int64 ticksNow, ticksStart = cv::getTickCount();

            cv::Rect rootCompareWindow(-2, -2, 5, 5);
            int qX, qY, key, dLevel;
            int level = quadTree->getLevel();
            std::vector<bool> searchedLevelsVec(16, false);
            int compareLevels[] = {0, 0};
            cv::Vec3f position;
            cv::Point2f projectedPosition;
            bool showPlanes = true;
            // TODO: Algorithm never increments compareLevels[0]!
            while (!timeBudgetExpired &&
                    quadTree->numLevels() >= std::max(compareLevels[0], compareLevels[1])) {
                dLevel = compareLevels[1] - compareLevels[0];
                QuadTreeLevel<sg::Plane<float>::Ptr>* qtA = quadTree->getQuadTreeLevel(compareLevels[0]);
                QuadTreeLevel<sg::Plane<float>::Ptr>* qtB = quadTree->getQuadTreeLevel(compareLevels[1]);
                QuadTreeLevel<sg::Plane<float>::Ptr>* qts[] = {qtA, qtB};
                if (showPlanes) {
                    for (int pair_idx = 0; pair_idx < 2; ++pair_idx) {
                        if (!searchedLevelsVec[compareLevels[pair_idx]]) {
                            searchedLevelsVec[compareLevels[pair_idx]] = true;
                            std::unordered_map<int, sg::Plane<float>::Ptr> data = qts[pair_idx]->getData();
                            for (auto it = data.begin(); it != data.end(); ++it) {
                                key = it->first;
                                quadTree->keyToXY(key, qX, qY);
                                //std::cout << "Quad(" << qX << ", " << qY << ") = " 
                                //<< ((!planeA) ? "null" : (*planeA).toString()) << std::endl;
                                sg::Plane<float>::Ptr plane = it->second; // plane from quadTree
                                // add all planes detected in the pair of quad tree levels being analyzed 
                                std::vector<sg::Shape::Ptr>& shapes = query_shapeMap[cv::rgbd::SurfaceType::PLANE];
                                shapes.push_back(plane);
                            }
                        }
                    }
                }
                std::vector<sg::Edge<float>::Ptr> edgeVec;

                std::unordered_map<int, sg::Plane<float>::Ptr> dataA = qts[0]->getData();
                for (auto it = dataA.begin(); it != dataA.end(); ++it) {
                    key = it->first;
                    qts[0]->keyToXY(key, qX, qY);
                    sg::Plane<float>::Ptr planeA = it->second; // plane from first quadTree

                    for (int dX = rootCompareWindow.x; dX < rootCompareWindow.x + rootCompareWindow.width; ++dX) {
                        for (int dY = rootCompareWindow.y; dY < rootCompareWindow.y + rootCompareWindow.height; ++dY) {
                            sg::Plane<float>::Ptr* planeB_ptr = qts[1]->get((qX << dLevel) + dX, (qY << dLevel) + dY);
                            if (planeB_ptr) {
                                if (planeA->epsilonPerpendicular(**planeB_ptr, 3 * cv::Plane3f::PERPENDICULAR_SIN_ANGLE_THRESHOLD)) {
                                    sg::Edge<float>::Ptr edge = boost::make_shared<sg::Edge<float>>(planeA, *planeB_ptr);
                                    //if (shapes.size() > 1) {
                                    //    sg::Edge<float>::Ptr edgePtr = boost::static_pointer_cast<sg::Edge<float>>(*(shapes.end() - 1));
                                    //    std::cout << "edgeA = " << edge->toString() << std::endl;
                                    //    std::cout << "edgeB = " << edgePtr->toString() << std::endl;
                                    //    std::cout << "test = " << edge->sharesPlane(edgePtr) << std::endl;
                                    //}
                                    // make sure extracted element is visible
                                    edge->getPose().getTranslation(position);
                                    projectedPosition = rgbd_img.project(position);
                                    if (projectedPosition.x > 0 && projectedPosition.y > 0 &&
                                            projectedPosition.x < rgbd_img.getWidth() &&
                                            projectedPosition.y < rgbd_img.getHeight()) {
                                        edgeVec.push_back(edge);
                                        std::vector<sg::Shape::Ptr>& shapes = query_shapeMap[cv::rgbd::SurfaceType::EDGE];
                                        shapes.push_back(edge);
                                    }
                                } /* restrict pair matches to be approximately perpendicular */
                            } /* restrict surface pairwise comparisons to planes */
                        } /* loop over candidate second match surface elements (surface pairs) */
                    } /* loop over candidate surface elements */
                }

                for (auto itA = edgeVec.begin(); itA != edgeVec.end(); ++itA) {
                    sg::Edge<float>::Ptr edgeA = *itA;
                    if (itA + 1 == edgeVec.end())
                        goto corner_search_done;
                    for (auto itB = itA + 1; itB != edgeVec.end(); ++itB) {
                        if (itB + 1 == edgeVec.end())
                            goto corner_search_done;
                        sg::Edge<float>::Ptr edgeB = *itB;
                        //if (std::abs(edgeA->v.dot(edgeB->v)) < .05) {
                        // force edgeA and edgeB to come from a common plane detection
                        if (edgeB->sharesPlane(edgeA) && std::abs(edgeA->v.dot(edgeB->v)) < .25) {
                            cv::Point3f perp_vector = edgeA->v.cross(edgeB->v);
                            for (auto itC = itB + 1; itC != edgeVec.end(); ++itC) {
                                sg::Edge<float>::Ptr edgeC = *itC;
                                // force edgeC to come from one of the planes detected in either edgeA or edgeB
                                if (edgeC->sharesPlane(edgeA) || edgeC->sharesPlane(edgeB)) {
                                    float piped_volume = std::abs(edgeC->v.dot(perp_vector));
                                    if (piped_volume > .75) {
                                        //std::cout << "Found corner, parallelpiped volume = " << piped_volume << std::endl;
                                        sg::Corner<float>::Ptr corner = sg::Corner<float>::create(edgeA, edgeB, edgeC);

                                        // make sure extracted element is visible
                                        corner->getPose().getTranslation(position);
                                        projectedPosition = rgbd_img.project(position);
                                        if (projectedPosition.x > 0 && projectedPosition.y > 0 &&
                                                projectedPosition.x < rgbd_img.getWidth() &&
                                                projectedPosition.y < rgbd_img.getHeight()) {

                                            Point2f corner_proj = rgbd_img.project(*corner);
                                            cv::circle(rgb_result, corner_proj, 5, cv::Scalar(255, 255, 0), 3);

                                            if (false) {
                                                std::cout << "Corner found at " << corner << " from planes :";
                                                std::unordered_set<sg::Plane<float>::Ptr> planeSet;
                                                corner->getPlanes(planeSet);
                                                for (auto plane_iter = planeSet.begin(); plane_iter != planeSet.end(); ++plane_iter) {
                                                    cv::Vec3f position;
                                                    (*plane_iter)->getPose().getTranslation(position);
                                                    std::cout << "pos = " << position << " ";
                                                }
                                                std::cout << std::endl;
                                            }
                                            std::vector<sg::Shape::Ptr>& shapes = query_shapeMap[cv::rgbd::SurfaceType::CORNER];
                                            shapes.push_back(corner);
                                        }
                                    } /* restrict triplet matches to be approximately perpendicular */
                                }
                            } /* loop over candidate third surface elements (triplets of Edges) */
                        } /* restrict pair matches to be approximately perpendicular */
                    } /* loop over candidate second match surface elements (pairs of Edges) */
                } /* loop over candidate surface elements */
corner_search_done:

                ticksNow = cv::getTickCount();
                timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;
                if (compareLevels[0] == compareLevels[1]) {
                    compareLevels[1]++;
                } else {
                    //compareLevels[0]++;
                    compareLevels[1]++;
                }
            }
            std::cout << "Descriptor extractor time used = " << ((double) (ticksNow - ticksStart) / cv::getTickFrequency())
                    << " sec time allocated = " << ((double) timeBudgetTicks / cv::getTickFrequency()) << " sec" << std::endl;

            static bool verbose = false;
            if (verbose) {
                for (auto it = query_shapeMap.begin(); it != query_shapeMap.end(); ++it) {
                    std::cout << "Found " << (*it).second.size() << " " <<
                            cv::rgbd::surfaceTypeToString[(*it).first] << " geometries." << std::endl;
                }
            }

            // 11.25in x 8.75in x 6.25in @ depth 11 in
            //            sg::Box::Ptr b1ptr(boost::make_shared<sg::Box>(cv::Vec3f(0.28575, 0.2225, 0.15875),
            //                    Pose(cv::Vec3f(0, 0, 0.3794 + 0.15875 / 2), cv::Vec3f(0, 0, 0))));

            //sg::Box::Ptr b2ptr(boost::make_shared<sg::Box>(cv::Vec3f(1, 1, 1),
            //        Pose(cv::Vec3f(1.5, 1.5, 10), cv::Vec3f(0, 0, CV_PI / 6))));
            //sg::Box::Ptr b2ptr(boost::make_shared<sg::Box>(cv::Vec3f(1, 1, 1),
            //        Pose(cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0))));
            //std::vector<cv::rgbd::ObjectGeometry::Ptr> cornerPtrs = b2ptr->getCorners();
            //for (cv::rgbd::ObjectGeometry::Ptr cornerPtr : cornerPtrs) {
            //    std::vector<cv::Plane3f::Ptr> planePtrs = cornerPtr->getPlanes();
            //    std::cout << "corner = { ";
            //    for (cv::Plane3f::Ptr planePtr : planePtrs) {
            //        std::cout << planePtr->toString() << ", ";
            //    }
            //    std::cout << "}" << std::endl;
            //}
            //sg::Cylinder::Ptr cyl1ptr(boost::make_shared<sg::Cylinder>(0.5, 0.5,
            //        Pose(cv::Vec3f(0, 0, 3.5), cv::Vec3f(-CV_PI / 6, 0, 0))));
            //shapePtrVec.push_back(b1ptr);
            //shapePtrVec.push_back(b2ptr);
            //shapePtrVec.push_back(cyl1ptr);
        }

        float MATCH_DISTANCE_THRESHOLD = 2;

        void SurfaceDescriptorMatcher::match(const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& quadTree,
                const std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&query_shapeMap,
                const cv::QuadTree<sg::Plane<float>::Ptr>::Ptr& prev_quadTree,
                const std::unordered_map<SurfaceType, std::vector < sg::Shape::Ptr>>&train_shapeMap,
                std::vector<cv::rgbd::ShapeMatch>& matches, std::vector<sg::Shape::Ptr>& newShapes, int timeBudget_ms,
                cv::Mat& rgb_result, Pose camPose, cv::Mat mask) const {
            cv::Vec3f position;

            sg::Shape::Ptr bestMatch;
            sg::Corner<float>::Ptr cornerPtrA, cornerPtrB;
            sg::Edge<float>::Ptr edgePtrA, edgePtrB;
            sg::Plane<float>::Ptr planePtrA, planePtrB;

            cv::Matx44f camTransform = camPose.getTransform();
            //std::cout << "camTransform = " << camTransform << std::endl;

            bool timeBudgetExpired = false;
            int64 timeBudgetTicks = timeBudget_ms * cv::getTickFrequency() / 1000;
            int64 ticksNow, ticksStart = cv::getTickCount();

            ticksNow = cv::getTickCount();
            timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;

            // Search for matching edges and corners first between successive frames
            // these elements encode more information than planes
            float distance, min_distance;
            for (auto shapeType_iter = train_shapeMap.begin();
                    shapeType_iter != train_shapeMap.end(); ++shapeType_iter) {
                const std::vector<sg::Shape::Ptr>& shapeVec = (*shapeType_iter).second;
                cv::rgbd::SurfaceType shapeType = (*shapeType_iter).first;
                if (!shapeType == cv::rgbd::PLANE) { // planes are matches using sequential frame quadtrees
                    for (auto shape_iterA = shapeVec.begin(); shape_iterA != shapeVec.end(); ++shape_iterA) {
                        const sg::Shape::Ptr& shapeA = *shape_iterA;
                        shapeA->getPose().getTranslation(position);
                        if (shape_iterA + 1 != shapeVec.end()) {
                            switch (shapeType) {
                                case SurfaceType::CORNER:
                                    cornerPtrA = boost::static_pointer_cast<sg::Corner<float>>(shapeA);
                                    min_distance = std::numeric_limits<float>::infinity();
                                    for (auto shape_iterB = shape_iterA + 1; shape_iterB != shapeVec.end(); ++shape_iterB) {
                                        cornerPtrB = boost::static_pointer_cast<sg::Corner<float>>(*shape_iterB);
                                        distance = cornerPtrA->matchDistance(cornerPtrB);
                                        if (distance < min_distance) {
                                            bestMatch = cornerPtrB;
                                            min_distance = distance;
                                        }
                                    }
                                    break;
                                case SurfaceType::EDGE:
                                    edgePtrA = boost::static_pointer_cast<sg::Edge<float>>(shapeA);
                                    for (auto shape_iterB = shape_iterA + 1; shape_iterB != shapeVec.end(); ++shape_iterB) {
                                        edgePtrB = boost::static_pointer_cast<sg::Edge<float>>(*shape_iterB);
                                        distance = edgePtrA->matchDistance(edgePtrB);
                                        if (distance < min_distance) {
                                            bestMatch = edgePtrB;
                                            min_distance = distance;
                                        }
                                    }
                                    break;
                                case SurfaceType::PLANE:
                                default:
                                    planePtrA = boost::static_pointer_cast<sg::Plane<float>>(shapeA);
                                    for (auto shape_iterB = shape_iterA + 1; shape_iterB != shapeVec.end(); ++shape_iterB) {
                                        planePtrB = boost::static_pointer_cast<sg::Plane<float>>(*shape_iterB);
                                        distance = planePtrA->matchDistance(planePtrB);
                                        if (distance < min_distance) {
                                            bestMatch = planePtrB;
                                            min_distance = distance;
                                        }
                                    }
                                    break;
                            }
                            if (min_distance < MATCH_DISTANCE_THRESHOLD) {
                                cv::rgbd::ShapeMatch matchAB;
                                matchAB.query_shape = shapeA;
                                matchAB.train_shape = bestMatch;
                                matchAB.distance = min_distance;
                                matchAB.surfaceType = shapeType;
                                // TODO: matching just planes for now....
                                //matches.push_back(matchAB);
                            } else {
                                newShapes.push_back(shapeA);
                            }
                        }
                    }
                }
            }
            ticksNow = cv::getTickCount();
            timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;


            // match planes directly out of the previous frame and current frame quadTrees
            cv::Rect rootCompareWindow(-2, -2, 5, 5);
            int qX, qY, key, dLevel;
            int level = quadTree->getLevel();
            std::vector<bool> searchedLevelsVec(16, false);
            int compareLevels[] = {0, 0};
            cv::Point2f projectedPosition;
            while (!timeBudgetExpired &&
                    quadTree->numLevels() >= std::max(compareLevels[0], compareLevels[1])) {
                dLevel = compareLevels[1] - compareLevels[0];
                QuadTreeLevel<sg::Plane<float>::Ptr>* qt_prv = prev_quadTree->getQuadTreeLevel(compareLevels[0]);
                QuadTreeLevel<sg::Plane<float>::Ptr>* qt_cur = quadTree->getQuadTreeLevel(compareLevels[1]);
                QuadTreeLevel<sg::Plane<float>::Ptr>* qts[] = {qt_prv, qt_cur};

                ticksNow = cv::getTickCount();
                timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;
                if (compareLevels[0] == compareLevels[1]) {
                    compareLevels[1]++;
                } else {
                    //compareLevels[0]++;
                    compareLevels[1]++;
                }
            }

            std::cout << "Surface matcher time used = " << ((double) (ticksNow - ticksStart) / cv::getTickFrequency())
                    << " sec time allocated = " << ((double) timeBudgetTicks / cv::getTickFrequency()) << " sec" << std::endl;

            static bool verbose = false;
            if (verbose) {
                for (auto it = query_shapeMap.begin(); it != query_shapeMap.end(); ++it) {
                    std::cout << "Found " << (*it).second.size() << " " <<
                            cv::rgbd::surfaceTypeToString[(*it).first] << " geometries." << std::endl;
                }
            }


        }
    } /* namespace rgbd */
} /* namespace cv */
