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
                cv::QuadTree<sg::Plane<float>::Ptr>* quadTree,
                std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>& query_shapeMap,
                int timeBudget_ms, cv::Mat& rgb_result) const {

            bool timeBudgetExpired = false;
            int64 timeBudgetTicks = timeBudget_ms * cv::getTickFrequency() / 1000;
            int64 ticksNow, ticksStart = cv::getTickCount();

            cv::Rect rootCompareWindow(-2, -2, 5, 5);
            int qX, qY, key, dLevel;
            int level = quadTree->getLevel();
            std::vector<bool> searchedLevelsVec(16, false);
            int compareLevels[] = {0, 0};

            // TODO: Algorithm never increments compareLevels[0]!
            while (!timeBudgetExpired &&
                    quadTree->numLevels() >= std::max(compareLevels[0], compareLevels[1])) {
                dLevel = compareLevels[1] - compareLevels[0];
                QuadTreeLevel<sg::Plane<float>::Ptr>* qtA = quadTree->getQuadTreeLevel(compareLevels[0]);
                QuadTreeLevel<sg::Plane<float>::Ptr>* qtB = quadTree->getQuadTreeLevel(compareLevels[1]);
                QuadTreeLevel<sg::Plane<float>::Ptr>* qts[] = {qtA, qtB};
                for (int pair_idx = 0; pair_idx < 2; ++pair_idx) {
                    if (!searchedLevelsVec[compareLevels[pair_idx]]) {
                        searchedLevelsVec[compareLevels[pair_idx]] = true;
                        std::unordered_map<int, sg::Plane<float>::Ptr> data = qts[pair_idx]->getData();
                        for (auto it = data.begin(); it != data.end(); ++it) {
                            key = it->first;
                            quadTree->keyToXY(key, qX, qY);
                            //std::cout << "Quad(" << qX << ", " << qY << ") = " 
                            //<< ((!planeA) ? "null" : (*planeA).toString()) << std::endl;
                            sg::Plane<float>::Ptr plane = it->second; // plane from first quadTree
                            // add all plane geometries from quad tree level pair being 
                            // processed to rendered scene
                            //ObjectGeometry planeGeom;
                            //planeGeom.setShape(planeA);
                            //planeGeom.setSurfaceType(cv::rgbd::PLANE);
                            //geometries.push_back(planeGeom);
                            std::vector<sg::Shape::Ptr>& shapes = query_shapeMap[cv::rgbd::SurfaceType::PLANE];
                            shapes.push_back(plane);
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
                                if (planeA->epsilonPerpendicular(**planeB_ptr, 2 * cv::Plane3f::PERPENDICULAR_SIN_ANGLE_THRESHOLD)) {
                                    sg::Edge<float>::Ptr edge = boost::make_shared<sg::Edge<float>>(planeA, *planeB_ptr);
                                    //ObjectGeometry edgeGeom;
                                    //edgeGeom.setShape(edge);
                                    //edgeGeom.setSurfaceType(cv::rgbd::EDGE);
                                    //geometries.push_back(edgeGeom);
                                    edgeVec.push_back(edge);
                                    std::vector<sg::Shape::Ptr>& shapes = query_shapeMap[cv::rgbd::SurfaceType::EDGE];
                                    shapes.push_back(edge);
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
                        if (std::abs(edgeA->v.dot(edgeB->v)) < .05) {
                            cv::Point3f perp_vector = edgeA->v.cross(edgeB->v);
                            for (auto itC = itB + 1; itC != edgeVec.end(); ++itC) {
                                sg::Edge<float>::Ptr edgeC = *itC;
                                float piped_volume = std::abs(edgeC->v.dot(perp_vector));
                                if (piped_volume > .95) {
                                    //std::cout << "Found corner, parallelpiped volume = " << piped_volume << std::endl;
                                    sg::Corner<float>::Ptr corner = sg::Corner<float>::create(edgeA, edgeB, edgeC);

                                    Point2f corner_proj = rgbd_img.project(*corner);
                                    cv::circle(rgb_result, corner_proj, 5, cv::Scalar(255, 255, 0), 3);

                                    //ObjectGeometry cornerGeom;
                                    //cornerGeom.setShape(corner_ptr);
                                    //cornerGeom.setSurfaceType(cv::rgbd::CORNER);
                                    //geometries.push_back(cornerGeom);
                                    std::vector<sg::Shape::Ptr>& shapes = query_shapeMap[cv::rgbd::SurfaceType::CORNER];
                                    shapes.push_back(corner);
                                } /* restrict triplet matches to be approximately perpendicular */
                            } /* loop over candidate third surface elements (triplets of Edges) */
                        } /* restrict pair matches to be approximately perpendicular */
                    } /* loop over candidate second match surface elements (pairs of Edges) */
                } /* loop over candidate surface elements */
corner_search_done:

                ticksNow = cv::getTickCount();
                timeBudgetExpired = (ticksNow - ticksStart) > timeBudgetTicks;
                compareLevels[1]++;
            }
            std::cout << "Descriptor extractor time used = " << ((double) (ticksNow - ticksStart) / cv::getTickFrequency())
                    << " sec time allocated = " << ((double) timeBudgetTicks / cv::getTickFrequency()) << " sec" << std::endl;
            for (auto it = query_shapeMap.begin(); it != query_shapeMap.end(); ++it) {
                std::cout << "Found " << (*it).second.size() << " " <<
                        cv::rgbd::surfaceTypeToString[(*it).first] << " geometries." << std::endl;
            }

            //            std::cout << "Made " << numTripletTests << " tests for corners." << std::endl;
            //            if (geometries.size() > 6) {
            //                for (int idx = 5; idx > 0; --idx) {
            //                    sg::Corner<float>::Ptr cornerPtr = boost::dynamic_pointer_cast<sg::Corner<float>>(geometries[geometries.size() - idx].getShape());
            //                    if (cornerPtr) {
            //                        std::string shapeStr = cornerPtr->isConvex() ? "convex" : "concave";
            //                        std::cout << "Corner is " << shapeStr << std::endl;
            //                        sg::CornerType id = sg::Box::getCameraFrameCornerType(cornerPtr);
            //                        std::cout << "CornerType = " << sg::cornerTypeToString[id] << std::endl;
            //                    }
            //                }
            //            }
            //            cv::Mat cvTransform(4, 4, CV_32FC1);
            //            //directly use the buffer allocated by OpenCV
            //            Eigen::Map<Eigen::Matrix4f> eigenTransformMap(cvTransform.ptr<float>(0, 0));
            //            Eigen::Matrix4f eigenTransform = Eigen::Matrix4f::Zero(4, 4);
            //            std::vector<cv::Plane3f::Ptr> moving_planes(3);
            //            std::vector<cv::Plane3f::Ptr> fixed_planes(3);
            //            //moving_planes[0] = boost::make_shared<cv::Plane3f>(0, 0, 1, -0.0);
            //            //moving_planes[1] = boost::make_shared<cv::Plane3f>(-1, 0, 0, -0.0);
            //            //moving_planes[2] = boost::make_shared<cv::Plane3f>(0, 1, 0, -0.0);
            //            moving_planes[0] = boost::make_shared<cv::Plane3f>(1, 0, 0, -0.0);
            //            moving_planes[1] = boost::make_shared<cv::Plane3f>(0, 1, 0, -0.0);
            //            moving_planes[2] = boost::make_shared<cv::Plane3f>(0, 0, 1, -0.0);
            //            fixed_planes[0] = planeA;
            //            fixed_planes[1] = planeB;
            //            fixed_planes[2] = planeC;
            //            int alignment_result = planeListAlignmentCV(moving_planes,
            //                    fixed_planes, eigenTransform);
            //            //std::cout << "eigen mat = " << eigenTransform << std::endl;
            //            eigenTransformMap = eigenTransform;
            //            //std::cout << "eigen mat mapped = " << eigenTransformMap << std::endl;
            //            cv::transpose(cvTransform, cvTransform);
            //            cv::Vec3f tVec = cvTransform(cv::Rect(3, 0, 1, 3));
            //            // chose the front top left corner to align
            //            // position of the Box center relative to this corner is below
            //            //tVec[0] += 0.05f;
            //            //tVec[0] -= 0.05f;
            //            //tVec[0] -= 0.05f;
            //            cv::Vec3f rVec;
            //            cv::Mat rotMat = cvTransform(cv::Rect(0, 0, 3, 3));
            //            cv::transpose(rotMat, rotMat);
            //            //std::cout << "R = " << rotMat << std::endl;
            //            //std::cout << "t = " << tVec << std::endl;
            //            //cv::Mat tMat(3, 1, CV_32F);
            //            cv::Rodrigues(rotMat, rVec);
            //            //initial_box = boost::make_shared<sg::Box>(cv::Vec3f(length, width, height),
            //            //Pose(static_cast<cv::Vec3f> (corner_pt + length / 2 * (*planeA) + width / 2 * (*planeB) + height / 2 * (*planeC)), rVec));
            //            initial_box = boost::make_shared<sg::Box>(cv::Vec3f(.3, .3, .3),
            //                    Pose(tVec, rVec));
            //
            //            shapePtrVec.push_back(initial_box);
            //            ObjectGeometry cornerGeom;
            //            cornerGeom.addPart(surfPatchA);
            //            cornerGeom.addPart(surfPatchB);
            //            cornerGeom.addPart(surfPatchC);
            //            cornerGeom.setShape(initial_box);
            //            cornerGeom.setSurfaceType(cv::rgbd::BOX);
            //            geometries.push_back(cornerGeom);



            // 11.25in x 8.75in x 6.25in @ depth 11 in
            //            sg::Box::Ptr b1ptr(boost::make_shared<sg::Box>(cv::Vec3f(0.28575, 0.2225, 0.15875),
            //                    Pose(cv::Vec3f(0, 0, 0.3794 + 0.15875 / 2), cv::Vec3f(0, 0, 0))));

            //sg::Box::Ptr b2ptr(boost::make_shared<sg::Box>(cv::Vec3f(1, 1, 1),
            //        Pose(cv::Vec3f(1.5, 1.5, 10), cv::Vec3f(0, 0, CV_PI / 6))));
            //            sg::Box::Ptr b2ptr(boost::make_shared<sg::Box>(cv::Vec3f(1, 1, 1),
            //                    Pose(cv::Vec3f(0,0,0), cv::Vec3f(0, 0, 0))));
            //            std::vector<cv::rgbd::ObjectGeometry::Ptr> cornerPtrs = b2ptr->getCorners();
            //            for (cv::rgbd::ObjectGeometry::Ptr cornerPtr : cornerPtrs) {
            //                std::vector<cv::Plane3f::Ptr> planePtrs = cornerPtr->getPlanes();
            //                std::cout << "corner = { ";
            //                for (cv::Plane3f::Ptr planePtr : planePtrs) {
            //                    std::cout << planePtr->toString() << ", ";
            //                }
            //                std::cout << "}" << std::endl;
            //            }
            //            sg::Cylinder::Ptr cyl1ptr(boost::make_shared<sg::Cylinder>(0.5, 0.5,
            //                    Pose(cv::Vec3f(0, 0, 3.5), cv::Vec3f(-CV_PI / 6, 0, 0))));
            //            shapePtrVec.push_back(b1ptr);
            //            shapePtrVec.push_back(b2ptr);
            //            shapePtrVec.push_back(cyl1ptr);
            //for (sg::Shape::Ptr shape_ptr : shapePtrVec) {
            //    sg::Shape& shape = *shape_ptr;
        }

        void SurfaceDescriptorMatcher::match(std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>& query_shapeMap,
                std::unordered_map<SurfaceType, std::vector<sg::Shape::Ptr>>& train_shapeMap,
                std::vector<cv::DMatch>& matches, int timeBudget_ms,
                cv::Mat& rgb_result, cv::Mat mask) {
            cv::Vec3f position;
            sg::Corner<float>::Ptr cornerPtr;
            sg::Edge<float>::Ptr edgePtr;
            sg::Plane<float>::Ptr planePtr;
            for (auto shapeType_iter = train_shapeMap.begin();
                    shapeType_iter != train_shapeMap.end(); ++shapeType_iter) {
                for (auto shape_iter = (*shapeType_iter).second.begin();
                        shape_iter != (*shapeType_iter).second.end(); ++shape_iter) {
                    switch ((*shapeType_iter).first) {
                        case SurfaceType::CORNER:
                            cornerPtr = boost::static_pointer_cast<sg::Corner<float>>(*shape_iter);
                            cornerPtr->getPose().getTranslation(position);
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
    } /* namespace rgbd */
} /* namespace cv */