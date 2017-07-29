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
                std::vector<AlgebraicSurfacePatch::Ptr>& surflets,
                std::vector<ObjectGeometry>& geometries, cv::Mat& rgb_result) const {

            std::vector<sg::Shape::Ptr> shapePtrVec;

            for (AlgebraicSurfacePatch::Ptr surfPatchPtr : surflets) {
                ObjectGeometry planeGeom;
                if (surfPatchPtr->getSurfaceType() == SurfaceType::PLANE) {
                    planeGeom.addPart(surfPatchPtr);
                    geometries.push_back(planeGeom);
                    shapePtrVec.push_back(planeGeom.getShape());
                }
            }
            //for (sg::Shape::Ptr shape_ptr : shapePtrVec) {
            //sg::Shape& shape = *shape_ptr;
            //std::cout << "shape " << shape_ptr->toString() << std::endl;
            //}
            std::cout << "geometries.size() = " << geometries.size() << std::endl;

            bool corner_found = false;
            static bool have_box;
            static sg::Box::Ptr initial_box;
            ObjectGeometry corner_geometry;

            for (auto surfPatchA_iter = surflets.begin();
                    surfPatchA_iter != surflets.end(); ++surfPatchA_iter) {
                AlgebraicSurfacePatch::Ptr surfPatchA = (*surfPatchA_iter);
                if (surfPatchA->getSurfaceType() == SurfaceType::PLANE) {
                    sg::Plane<float>::Ptr planeA = boost::static_pointer_cast<sg::Plane<float>>(surfPatchA->getShape());
                    if (surfPatchA_iter + 1 != surflets.end()) {
                        for (auto surfPatchB_iter = surfPatchA_iter + 1;
                                surfPatchB_iter != surflets.end(); ++surfPatchB_iter) {
                            AlgebraicSurfacePatch::Ptr surfPatchB = (*surfPatchB_iter);
                            if (surfPatchB->getSurfaceType() == SurfaceType::PLANE) {
                                sg::Plane<float>::Ptr planeB = boost::static_pointer_cast<sg::Plane<float>>(surfPatchB->getShape());

                                if (//planeB->avgError() < plane_error_thresh && //plane3 has no error
                                        planeA->epsilonPerpendicular(*planeB, 0.5 * cv::Plane3f::PERPENDICULAR_SIN_ANGLE_THRESHOLD)) {
                                    //std::cout << "surfABEdge = " << surfABEdge.toString() << std::endl;
                                    sg::Edge<float>::Ptr edge = boost::make_shared<sg::Edge<float>>(planeA, planeB);
                                    ObjectGeometry edgeGeom;
                                    edgeGeom.addPart(surfPatchA);
                                    edgeGeom.addPart(surfPatchB);
                                    edgeGeom.setShape(edge);
                                    edgeGeom.setSurfaceType(cv::rgbd::EDGE);
                                    geometries.push_back(edgeGeom);
                                    //goto foundEdge;
                                } /* restrict pair matches to be approximately perpendicular */
                            } /* restrict surface pairwise comparisons to planes */
                        } /* loop over candidate second match surface elements (surface pairs) */
                    } // check for second surface availability (not really necessary)
                } /* restrict surface comparisons to plane pairs */
            } /* loop over candidate surface elements */
foundEdge:
            int numCorners = 0;
            std::cout << "geometries.size() = " << geometries.size() << std::endl;
            int numTripletTests = 0;
            if (geometries.size() > 2) {
                for (std::vector<ObjectGeometry>::iterator geoA_iter = geometries.begin();
                        geoA_iter != geometries.end() - 2; ++geoA_iter) {
                    ObjectGeometry& geometryA = *geoA_iter;

                    if (geometryA.getSurfaceType() == cv::rgbd::EDGE) {
                        sg::Edge<float>::Ptr edgeA = boost::static_pointer_cast<sg::Edge<float>>(geometryA.getShape());
                        if (geoA_iter + 1 != geometries.end()) {
                            for (std::vector<ObjectGeometry>::iterator geoB_iter = geoA_iter + 1;
                                    geoB_iter != geometries.end() - 1; ++geoB_iter) {
                                ObjectGeometry& geometryB = *geoB_iter;

                                if (geometryB.getSurfaceType() == cv::rgbd::EDGE) {
                                    sg::Edge<float>::Ptr edgeB = boost::static_pointer_cast<sg::Edge<float>>(geometryB.getShape());
                                    if (std::abs(edgeA->v.dot(edgeB->v)) < .05) {
                                        //std::cout << "Approx. perpendicular edge found\n";
                                        cv::Point3f perp_vector = edgeA->v.cross(edgeB->v);
                                        if (geoB_iter + 1 != geometries.end()) {
                                            for (std::vector<ObjectGeometry>::iterator geoC_iter = geoB_iter + 1;
                                                    geoC_iter != geometries.end(); ++geoC_iter) {
                                                ObjectGeometry& geometryC = *geoC_iter;

                                                if (geometryC.getSurfaceType() == cv::rgbd::EDGE) {
                                                    sg::Edge<float>::Ptr edgeC = boost::static_pointer_cast<sg::Edge<float>>(geometryC.getShape());

                                                    float piped_volume = std::abs(edgeC->v.dot(perp_vector));
                                                    numTripletTests++;
                                                    if (piped_volume > .95) {
                                                        std::cout << "Found corner, parallelpiped volume = " << piped_volume << std::endl;
                                                        sg::Corner<float>::Ptr corner_ptr = sg::Corner<float>::create(edgeA, edgeB, edgeC);

                                                        Point2f corner_proj = rgbd_img.project(*corner_ptr);
                                                        cv::circle(rgb_result, corner_proj, 5, cv::Scalar(255, 255, 0), 3);

                                                        ObjectGeometry cornerGeom;
                                                        //cornerGeom.addPart(surfPatchA);
                                                        //cornerGeom.addPart(surfPatchB);
                                                        cornerGeom.setShape(corner_ptr);
                                                        cornerGeom.setSurfaceType(cv::rgbd::CORNER);
                                                        geometries.push_back(cornerGeom);
                                                        numCorners++;
                                                        if (numCorners > 5) {
                                                            goto foundCorner;
                                                        }
                                                    } /* restrict triplet matches to be approximately perpendicular */
                                                } /* restrict triplet matches comparisons to Edges */
                                            } /* loop over candidate third surface elements (triplets of Edges) */
                                        } // check for third surface availability
                                    } /* restrict pair matches to be approximately perpendicular */
                                } /* restrict surface comparisons to plane pairs (Edges) */
                            } /* loop over candidate second match surface elements (pairs of Edges) */
                        } // check for second surface availability 
                    } /* restrict surface comparisons to plane pairs (Edges) */
                } /* loop over candidate surface elements */
            } // check for second and third surface availability 
foundCorner:
            std::cout << "Made " << numTripletTests << " tests for corners." << std::endl;
            if (geometries.size() > 6) {
                for (int idx = 5; idx > 0; --idx) {
                    sg::Corner<float>::Ptr cornerPtr = boost::dynamic_pointer_cast<sg::Corner<float>>(geometries[geometries.size() - idx].getShape());
                    if (cornerPtr) {
                        std::string shapeStr = cornerPtr->isConvex() ? "convex" : "concave";
                        std::cout << "Corner is " << shapeStr << std::endl;
                        sg::CornerType id = sg::Box::getCameraFrameCornerType(cornerPtr);
                        std::cout << "CornerType = " << sg::cornerTypeToString[id] << std::endl;                        
                    }
                }
            }
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
            for (ObjectGeometry& geom : geometries) {
                sg::Shape& shape = *geom.getShape();
                std::vector<cv::Vec3f> pts = shape.generateCoords();
                std::vector<int> triIdxList = shape.generateCoordIndices();
                for (int triIdxs : triIdxList) {
                    geom.verts.push_back(pts[triIdxs]);
                }
                std::vector<cv::Vec3f> norms = shape.generateNormals();
                std::vector<int> triNormalIdxList = shape.generateNormalCoordIndices();
                for (int triNormalIdxs : triNormalIdxList) {
                    geom.normals.push_back(norms[triNormalIdxs]);
                }
                std::vector<cv::Vec3f> colors = shape.generateColorCoords();
                std::vector<int> triColorIdxList = shape.generateColorCoordIndices();
                for (int triColorIdxs : triColorIdxList) {
                    geom.colors.push_back(colors[triColorIdxs]);
                }
                //geometries.push_back(geom);
            }
        }

        void SurfaceDescriptorMatcher::match(std::vector<ObjectGeometry> queryDescriptors,
                std::vector<ObjectGeometry> trainDescriptors,
                std::vector<cv::DMatch>& matches, cv::Mat mask) {

        }
    } /* namespace rgbd */
} /* namespace cv */