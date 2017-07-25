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

            bool corner_found = false;
            static bool have_box;
            static sg::Box::Ptr initial_box;
            ObjectGeometry corner_geometry;

            //if (have_box) {
            // estimate TF from corners
            //shapePtrVec.push_back(initial_box);
            //}
            for (auto surfPatchA_iter = surflets.begin();
                    surfPatchA_iter != surflets.end(); ++surfPatchA_iter) {
                AlgebraicSurfacePatch::Ptr surfPatchA = (*surfPatchA_iter);
                if (corner_found)
                    break;
                if (surfPatchA->getSurfaceType() == SurfaceType::PLANE) {
                    sg::Plane::Ptr planeA = boost::static_pointer_cast<sg::Plane>(surfPatchA->getShape());
                    if (surfPatchA_iter + 1 != surflets.end()) {
                        for (auto surfPatchB_iter = surfPatchA_iter + 1;
                                surfPatchB_iter != surflets.end(); ++surfPatchB_iter) {
                            AlgebraicSurfacePatch::Ptr surfPatchB = (*surfPatchB_iter);
                            if (corner_found)
                                break;

                            if (surfPatchB->getSurfaceType() == SurfaceType::PLANE) {
                                sg::Plane::Ptr planeB = boost::static_pointer_cast<sg::Plane>(surfPatchB->getShape());

                                if (//planeB->avgError() < plane_error_thresh && //plane3 has no error
                                        planeA->epsilonPerpendicular(*planeB, 2 * cv::Plane3f::PERPENDICULAR_SIN_ANGLE_THRESHOLD)) {
                                    //std::cout << "surfABEdge = " << surfABEdge.toString() << std::endl;
                                    sg::Edge::Ptr edge = boost::make_shared<sg::Edge>(planeA, planeB);
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

            for (auto surfPatchA_iter = surflets.begin();
                    surfPatchA_iter != surflets.end(); ++surfPatchA_iter) {
                AlgebraicSurfacePatch::Ptr surfPatchA = (*surfPatchA_iter);
                if (corner_found)
                    break;
                if (surfPatchA->getSurfaceType() == SurfaceType::PLANE) {
                    cv::Plane3f::Ptr planeA = boost::static_pointer_cast<sg::Plane>(surfPatchA->getShape());
                    if (surfPatchA_iter + 1 != surflets.end()) {
                        for (auto surfPatchB_iter = surfPatchA_iter + 1;
                                surfPatchB_iter != surflets.end(); ++surfPatchB_iter) {
                            AlgebraicSurfacePatch::Ptr surfPatchB = (*surfPatchB_iter);
                            if (corner_found)
                                break;

                            if (surfPatchB->getSurfaceType() == SurfaceType::PLANE) {
                                cv::Plane3f::Ptr planeB = boost::static_pointer_cast<sg::Plane>(surfPatchB->getShape());

                                if (//planeB->avgError() < plane_error_thresh && //plane3 has no error
                                        planeA->epsilonPerpendicular(*planeB, 2 * cv::Plane3f::PERPENDICULAR_SIN_ANGLE_THRESHOLD)) {
                                    //                                
                                    cv::Point3f perp_vector = planeA->cross(*planeB);

                                    // look ahead for plane perpendicular to edge
                                    if (surfPatchB_iter + 1 != surflets.end()) {
                                        for (auto surfPatchC_iter = surfPatchB_iter + 1;
                                                surfPatchC_iter != surflets.end(); ++surfPatchC_iter) {
                                            AlgebraicSurfacePatch::Ptr surfPatchC = (*surfPatchC_iter);
                                            if (surfPatchC->getSurfaceType() == SurfaceType::PLANE) {
                                                cv::Plane3f::Ptr planeC = boost::static_pointer_cast<sg::Plane>(surfPatchC->getShape());

                                                float corner_certainty = std::abs(planeC->dot(perp_vector));

                                                if (corner_certainty > .9) {

                                                    corner_found = true;

                                                    corner_geometry.addPart(surfPatchA);
                                                    corner_geometry.addPart(surfPatchB);
                                                    corner_geometry.addPart(surfPatchC);

                                                    // compute corner point

                                                    CornerSurfaceProduct<float> corn;
                                                    corn.addSubSurface(boost::make_shared<PlanarSurface<float>>(Eigen::RowVector4f(planeA->d, planeA->x, planeA->y, planeA->z)));
                                                    corn.addSubSurface(boost::make_shared<PlanarSurface<float>>(Eigen::RowVector4f(planeB->d, planeB->x, planeB->y, planeB->z)));
                                                    corn.addSubSurface(boost::make_shared<PlanarSurface<float>>(Eigen::RowVector4f(planeC->d, planeC->x, planeC->y, planeC->z)));
                                                    Eigen::RowVector3f corner_pt_eigen = corn.getPoint();
                                                    cv::Point3f corner_pt(corner_pt_eigen(0), corner_pt_eigen(1), corner_pt_eigen(2));

                                                    //                                                std::cout << "planeA:" << planeA->toString() << "\n";
                                                    //                                                std::cout << "planeB:" << planeB->toString() << "\n";
                                                    //                                                std::cout << "planeC:" << planeC->toString() << "\n";
                                                    //std::cout << "Corner pt: " << corner_pt << "\n";
                                                    //std::cout << "|Normals|: " << corner_certainty << "\n";
                                                    //std::cout << "Evaluation: " << corn.evaluate(corner_pt_eigen) << "\n";
                                                    Point2f corner_proj = rgbd_img.project(corner_pt);
                                                    cv::circle(rgb_result, corner_proj, 5, cv::Scalar(255, 255, 0), 3);

                                                    if (!have_box) {

                                                        // assuming concave

                                                        float length = .5;
                                                        float width = .5;
                                                        float height = .5;
                                                        //allocate memory for a 4x4 float matrix
                                                        cv::Mat cvTransform(4, 4, CV_32FC1);
                                                        //directly use the buffer allocated by OpenCV
                                                        Eigen::Map<Eigen::Matrix4f> eigenTransformMap(cvTransform.ptr<float>(0, 0));
                                                        Eigen::Matrix4f eigenTransform = Eigen::Matrix4f::Zero(4, 4);
                                                        std::vector<cv::Plane3f::Ptr> moving_planes(3);
                                                        std::vector<cv::Plane3f::Ptr> fixed_planes(3);
                                                        //moving_planes[0] = boost::make_shared<cv::Plane3f>(0, 0, 1, -0.0);
                                                        //moving_planes[1] = boost::make_shared<cv::Plane3f>(-1, 0, 0, -0.0);
                                                        //moving_planes[2] = boost::make_shared<cv::Plane3f>(0, 1, 0, -0.0);
                                                        moving_planes[0] = boost::make_shared<cv::Plane3f>(1, 0, 0, -0.0);
                                                        moving_planes[1] = boost::make_shared<cv::Plane3f>(0, 1, 0, -0.0);
                                                        moving_planes[2] = boost::make_shared<cv::Plane3f>(0, 0, -1, -0.0);
                                                        fixed_planes[0] = planeA;
                                                        fixed_planes[1] = planeB;
                                                        fixed_planes[2] = planeC;
                                                        planeListAlignmentCV(moving_planes,
                                                                fixed_planes, eigenTransform);
                                                        //std::cout << "eigen mat = " << eigenTransform << std::endl;
                                                        eigenTransformMap = eigenTransform;
                                                        //std::cout << "eigen mat mapped = " << eigenTransformMap << std::endl;
                                                        cv::transpose(cvTransform, cvTransform);
                                                        //std::cout << "cvTransform = " << cvTransform << std::endl;
                                                        cv::Vec3f normA((*planeA).x, (*planeA).y, (*planeA).z);
                                                        cv::Vec3f normB((*planeB).x, (*planeB).y, (*planeB).z);
                                                        //cv::Vec3f normA(1.0/sqrt(2),0,1.0/sqrt(2));
                                                        //cv::Vec3f normB(1.0/sqrt(2),-1.0/sqrt(2),0);
                                                        normB -= normB.dot(normA) * normA;
                                                        normA *= 1.0 / std::sqrt(normA.dot(normA));
                                                        normB *= 1.0 / std::sqrt(normB.dot(normB));
                                                        cv::Vec3f normC = normB.cross(normA);
                                                        cv::Mat Rmat(3, 3, CV_32F);
                                                        float *r0matptr = Rmat.ptr<float>(0, 0);
                                                        float *r1matptr = Rmat.ptr<float>(1, 0);
                                                        float *r2matptr = Rmat.ptr<float>(2, 0);
                                                        for (int idx = 0; idx < 3; ++idx) {
                                                            *r0matptr++ = normA[idx];
                                                            *r1matptr++ = normB[idx];
                                                            *r2matptr++ = normC[idx];
                                                        }
                                                        //std::cout << "Rmat = " << Rmat << std::endl;
                                                        //std::cout << cv::determinant(Rmat) << std::endl;
                                                        cv::Vec3f tVec = cvTransform(cv::Rect(3, 0, 1, 3));
                                                        // chose the front top left corner to align
                                                        // position of the Box center relative to this corner is below
                                                        //tVec[0] += 0.05f;
                                                        //tVec[0] -= 0.05f;
                                                        //tVec[0] -= 0.05f;
                                                        cv::Vec3f rVec;
                                                        cv::Mat rotMat = cvTransform(cv::Rect(0, 0, 3, 3));
                                                        cv::transpose(rotMat, rotMat);
                                                        //std::cout << "R = " << rotMat << std::endl;
                                                        //std::cout << "t = " << tVec << std::endl;
                                                        //cv::Mat tMat(3, 1, CV_32F);
                                                        //tMat = tVec;
                                                        //tMat = rotMat*tMat;
                                                        //tVec = tMat;
                                                        cv::Rodrigues(rotMat, rVec);
                                                        //initial_box = boost::make_shared<sg::Box>(cv::Vec3f(length, width, height),
                                                        //Pose(static_cast<cv::Vec3f> (corner_pt + length / 2 * (*planeA) + width / 2 * (*planeB) + height / 2 * (*planeC)), rVec));
                                                        initial_box = boost::make_shared<sg::Box>(cv::Vec3f(.3, .3, .3),
                                                                Pose(tVec, rVec));

                                                        shapePtrVec.push_back(initial_box);
                                                        ObjectGeometry cornerGeom;
                                                        cornerGeom.addPart(surfPatchA);
                                                        cornerGeom.addPart(surfPatchB);
                                                        cornerGeom.addPart(surfPatchC);
                                                        cornerGeom.setShape(initial_box);
                                                        cornerGeom.setSurfaceType(cv::rgbd::BOX);
                                                        geometries.push_back(cornerGeom);

                                                        //have_box = true;
                                                    } /* make a box at this detected corner */

                                                    break;
                                                } /* restrict triplet matches to be approximately perpendicular */
                                            } /* restrict triplet matches comparisons to planes */
                                        } /* loop over candidate third surface elements (plane triplets) */
                                    } // check for third surface availability (not really necessary)
                                } /* restrict pair matches to be approximately perpendicular */
                            } /* restrict surface pairwise comparisons to planes */
                        } /* loop over candidate second match surface elements (surface pairs) */
                    } // check for second surface availability (not really necessary)
                } /* restrict surface comparisons to plane pairs */
            } /* loop over candidate surface elements */


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