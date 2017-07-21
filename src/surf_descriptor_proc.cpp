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
                std::vector<ObjectGeometry>& geometries) const {

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
            //                for (int indexB = indexA + 1; indexB < planeList.size(); indexB++) {
            //                    TesselatedPlane3f::Ptr& planeB = planeList[indexB ];
            //                    if (planeA->epsilonPerpendicular(*planeB, 2 * Plane3f::PERPENDICULAR_SIN_ANGLE_THRESHOLD)) {
            //                        //                    std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() << " planeA = " << *planeA
            //                        //                            << " areaB " << planeB->area() << " errorB = " << planeB->avgError() << " planeB = " << *planeB
            //                        //                            << " cosangle = " << planeA->cosDihedralAngle(*planeB)
            //                        //                            << std::endl;
            //                        edgeList.push_back(Edge3f(*planeA, *planeB));
            //                    }
            //                }


            bool corner_found = false;
            static bool have_box;
            static sg::Box::Ptr initial_box;
            ObjectGeometry corner_geometry;

            for (auto shapeA_iter = surflets.begin(); shapeA_iter != surflets.end() - 1; ++shapeA_iter) {

                if (corner_found)
                    break;

                if ((*shapeA_iter)->getSurfaceType() == SurfaceType::PLANE) {


                    cv::Plane3f::Ptr planeA = boost::static_pointer_cast<sg::Plane>((*shapeA_iter)->getShape());

                    for (auto shapeB_iter = shapeA_iter + 1; shapeB_iter != surflets.end(); ++shapeB_iter) {

                        if (corner_found)
                            break;

                        if ((*shapeB_iter)->getSurfaceType() == SurfaceType::PLANE) {
                            cv::Plane3f::Ptr planeB = boost::static_pointer_cast<sg::Plane>((*shapeB_iter)->getShape());

                            if (//planeB->avgError() < plane_error_thresh && //plane3 has no error
                                    planeA->epsilonPerpendicular(*planeB, 2 * cv::Plane3f::PERPENDICULAR_SIN_ANGLE_THRESHOLD)) {
                                //                                
                                cv::Point3f perp_vector = planeA->cross(*planeB);

                                // look ahead for plane perpendicular to edge
                                if (shapeB_iter < surflets.end() - 1) {
                                    for (auto shapeC_iter = shapeB_iter + 1; shapeC_iter != surflets.end(); ++shapeC_iter) {

                                        if ((*shapeC_iter)->getSurfaceType() == SurfaceType::PLANE) {
                                            cv::Plane3f::Ptr planeC = boost::static_pointer_cast<sg::Plane>((*shapeC_iter)->getShape());
                                            
                                            float corner_certainty = std::abs(planeC->dot(perp_vector));

                                            if (corner_certainty > .9) {
                                                std::cout << "Corner detected, certainty: " << std::abs(planeC->dot(perp_vector)) << std::endl;
                                                corner_found = true;

                                                corner_geometry.addPart(*shapeA_iter);
                                                corner_geometry.addPart(*shapeB_iter);
                                                corner_geometry.addPart(*shapeC_iter);
                                                
                                                // compute corner point
                                                
                                                CornerSurfaceProduct<float> corn;
                                                corn.addSubSurface(boost::make_shared<PlanarSurface<float>>(Eigen::RowVector4f(planeA->d, planeA->x, planeA->y, planeA->z)));
                                                corn.addSubSurface(boost::make_shared<PlanarSurface<float>>(Eigen::RowVector4f(planeB->d, planeB->x, planeB->y, planeB->z)));
                                                corn.addSubSurface(boost::make_shared<PlanarSurface<float>>(Eigen::RowVector4f(planeC->d, planeC->x, planeC->y, planeC->z)));
                                                Eigen::RowVector3f corner_pt = corn.getPoint();
                                                
                                                
//                                                Eigen::Matrix<double, 3, 4> corner_coeffs_mat;
//                                                corner_coeffs_mat.row(0) << planeA->d, planeA->x, planeA->y, planeA->z;
//                                                corner_coeffs_mat.row(1) << planeB->d, planeB->x, planeB->y, planeB->z;
//                                                corner_coeffs_mat.row(2) << planeC->d, planeC->x, planeC->y, planeC->z;
//                                                
//                                                float w = (Eigen::Matrix3d() << corner_coeffs_mat.col(1), corner_coeffs_mat.col(2), corner_coeffs_mat.col(3)).finished().determinant();
//                                                float corner_x = (Eigen::Matrix3d() << corner_coeffs_mat.col(0), corner_coeffs_mat.col(2), corner_coeffs_mat.col(3)).finished().determinant()/w;
//                                                float corner_y = (Eigen::Matrix3d() << corner_coeffs_mat.col(1), corner_coeffs_mat.col(0), corner_coeffs_mat.col(3)).finished().determinant()/w;
//                                                float corner_z = (Eigen::Matrix3d() << corner_coeffs_mat.col(1), corner_coeffs_mat.col(2), corner_coeffs_mat.col(0)).finished().determinant()/w;
                                                
                                                
                                                std::cout << "planeA:" << planeA->toString() << "\n";
                                                std::cout << "planeB:" << planeB->toString() << "\n";
                                                std::cout << "planeC:" << planeC->toString() << "\n";
//                                                std::cout << "w:" << w << "\n";
//                                                std::cout << "corner pt:" << corner_x << ", " << corner_y << ", " << corner_z << "\n";
//                                                std::cout << "x_mat:\n" << (Eigen::Matrix3d() << corner_coeffs_mat.col(0), corner_coeffs_mat.col(2), corner_coeffs_mat.col(3)).finished() << "\n";
//                                                std::cout << "y_mat:\n" << (Eigen::Matrix3d() << corner_coeffs_mat.col(1), corner_coeffs_mat.col(0), corner_coeffs_mat.col(3)).finished() << "\n";
//                                                std::cout << "z_mat:\n" << (Eigen::Matrix3d() << corner_coeffs_mat.col(1), corner_coeffs_mat.col(2), corner_coeffs_mat.col(0)).finished() << "\n";
                                                std::cout << "Corner pt: " << corner_pt << "\n";
                                                std::cout << "Evaluation: " << corn.evaluate(corner_pt) << "\n";
                                                
                                                
                                                
                                                if (!have_box) {
                                                    
                                                    // assuming concave
                                                    
                                                    float assumed_length = 7.0;
                                                    float assumed_width = 7.0;
                                                    float assumed_height = 7.0;
                                                    initial_box = boost::make_shared<sg::Box>(cv::Vec3f(assumed_length, assumed_width, assumed_height),
                                                        Pose(cv::Vec3f(0, 0, 0.3794 + 0.15875 / 2), cv::Vec3f(0, 0, 0)));
                                                            
                                                    have_box = true;
                                                }

                                                break;
                                            }
                                        }
                                    }
                                }
                            }

                        }

                    }
                }
            }



            if (corner_found && have_box) {
                // estimate TF from corners
                
                

            }

            //
            //            std::vector<cv::TesselatedPlane3f::Ptr>& planeArray = quadTree.getObjects();
            //            *curPlanesPtr = planeArray;
            //            if (curPlanesPtr->size() > 0 &&
            //                    curPlanesPtr->size() == prevPlanesPtr->size()) {
            //                //        std::cout << "planes_src_tgt = [ ";
            //                Eigen::Matrix<Scalar, 3, 3> normalsCovMat = Eigen::Matrix<Scalar, 3, 3>::Zero();
            //                Eigen::Matrix<Scalar, 1, 3> deltaDt_NA = Eigen::Matrix<Scalar, 1, 3>::Zero();
            //                Eigen::Matrix<Scalar, 1, 3> deltaDt_NB = Eigen::Matrix<Scalar, 1, 3>::Zero();
            //
            //                for (int indexA = 0; indexA < planeArray.size(); indexA++) {
            //                    cv::TesselatedPlane3f::Ptr& plane_tgt = curPlanesPtr->at(indexA);
            //                    cv::TesselatedPlane3f::Ptr& plane_src = prevPlanesPtr->at(indexA);
            //                    //            std::cout << "areaA " << planeA->area() << " errorA = " << planeA->avgError() 
            //                    //                    << " planeA = " << *planeA << std::endl;
            //                    if (plane_tgt && plane_src) {
            //                        float norm = (plane_tgt->x - plane_src->x)*(plane_tgt->x - plane_src->x) +
            //                                (plane_tgt->y - plane_src->y)*(plane_tgt->y - plane_src->y)+
            //                                (plane_tgt->z - plane_src->z)*(plane_tgt->z - plane_src->z)+
            //                                (plane_tgt->d - plane_src->d)*(plane_tgt->d - plane_src->d);
            //                        if (norm < OUTLIER_THRESHOLD) {
            //                        }
            //                    }
            //                }

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
            for (sg::Shape::Ptr shape_ptr : shapePtrVec) {
                sg::Shape& shape = *shape_ptr;
                ObjectGeometry geom;
                std::vector<cv::Vec3f> pts = shape.generateCoords();
                std::vector<cv::Vec3i> triIdxList = shape.generateCoordIndices();
                for (cv::Vec3i triIdxs : triIdxList) {
                    geom.verts.push_back(pts[triIdxs[0]]);
                    geom.verts.push_back(pts[triIdxs[1]]);
                    geom.verts.push_back(pts[triIdxs[2]]);
                }
                std::vector<cv::Vec3f> norms = shape.generateNormals();
                std::vector<cv::Vec3i> triNormalIdxList = shape.generateNormalCoordIndices();
                for (cv::Vec3i triNormalIdxs : triNormalIdxList) {
                    geom.normals.push_back(norms[triNormalIdxs[0]]);
                    geom.normals.push_back(norms[triNormalIdxs[1]]);
                    geom.normals.push_back(norms[triNormalIdxs[2]]);
                }
                std::vector<cv::Vec3f> colors = shape.generateColorCoords();
                std::vector<cv::Vec3i> triColorIdxList = shape.generateColorCoordIndices();
                for (cv::Vec3i triColorIdxs : triColorIdxList) {
                    geom.colors.push_back(colors[triColorIdxs[0]]);
                    geom.colors.push_back(colors[triColorIdxs[1]]);
                    geom.colors.push_back(colors[triColorIdxs[2]]);
                }
                geometries.push_back(geom);
            }
        }

        void SurfaceDescriptorMatcher::match(std::vector<ObjectGeometry> queryDescriptors,
                std::vector<ObjectGeometry> trainDescriptors,
                std::vector<cv::DMatch>& matches, cv::Mat mask) {

        }
    } /* namespace rgbd */
} /* namespace cv */