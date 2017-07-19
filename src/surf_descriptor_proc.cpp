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
            std::vector<cv::Edge3f> edgeList;
            std::vector<cv::Corner3f::Ptr> cornerList;

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
//            sg::Box::Ptr b2ptr(boost::make_shared<sg::Box>(cv::Vec3f(1, 1, 1),
//                    Pose(cv::Vec3f(1.5, 1.5, 10), cv::Vec3f(0, 0, CV_PI / 6))));
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