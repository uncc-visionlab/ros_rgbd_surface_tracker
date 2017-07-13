/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>
namespace cv {
    namespace rgbd {

        void SurfaceDescriptorExtractor::compute(const cv::rgbd::RgbdImage& rgbd_img,
                std::vector<AlgebraicSurfacePatch>& surflets,
                std::vector<ObjectGeometry>& geometries) const {

            ObjectGeometry geom;
            Box bb(cv::Vec3f(1, 1, 1),
                    Pose(cv::Vec3f(0, 0, 1), cv::Vec3f(0, 0, CV_PI / 4)));

            std::vector<cv::Vec3f> pts = bb.generateCoords();
            std::vector<cv::Vec3i> triIdxList = bb.generateCoordIndices();
            for (cv::Vec3i triIdxs : triIdxList) {
                geom.verts.push_back(pts[triIdxs[0]]);
                geom.verts.push_back(pts[triIdxs[1]]);
                geom.verts.push_back(pts[triIdxs[2]]);
            }
            std::vector<cv::Vec3f> colors = bb.generateColorCoords();
            std::vector<cv::Vec3i> triColorIdxList = bb.generateColorCoordIndices();
            for (cv::Vec3i triColorIdxs : triColorIdxList) {
                geom.colors.push_back(colors[triColorIdxs[0]]);
                geom.colors.push_back(colors[triColorIdxs[1]]);
                geom.colors.push_back(colors[triColorIdxs[2]]);
            }
            geometries.push_back(geom);
        }

        void SurfaceDescriptorMatcher::match(std::vector<ObjectGeometry> queryDescriptors,
                std::vector<ObjectGeometry> trainDescriptors,
                std::vector<cv::DMatch>& matches, cv::Mat mask) {

        }
    } /* namespace rgbd */
} /* namespace cv */