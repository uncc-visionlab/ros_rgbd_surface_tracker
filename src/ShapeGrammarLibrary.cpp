/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <vector>
#include <algorithm>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_datatypes.hpp>

namespace sg {

    std::map<CornerType, const char*> cornerTypeToString = {
        {CornerType::UNKNOWN, "UNKNOWN"},
        {CornerType::BACK_BOTTOM_RIGHT, "BACK_BOTTOM_RIGHT"},
        {CornerType::BACK_BOTTOM_LEFT, "BACK_BOTTOM_LEFT"},
        {CornerType::BACK_TOP_RIGHT, "BACK_TOP_RIGHT"},
        {CornerType::BACK_TOP_LEFT, "BACK_TOP_LEFT"},
        {CornerType::FRONT_BOTTOM_RIGHT, "FRONT_BOTTOM_RIGHT"},
        {CornerType::FRONT_BOTTOM_LEFT, "FRONT_BOTTOM_LEFT"},
        {CornerType::FRONT_TOP_RIGHT, "FRONT_TOP_RIGHT"},
        {CornerType::FRONT_TOP_LEFT, "FRONT_TOP_LEFT"}
    };
    // compute vertices

    std::vector<cv::Vec3f> Box::generateCoords() {
        std::vector<cv::Vec3f> pts = {
            cv::Vec3f(-dims.x / 2, -dims.y / 2, -dims.z / 2),
            cv::Vec3f(dims.x / 2, -dims.y / 2, -dims.z / 2),
            cv::Vec3f(dims.x / 2, dims.y / 2, -dims.z / 2),
            cv::Vec3f(-dims.x / 2, dims.y / 2, -dims.z / 2),
            cv::Vec3f(-dims.x / 2, -dims.y / 2, dims.z / 2),
            cv::Vec3f(dims.x / 2, -dims.y / 2, dims.z / 2),
            cv::Vec3f(dims.x / 2, dims.y / 2, dims.z / 2),
            cv::Vec3f(-dims.x / 2, dims.y / 2, dims.z / 2)
        };
        for (int idx = 0; idx < pts.size(); ++idx) {
            pose.transformInPlace(pts[idx]);
        }
        return pts;
    }

    // compute triangle surfaces

    std::vector<int> Box::generateCoordIndices() {
        std::vector<int> triIdxs = {
            0, 5, 4,
            5, 0, 1,
            1, 6, 5,
            6, 1, 2,
            2, 7, 6,
            7, 2, 3,
            3, 4, 7,
            4, 3, 0,
            4, 6, 7,
            6, 4, 5,
            1, 3, 2,
            3, 1, 0
        };
        return triIdxs;
    }

    std::vector<cv::Vec3f> Box::generateNormals() {
        std::vector<cv::Vec3f> norms = {
            cv::Vec3f(0, -1, 0),
            cv::Vec3f(1, 0, 0),
            cv::Vec3f(0, 1, 0),
            cv::Vec3f(-1, 0, 0),
            cv::Vec3f(0, 0, 1),
            cv::Vec3f(0, 0, -1)
        };
        for (int idx = 0; idx < norms.size(); ++idx) {
            pose.rotateInPlace(norms[idx]);
        }
        return norms;
    }

    std::vector<int> Box::generateNormalCoordIndices() {
        std::vector<int> trinormIdxs = {
            0, 0, 0,
            0, 0, 0,
            1, 1, 1,
            1, 1, 1,
            2, 2, 2,
            2, 2, 2,
            3, 3, 3,
            3, 3, 3,
            4, 4, 4,
            4, 4, 4,
            5, 5, 5,
            5, 5, 5
        };
        return trinormIdxs;
    }

    std::vector<cv::Vec3f> Box::generateColorCoords() {
        std::vector<cv::Vec3f> colors = {
            cv::Point3f(1.0, 0.0, 0.0),
            cv::Point3f(0.0, 1.0, 0.0),
            cv::Point3f(0.0, 0.0, 1.0),
            cv::Point3f(1.0, 0.3, 0.0),
            cv::Point3f(0.0, 1.0, 0.3),
            cv::Point3f(0.3, 0.0, 1.0)
        };
        return colors;
    }

    std::vector<int> Box::generateColorCoordIndices() {
        std::vector<int> tricolIdxs = {
            0, 0, 0,
            0, 0, 0,
            1, 1, 1,
            1, 1, 1,
            3, 3, 3, // face opposite red
            3, 3, 3, // face opposite red
            4, 4, 4, // face opposite green
            4, 4, 4, // face opposite green
            2, 2, 2,
            2, 2, 2,
            5, 5, 5, // face opposite blue
            5, 5, 5 // face opposite blue
        };
        return tricolIdxs;
    }

//    CornerType Box::getCameraFrameCornerType(sg::Corner<float>::Ptr corner_ptr) {
//        std::vector<cv::Vec3f> axes(3);
//        corner_ptr->getXYZAxes(axes);
//        //std::cout << "X-axis = " << axes[0] << std::endl;
//        //std::cout << "Y-axis = " << axes[1] << std::endl;
//        //std::cout << "Z-axis = " << axes[2] << std::endl;
//        int idVal = (axes[0][0] > 0) | ((axes[1][1] > 0) << 1) | ((axes[2][2] > 0) << 2);
//        //std::cout << "idVal = " << idVal << std::endl;
//        CornerType id = CornerType(idVal);
//        return id;
//    }

//    std::vector<cv::rgbd::ObjectGeometry::Ptr> Box::getCorners() {
//        std::vector<cv::rgbd::ObjectGeometry::Ptr> geomVec;
//        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> faceVec;
//        std::vector<cv::Vec3f> pts = generateCoords();
//        std::vector<cv::Plane3f> planes(6);
//        planes[0] = cv::Plane3f(pts[0], pts[1], pts[2]); // FRONT
//        planes[1] = cv::Plane3f(pts[4], pts[7], pts[6]); // BACK
//        planes[2] = cv::Plane3f(pts[4], pts[0], pts[3]); // RIGHT
//        planes[3] = cv::Plane3f(pts[6], pts[2], pts[1]); // LEFT
//        planes[4] = cv::Plane3f(pts[7], pts[3], pts[2]); // TOP
//        planes[5] = cv::Plane3f(pts[1], pts[0], pts[4]); // BOTTOM
//        cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
//        for (int idx = 0; idx < 6; ++idx) {
//            faceVec.push_back(cv::rgbd::AlgebraicSurfacePatch::create(cv::TesselatedPlane3f(planes[idx])));
//        }
//        std::vector<cv::Vec3i> cornerIdxs(8);
//        cornerIdxs[0] = cv::Vec3i(4, 1, 2); // TOP, BACK, RIGHT
//        cornerIdxs[1] = cv::Vec3i(4, 1, 3); // TOP, BACK, LEFT
//        cornerIdxs[2] = cv::Vec3i(4, 0, 2); // TOP, FRONT, RIGHT
//        cornerIdxs[3] = cv::Vec3i(4, 0, 3); // TOP, FRONT, LEFT
//        cornerIdxs[4] = cv::Vec3i(5, 1, 2); // BOTTOM, BACK, RIGHT
//        cornerIdxs[5] = cv::Vec3i(5, 1, 3); // BOTTOM, BACK, LEFT
//        cornerIdxs[6] = cv::Vec3i(5, 0, 2); // BOTTOM, FRONT, RIGHT
//        cornerIdxs[7] = cv::Vec3i(5, 0, 3); // BOTTOM, FRONT, LEFT
//        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> cornerGeom(3);
//        for (int idx = 0; idx < 8; ++idx) {
//            cornerGeom[0] = faceVec[cornerIdxs[idx][0]];
//            cornerGeom[1] = faceVec[cornerIdxs[idx][1]];
//            cornerGeom[2] = faceVec[cornerIdxs[idx][2]];
//            cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
//            for (int planeIdx = 0; planeIdx < 3; ++planeIdx) {
//                geomPtr->addPart(cornerGeom[planeIdx]);
//            }
//            geomVec.push_back(geomPtr);
//        }
//        return geomVec;
//    }
//
//    std::vector<cv::rgbd::ObjectGeometry::Ptr> Box::getEdges() {
//        std::vector<cv::rgbd::ObjectGeometry::Ptr> geomVec;
//        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> faceVec;
//        std::vector<cv::Vec3f> pts = generateCoords();
//        std::vector<cv::Plane3f> planes(6);
//        planes[0] = cv::Plane3f(pts[0], pts[1], pts[2]); // FRONT
//        planes[1] = cv::Plane3f(pts[4], pts[7], pts[6]); // BACK
//        planes[2] = cv::Plane3f(pts[4], pts[0], pts[3]); // RIGHT
//        planes[3] = cv::Plane3f(pts[6], pts[2], pts[1]); // LEFT
//        planes[4] = cv::Plane3f(pts[7], pts[3], pts[2]); // TOP
//        planes[5] = cv::Plane3f(pts[1], pts[0], pts[4]); // BOTTOM
//        cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
//        for (int idx = 0; idx < 6; ++idx) {
//            faceVec.push_back(cv::rgbd::AlgebraicSurfacePatch::create(cv::TesselatedPlane3f(planes[idx])));
//        }
//        std::vector<cv::Vec2i> edgeIdxs(12);
//        edgeIdxs[0] = cv::Vec2i(0, 2);
//        edgeIdxs[1] = cv::Vec2i(0, 3);
//        edgeIdxs[2] = cv::Vec2i(0, 4);
//        edgeIdxs[3] = cv::Vec2i(0, 5);
//        edgeIdxs[4] = cv::Vec2i(1, 2);
//        edgeIdxs[5] = cv::Vec2i(1, 3);
//        edgeIdxs[6] = cv::Vec2i(1, 4);
//        edgeIdxs[7] = cv::Vec2i(1, 5);
//        edgeIdxs[8] = cv::Vec2i(2, 4);
//        edgeIdxs[9] = cv::Vec2i(2, 5);
//        edgeIdxs[10] = cv::Vec2i(3, 4);
//        edgeIdxs[11] = cv::Vec2i(3, 5);
//        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> edgeGeom(2);
//        for (int idx = 0; idx < 12; ++idx) {
//            edgeGeom[0] = faceVec[edgeIdxs[idx][0]];
//            edgeGeom[1] = faceVec[edgeIdxs[idx][1]];
//            cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
//            for (int planeIdx = 0; planeIdx < 2; ++planeIdx) {
//                geomPtr->addPart(edgeGeom[planeIdx]);
//            }
//            geomVec.push_back(geomPtr);
//        }
//        return geomVec;
//    }
//
//    std::vector<cv::rgbd::ObjectGeometry::Ptr> Box::getPlanes() {
//        std::vector<cv::rgbd::ObjectGeometry::Ptr> geomVec;
//        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> faceVec;
//        std::vector<cv::Vec3f> pts = generateCoords();
//        std::vector<cv::Plane3f> planes(6);
//        planes[0] = cv::Plane3f(pts[0], pts[1], pts[2]); // FRONT
//        planes[1] = cv::Plane3f(pts[4], pts[7], pts[6]); // BACK
//        planes[2] = cv::Plane3f(pts[4], pts[0], pts[3]); // RIGHT
//        planes[3] = cv::Plane3f(pts[6], pts[2], pts[1]); // LEFT
//        planes[4] = cv::Plane3f(pts[7], pts[3], pts[2]); // TOP
//        planes[5] = cv::Plane3f(pts[1], pts[0], pts[4]); // BOTTOM
//        cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
//        for (int idx = 0; idx < 6; ++idx) {
//            faceVec.push_back(cv::rgbd::AlgebraicSurfacePatch::create(cv::TesselatedPlane3f(planes[idx])));
//        }
//        for (int idx = 0; idx < 6; ++idx) {
//            cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
//            geomPtr->addPart(faceVec[idx]);
//            geomVec.push_back(geomPtr);
//        }
//        return geomVec;
//    }

    std::vector<cv::Vec3f> Cylinder::generateCoords(int N) {
        double x, z;
        std::vector<cv::Vec3f> pts(N * 2 + 2);
        for (int i = 0; i < N; ++i) {
            x = r * std::cos(2 * CV_PI * i / N);
            z = -r * std::sin(2 * CV_PI * i / N);
            pts[i] = cv::Vec3f(x, h / 2, z);
            pts[i + N + 1] = cv::Vec3f(x, -h / 2, z);
        }
        pts[N] = cv::Vec3f(0, h / 2, 0);
        pts[2 * N + 1] = cv::Vec3f(0, -h / 2, 0);
        for (int idx = 0; idx < pts.size(); ++idx) {
            pose.transformInPlace(pts[idx]);
        }
        return pts;
    }

    std::vector<int> Cylinder::generateCoordIndices(int N) {
        std::vector<int> tris; // reserve(N * 4 * 3);
        for (int i = 0; i < N - 1; ++i) {
            tris.insert(tris.end(),{N, i + 1, i});
            tris.insert(tris.end(),{2 * N + 1, N + 1 + i, N + 2 + i});
            tris.insert(tris.end(),{N + i + 1, i + 1, N + 2 + i});
            tris.insert(tris.end(),{i + 1, N + i + 1, i});
        }
        tris.insert(tris.end(),{N, 0, N - 1});
        tris.insert(tris.end(),{2 * N + 1, 2 * N, N + 1});
        tris.insert(tris.end(),{0, 2 * N, N - 1});
        tris.insert(tris.end(),{2 * N, 0, N + 1});
        return tris;
    }

    std::vector<cv::Vec3f> Cylinder::generateNormals() {
        double x, z;
        static int N = DEFAULT_RESOLUTION;
        std::vector<cv::Vec3f> norms(N + 2);
        for (int i = 0; i < N; ++i) {
            x = std::cos(2 * CV_PI * i / N);
            z = std::sin(2 * CV_PI * i / N);
            norms[i] = cv::Vec3f(x, 0, z);
        }
        norms[N] = cv::Vec3f(0, 1, 0);
        norms[N + 1] = cv::Vec3f(0, -1, 0);
        for (int idx = 0; idx < norms.size(); ++idx) {
            pose.rotateInPlace(norms[idx]);
        }
        return norms;
    }

    std::vector<int> Cylinder::generateNormalCoordIndices() {
        static int N = DEFAULT_RESOLUTION;
        std::vector<int> norms; // reserve(N * 4 * 3);
        for (int i = 0; i < N - 1; ++i) {
            norms.insert(norms.end(),{N, N, N});
            norms.insert(norms.end(),{N + 1, N + 1, N + 1});
            norms.insert(norms.end(),{i, i + 1, 1 + i});
            norms.insert(norms.end(),{i + 1, i, i});
        }
        norms.insert(norms.end(),{N, N, N});
        norms.insert(norms.end(),{N + 1, N + 1, N + 1});
        norms.insert(norms.end(),{0, N - 1, N - 1});
        norms.insert(norms.end(),{N - 1, 0, N - 1});
        return norms;
    }

    std::vector<cv::Vec3f> Cylinder::generateColorCoords() {
        static int N = DEFAULT_RESOLUTION;
        std::vector<cv::Vec3f> colors = {
            cv::Vec3f(1, 0, 0),
            cv::Vec3f(0, 1, 0),
            cv::Vec3f(0, 0, 1)
        };
        return colors;
    }

    std::vector<int> Cylinder::generateColorCoordIndices() {
        static int N = DEFAULT_RESOLUTION;
        std::vector<int> colorIdxs; //reserve(N * 4 * 3);
        for (int i = 0; i < N - 1; ++i) {
            colorIdxs.insert(colorIdxs.end(),{1, 1, 1});
            colorIdxs.insert(colorIdxs.end(),{2, 2, 2});
            colorIdxs.insert(colorIdxs.end(),{0, 0, 0});
            colorIdxs.insert(colorIdxs.end(),{0, 0, 0});
        }
        colorIdxs.insert(colorIdxs.end(),{1, 1, 1});
        colorIdxs.insert(colorIdxs.end(),{2, 2, 2});
        colorIdxs.insert(colorIdxs.end(),{0, 0, 0});
        colorIdxs.insert(colorIdxs.end(),{0, 0, 0});
        return colorIdxs;
    }
} /* namespace sg */