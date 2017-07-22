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

    std::vector<cv::Vec3f> Plane::generateCoords() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<cv::Vec3f> pts(uv_poly_coords.size() + 1);
        cv::Vec3f ctr_pt(0, 0, 0);
        for (int ptidx = 0; ptidx < uv_poly_coords.size(); ++ptidx) {
            pts[ptidx] = uvToXYZ(uv_poly_coords[ptidx]);
            ctr_pt += pts[ptidx];
        }
        ctr_pt *= 1.0f / uv_poly_coords.size();
        pts[uv_poly_coords.size()] = ctr_pt;
        for (int idx = 0; idx < pts.size(); ++idx) {
            pose.transformInPlace(pts[idx]);
        }
        return pts;
    }

    std::vector<cv::Vec3i> Plane::generateCoordIndices() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<cv::Vec3i> ptidxs(uv_poly_coords.size());
        int ctr_pt_idx = uv_poly_coords.size();
        for (int triIdx = 0; triIdx < ptidxs.size() - 1; ++triIdx) {
            ptidxs[triIdx] = cv::Vec3i(ctr_pt_idx, triIdx, triIdx + 1);
        }
        ptidxs[ptidxs.size() - 1] = cv::Vec3i(ctr_pt_idx, ptidxs.size() - 1, 0);
        return ptidxs;
    }

    std::vector<cv::Vec3f> Plane::generateNormals() {
        std::vector<cv::Vec3f> norms(1);
        norms[0] = cv::Vec3f(x, y, z);
        for (int idx = 0; idx < norms.size(); ++idx) {
            pose.rotateInPlace(norms[idx]);
        }
        return norms;
    }

    std::vector<cv::Vec3i> Plane::generateNormalCoordIndices() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<cv::Vec3i> normidxs(uv_poly_coords.size());
        for (int triIdx = 0; triIdx < uv_poly_coords.size(); ++triIdx) {
            normidxs[triIdx] = cv::Vec3i(0, 0, 0);
        }
        return normidxs;
    }

    std::vector<cv::Vec3f> Plane::generateColorCoords() {
        std::vector<cv::Vec3f> colors(1);
        cv::Mat hsv(1, 1, CV_32FC3, cv::Scalar(x, y, 0.7));
        cv::Mat rgb(1, 1, CV_32FC3);
        cv::cvtColor(hsv, rgb, CV_HSV2BGR);
        //colors[0] = cv::Vec3f(1.0f, 0.0f, 0.0f);
        colors[0] = hsv.at<cv::Vec3f>(0, 0);
        return colors;
    }

    std::vector<cv::Vec3i> Plane::generateColorCoordIndices() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<cv::Vec3i> coloridxs(uv_poly_coords.size());
        for (int triIdx = 0; triIdx < uv_poly_coords.size(); ++triIdx) {
            coloridxs[triIdx] = cv::Vec3i(0, 0, 0);
        }
        return coloridxs;
    }
    // compute vertices

    std::vector<cv::Vec3f> Box::generateCoords() {
        std::vector<cv::Vec3f> pts(8);
        pts[0] = cv::Vec3f(-dims.x / 2, -dims.y / 2, -dims.z / 2);
        pts[1] = cv::Vec3f(dims.x / 2, -dims.y / 2, -dims.z / 2);
        pts[2] = cv::Vec3f(dims.x / 2, dims.y / 2, -dims.z / 2);
        pts[3] = cv::Vec3f(-dims.x / 2, dims.y / 2, -dims.z / 2);
        pts[4] = cv::Vec3f(-dims.x / 2, -dims.y / 2, dims.z / 2);
        pts[5] = cv::Vec3f(dims.x / 2, -dims.y / 2, dims.z / 2);
        pts[6] = cv::Vec3f(dims.x / 2, dims.y / 2, dims.z / 2);
        pts[7] = cv::Vec3f(-dims.x / 2, dims.y / 2, dims.z / 2);
        for (int idx = 0; idx < pts.size(); ++idx) {
            pose.transformInPlace(pts[idx]);
        }
        return pts;
    }

    // compute triangle surfaces

    std::vector<cv::Vec3i> Box::generateCoordIndices() {
        std::vector<cv::Vec3i> tris(12);
        tris[0] = cv::Vec3i(0, 5, 4);
        tris[1] = cv::Vec3i(5, 0, 1);
        tris[2] = cv::Vec3i(1, 6, 5);
        tris[3] = cv::Vec3i(6, 1, 2);
        tris[4] = cv::Vec3i(2, 7, 6);
        tris[5] = cv::Vec3i(7, 2, 3);
        tris[6] = cv::Vec3i(3, 4, 7);
        tris[7] = cv::Vec3i(4, 3, 0);
        tris[8] = cv::Vec3i(4, 6, 7);
        tris[9] = cv::Vec3i(6, 4, 5);
        tris[10] = cv::Vec3i(1, 3, 2);
        tris[11] = cv::Vec3i(3, 1, 0);
        return tris;
    }

    std::vector<cv::Vec3f> Box::generateNormals() {
        std::vector<cv::Vec3f> norms(6);
        norms[0] = cv::Vec3f(0, -1, 0);
        norms[1] = cv::Vec3f(1, 0, 0);
        norms[2] = cv::Vec3f(0, 1, 0);
        norms[3] = cv::Vec3f(-1, 0, 0);
        norms[4] = cv::Vec3f(0, 0, 1);
        norms[5] = cv::Vec3f(0, 0, -1);
        for (int idx = 0; idx < norms.size(); ++idx) {
            pose.rotateInPlace(norms[idx]);
        }
        return norms;
    }

    std::vector<cv::Vec3i> Box::generateNormalCoordIndices() {
        std::vector<cv::Vec3i> tricols(12);
        tricols[0] = cv::Vec3i(0, 0, 0);
        tricols[1] = cv::Vec3i(0, 0, 0);
        tricols[2] = cv::Vec3i(1, 1, 1);
        tricols[3] = cv::Vec3i(1, 1, 1);
        tricols[4] = cv::Vec3i(2, 2, 2);
        tricols[5] = cv::Vec3i(2, 2, 2);
        tricols[6] = cv::Vec3i(3, 3, 3);
        tricols[7] = cv::Vec3i(3, 3, 3);
        tricols[8] = cv::Vec3i(4, 4, 4);
        tricols[9] = cv::Vec3i(4, 4, 4);
        tricols[10] = cv::Vec3i(5, 5, 5);
        tricols[11] = cv::Vec3i(5, 5, 5);
        return tricols;
    }

    std::vector<cv::Vec3f> Box::generateColorCoords() {
        std::vector<cv::Vec3f> colors(6);
        colors[0] = cv::Point3f(1, 0, 0);
        colors[1] = cv::Point3f(0, 1, 0);
        colors[2] = cv::Point3f(0, 0, 1);
        colors[3] = cv::Point3f(1, 0, 0); // not red
        colors[4] = cv::Point3f(0, 1, 0); // not red
        colors[5] = cv::Point3f(0, 0, 1);
        return colors;
    }

    std::vector<cv::Vec3i> Box::generateColorCoordIndices() {
        std::vector<cv::Vec3i> tricols(12);
        tricols[0] = cv::Vec3i(0, 0, 0);
        tricols[1] = cv::Vec3i(0, 0, 0);
        tricols[2] = cv::Vec3i(1, 1, 1);
        tricols[3] = cv::Vec3i(1, 1, 1);
        tricols[4] = cv::Vec3i(0, 0, 0);
        tricols[5] = cv::Vec3i(0, 0, 0);
        tricols[6] = cv::Vec3i(1, 1, 1);
        tricols[7] = cv::Vec3i(1, 1, 1);
        tricols[8] = cv::Vec3i(2, 2, 2);
        tricols[9] = cv::Vec3i(2, 2, 2);
        tricols[10] = cv::Vec3i(2, 2, 2);
        tricols[11] = cv::Vec3i(2, 2, 2);
        return tricols;
    }

    std::vector<cv::rgbd::ObjectGeometry::Ptr> Box::getCorners() {
        std::vector<cv::rgbd::ObjectGeometry::Ptr> geomVec;
        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> faceVec;
        std::vector<cv::Vec3f> pts = generateCoords();
        std::vector<cv::Plane3f> planes(6);
        planes[0] = cv::Plane3f(pts[0], pts[1], pts[2]); // FRONT
        planes[1] = cv::Plane3f(pts[4], pts[7], pts[6]); // BACK
        planes[2] = cv::Plane3f(pts[4], pts[0], pts[3]); // RIGHT
        planes[3] = cv::Plane3f(pts[6], pts[2], pts[1]); // LEFT
        planes[4] = cv::Plane3f(pts[7], pts[3], pts[2]); // TOP
        planes[5] = cv::Plane3f(pts[1], pts[0], pts[4]); // BOTTOM
        cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
        for (int idx = 0; idx < 6; ++idx) {
            faceVec.push_back(cv::rgbd::AlgebraicSurfacePatch::create(planes[idx]));
        }
        std::vector<cv::Vec3i> cornerIdxs(8);
        cornerIdxs[0] = cv::Vec3i(4, 1, 2); // TOP, BACK, RIGHT
        cornerIdxs[1] = cv::Vec3i(4, 1, 3); // TOP, BACK, LEFT
        cornerIdxs[2] = cv::Vec3i(4, 0, 2); // TOP, FRONT, RIGHT
        cornerIdxs[3] = cv::Vec3i(4, 0, 3); // TOP, FRONT, LEFT
        cornerIdxs[4] = cv::Vec3i(5, 1, 2); // BOTTOM, BACK, RIGHT
        cornerIdxs[5] = cv::Vec3i(5, 1, 3); // BOTTOM, BACK, LEFT
        cornerIdxs[6] = cv::Vec3i(5, 0, 2); // BOTTOM, FRONT, RIGHT
        cornerIdxs[7] = cv::Vec3i(5, 0, 3); // BOTTOM, FRONT, LEFT
        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> cornerGeom(3);
        for (int idx = 0; idx < 8; ++idx) {
            cornerGeom[0] = faceVec[cornerIdxs[idx][0]];
            cornerGeom[1] = faceVec[cornerIdxs[idx][1]];
            cornerGeom[2] = faceVec[cornerIdxs[idx][2]];
            cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
            for (int planeIdx = 0; planeIdx < 3; ++planeIdx) {
                geomPtr->addPart(cornerGeom[planeIdx]);
            }
            geomVec.push_back(geomPtr);
        }
        return geomVec;
    }

    std::vector<cv::rgbd::ObjectGeometry::Ptr> Box::getEdges() {
        std::vector<cv::rgbd::ObjectGeometry::Ptr> geomVec;
        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> faceVec;
        std::vector<cv::Vec3f> pts = generateCoords();
        std::vector<cv::Plane3f> planes(6);
        planes[0] = cv::Plane3f(pts[0], pts[1], pts[2]); // FRONT
        planes[1] = cv::Plane3f(pts[4], pts[7], pts[6]); // BACK
        planes[2] = cv::Plane3f(pts[4], pts[0], pts[3]); // RIGHT
        planes[3] = cv::Plane3f(pts[6], pts[2], pts[1]); // LEFT
        planes[4] = cv::Plane3f(pts[7], pts[3], pts[2]); // TOP
        planes[5] = cv::Plane3f(pts[1], pts[0], pts[4]); // BOTTOM
        cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
        for (int idx = 0; idx < 6; ++idx) {
            faceVec.push_back(cv::rgbd::AlgebraicSurfacePatch::create(planes[idx]));
        }
        std::vector<cv::Vec2i> edgeIdxs(12);
        edgeIdxs[0] = cv::Vec2i(0, 2);
        edgeIdxs[1] = cv::Vec2i(0, 3);
        edgeIdxs[2] = cv::Vec2i(0, 4);
        edgeIdxs[3] = cv::Vec2i(0, 5);
        edgeIdxs[4] = cv::Vec2i(1, 2);
        edgeIdxs[5] = cv::Vec2i(1, 3);
        edgeIdxs[6] = cv::Vec2i(1, 4);
        edgeIdxs[7] = cv::Vec2i(1, 5);
        edgeIdxs[8] = cv::Vec2i(2, 4);
        edgeIdxs[9] = cv::Vec2i(2, 5);
        edgeIdxs[10] = cv::Vec2i(3, 4);
        edgeIdxs[11] = cv::Vec2i(3, 5);
        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> edgeGeom(2);
        for (int idx = 0; idx < 12; ++idx) {
            edgeGeom[0] = faceVec[edgeIdxs[idx][0]];
            edgeGeom[1] = faceVec[edgeIdxs[idx][1]];
            cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
            for (int planeIdx = 0; planeIdx < 2; ++planeIdx) {
                geomPtr->addPart(edgeGeom[planeIdx]);
            }
            geomVec.push_back(geomPtr);
        }
        return geomVec;
    }

    std::vector<cv::rgbd::ObjectGeometry::Ptr> Box::getPlanes() {
        std::vector<cv::rgbd::ObjectGeometry::Ptr> geomVec;
        std::vector<cv::rgbd::AlgebraicSurfacePatch::Ptr> faceVec;
        std::vector<cv::Vec3f> pts = generateCoords();
        std::vector<cv::Plane3f> planes(6);
        planes[0] = cv::Plane3f(pts[0], pts[1], pts[2]); // FRONT
        planes[1] = cv::Plane3f(pts[4], pts[7], pts[6]); // BACK
        planes[2] = cv::Plane3f(pts[4], pts[0], pts[3]); // RIGHT
        planes[3] = cv::Plane3f(pts[6], pts[2], pts[1]); // LEFT
        planes[4] = cv::Plane3f(pts[7], pts[3], pts[2]); // TOP
        planes[5] = cv::Plane3f(pts[1], pts[0], pts[4]); // BOTTOM
        cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
        for (int idx = 0; idx < 6; ++idx) {
            faceVec.push_back(cv::rgbd::AlgebraicSurfacePatch::create(planes[idx]));
        }
        for (int idx = 0; idx < 6; ++idx) {
            cv::rgbd::ObjectGeometry::Ptr geomPtr = cv::rgbd::ObjectGeometry::create();
            geomPtr->addPart(faceVec[idx]);
            geomVec.push_back(geomPtr);
        }
        return geomVec;
    }

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

    std::vector<cv::Vec3i> Cylinder::generateCoordIndices(int N) {
        std::vector<cv::Vec3i> tris(N * 4);
        for (int i = 0; i < N - 1; ++i) {
            tris[i * 4] = cv::Vec3i(N, i + 1, i);
            tris[i * 4 + 1] = cv::Vec3i(2 * N + 1, N + 1 + i, N + 2 + i);
            tris[i * 4 + 2] = cv::Vec3i(N + i + 1, i + 1, N + 2 + i);
            tris[i * 4 + 3] = cv::Vec3i(i + 1, N + i + 1, i);
        }
        tris[N * 4 - 4] = cv::Vec3i(N, 0, N - 1);
        tris[N * 4 - 3] = cv::Vec3i(2 * N + 1, 2 * N, N + 1);
        tris[N * 4 - 2] = cv::Vec3i(0, 2 * N, N - 1);
        tris[N * 4 - 1] = cv::Vec3i(2 * N, 0, N + 1);
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

    std::vector<cv::Vec3i> Cylinder::generateNormalCoordIndices() {
        static int N = DEFAULT_RESOLUTION;
        std::vector<cv::Vec3i> norms(N * 4);
        for (int i = 0; i < N - 1; ++i) {
            norms[i * 4] = cv::Vec3i(N, N, N);
            norms[i * 4 + 1] = cv::Vec3i(N + 1, N + 1, N + 1);
            norms[i * 4 + 2] = cv::Vec3i(i, i + 1, 1 + i);
            norms[i * 4 + 3] = cv::Vec3i(i + 1, i, i);
        }
        norms[N * 4 - 4] = cv::Vec3i(N, N, N);
        norms[N * 4 - 3] = cv::Vec3i(N + 1, N + 1, N + 1);
        norms[N * 4 - 2] = cv::Vec3i(0, N - 1, N - 1);
        norms[N * 4 - 1] = cv::Vec3i(N - 1, 0, N - 1);
        return norms;
    }

    std::vector<cv::Vec3f> Cylinder::generateColorCoords() {
        static int N = DEFAULT_RESOLUTION;
        std::vector<cv::Vec3f> colors(N * 2 + 2);
        colors[0] = cv::Vec3f(1, 0, 0);
        colors[1] = cv::Vec3f(0, 1, 0);
        colors[2] = cv::Vec3f(0, 0, 1);
        return colors;
    }

    std::vector<cv::Vec3i> Cylinder::generateColorCoordIndices() {
        static int N = DEFAULT_RESOLUTION;
        std::vector<cv::Vec3i> colorIdxs(N * 4);
        for (int i = 0; i < N - 1; ++i) {
            colorIdxs[i * 4] = cv::Vec3i(1, 1, 1);
            colorIdxs[i * 4 + 1] = cv::Vec3i(2, 2, 2);
            colorIdxs[i * 4 + 2] = cv::Vec3i(0, 0, 0);
            colorIdxs[i * 4 + 3] = cv::Vec3i(0, 0, 0);
        }
        colorIdxs[N * 4 - 4] = cv::Vec3i(1, 1, 1);
        colorIdxs[N * 4 - 3] = cv::Vec3i(2, 2, 2);
        colorIdxs[N * 4 - 2] = cv::Vec3i(0, 0, 0);
        colorIdxs[N * 4 - 1] = cv::Vec3i(0, 0, 0);
        return colorIdxs;
    }
} /* namespace sg */