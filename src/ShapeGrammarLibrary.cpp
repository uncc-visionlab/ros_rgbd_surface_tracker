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

    std::vector<int> Plane::generateCoordIndices() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<int> ptidxs; // reserve(uv_poly_coords.size()*3)
        int ctr_pt_idx = uv_poly_coords.size();
        for (int triIdx = 0; triIdx < uv_poly_coords.size() - 1; ++triIdx) {
            ptidxs.insert(ptidxs.end(),{ctr_pt_idx, triIdx, triIdx + 1});
        }
        ptidxs.insert(ptidxs.end(),{ctr_pt_idx, (int) uv_poly_coords.size() - 1, 0});
        return ptidxs;
    }

    std::vector<cv::Vec3f> Plane::generateNormals() {
        std::vector<cv::Vec3f> norms = {cv::Vec3f(x, y, z)};
        for (int idx = 0; idx < norms.size(); ++idx) {
            pose.rotateInPlace(norms[idx]);
        }
        return norms;
    }

    std::vector<int> Plane::generateNormalCoordIndices() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<int> normidxs; // reserve(uv_poly_coords.size()*3);
        for (int triIdx = 0; triIdx < uv_poly_coords.size(); ++triIdx) {
            normidxs.insert(normidxs.end(),{0, 0, 0});
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

    std::vector<int> Plane::generateColorCoordIndices() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<int> coloridxs; // reserve(uv_poly_coords.size()*3);
        for (int triIdx = 0; triIdx < uv_poly_coords.size(); ++triIdx) {
            coloridxs.insert(coloridxs.end(),{0, 0, 0});
        }
        return coloridxs;
    }
    // compute vertices

    std::vector<cv::Vec3f> Edge::generateCoords() {
        std::vector<cv::Vec3f> pts = {
            getPoint(start),
            getPoint(end),
            //getPoint(tstart[0]),
            //getPoint(tend[0]),
            //getPoint(tstart[1]),
            //getPoint(tend[1])
        };
        return pts;
    }

    std::vector<int> Edge::generateCoordIndices() {
        std::vector<int> ptIdxs = {0, 1}; //, 2, 3};
        return ptIdxs;
    }

    std::vector<cv::Vec3f> Edge::generateNormals() {
        std::vector<cv::Vec3f> norms = {cv::Vec3f(0, 0, -1)};
        return norms;
    }

    std::vector<int> Edge::generateNormalCoordIndices() {
        std::vector<int> normIdxs = {0, 0}; //, 0, 0};
        return normIdxs;
    }

    std::vector<cv::Vec3f> Edge::generateColorCoords() {
        std::vector<cv::Vec3f> colors = {cv::Vec3f(1, 0, 0)}; //, cv::Vec3f(0, 1, 0)};
        return colors;
    }

    std::vector<int> Edge::generateColorCoordIndices() {
        std::vector<int> colorIdxs = {0, 0}; //, 1, 1};
        return colorIdxs;
    }

    std::vector<cv::Vec3f> Corner::generateCoords() {
        std::vector<cv::Vec3f> pts = {
            edges[0]->getPoint(edges[0]->end),
            edges[0]->getPoint(eline_lambdas[0]),
            edges[1]->getPoint(edges[1]->end),
            edges[1]->getPoint(eline_lambdas[1]),
            edges[2]->getPoint(edges[2]->end),
            edges[2]->getPoint(eline_lambdas[2])
        };
        return pts;
    }

    std::vector<int> Corner::generateCoordIndices() {
        std::vector<int> ptIdxs = {0, 1, 2, 3, 4, 5};
        return ptIdxs;
    }

    std::vector<cv::Vec3f> Corner::generateNormals() {
        std::vector<cv::Vec3f> norms = {cv::Vec3f(0, 0, -1)};
        return norms;
    }

    std::vector<int> Corner::generateNormalCoordIndices() {
        std::vector<int> normIdxs = {0, 0, 0, 0, 0, 0};
        return normIdxs;
    }

    std::vector<cv::Vec3f> Corner::generateColorCoords() {
        std::vector<cv::Vec3f> colors = {
            cv::Vec3f(0, 1, 0),
            cv::Vec3f(0, 1, 0),
            cv::Vec3f(0, 1, 0)
        };
        return colors;
    }

    std::vector<int> Corner::generateColorCoordIndices() {
        std::vector<int> colorIdxs = {0, 0, 1, 1, 2, 2};
        return colorIdxs;
    }

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