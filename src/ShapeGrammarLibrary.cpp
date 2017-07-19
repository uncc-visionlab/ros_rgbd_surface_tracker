/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <vector>

#include <ros_rgbd_surface_tracker/ShapeGrammarLibrary.hpp>

namespace sg {

    std::vector<cv::Vec3f> Plane::generateCoords() {
        std::vector<cv::Vec2f> uv_poly_coords = uv_coords[0];
        std::vector<cv::Vec3f> pts(uv_poly_coords.size() + 1);
        cv::Vec3f ctr_pt(0, 0, 0);
        for (int ptidx=0; ptidx < uv_poly_coords.size(); ++ptidx) {
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
        norms[0] = cv::Vec3f(x, y, -z);
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
        colors[0] = cv::Vec3f(1.0f, 0.0f, 0.0f);
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