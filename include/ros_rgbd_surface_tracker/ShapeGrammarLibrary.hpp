/* 
 * File:   ShapeGrammarLibrary.hpp
 * Author: arwillis
 *
 * Created on July 11, 2017, 9:35 PM
 */

#ifndef SHAPEGRAMMARLIBRARY_HPP
#define SHAPEGRAMMARLIBRARY_HPP

#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <GL/glut.h>

#ifdef __cplusplus

class Pose {
public:

    Pose() : rodrigues(0, 0, 0), position(0, 0, 0) {

    }

    Pose(cv::Vec3f _position, cv::Vec3f _rodrigues = cv::Vec3f(0, 0, 0)) {
        position = _position;
        rodrigues = _rodrigues;
    }

    void rotateInPlace(cv::Vec3f& vec) {
        cv::Mat rotMat;
        cv::Rodrigues(rodrigues, rotMat);
        float *mm = rotMat.ptr<float>(0, 0);
        vec[0] = mm[0] * vec[0] + mm[1] * vec[1] + mm[2] * vec[2];
        vec[1] = mm[3] * vec[0] + mm[4] * vec[1] + mm[5] * vec[2];
        vec[2] = mm[6] * vec[0] + mm[7] * vec[1] + mm[8] * vec[2];
    }

    void transformInPlace(cv::Vec3f& pt) {
        cv::Mat rotMat;
        cv::Rodrigues(rodrigues, rotMat);
        float *mm = rotMat.ptr<float>(0, 0);
        pt[0] = mm[0] * pt[0] + mm[1] * pt[1] + mm[2] * pt[2] + position[0];
        pt[1] = mm[3] * pt[0] + mm[4] * pt[1] + mm[5] * pt[2] + position[1];
        pt[2] = mm[6] * pt[0] + mm[7] * pt[1] + mm[8] * pt[2] + position[2];
    }

    cv::Vec3f getTranslation() {
        return position;
    }
private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    //Pose(const Pose& ref);
    //Pose& operator=(const Pose& ref);
    cv::Vec3f rodrigues;
    cv::Vec3f position;
};

// Global interface for shapes

class Shape {
public:

    virtual ~Shape() {
    }
    virtual std::vector<cv::Vec3f> generateCoords() = 0;
    virtual std::vector<cv::Vec3i> generateCoordIndices() = 0;
    virtual std::vector<cv::Vec3f> generateNormals() = 0;
    virtual std::vector<cv::Vec3i> generateNormalCoordIndices() = 0;
    virtual std::vector<cv::Vec3f> generateColorCoords() = 0;
    virtual std::vector<cv::Vec3i> generateColorCoordIndices() = 0;
};

class Box : public Shape {
public:

    Box() {
    }

    Box(cv::Vec3f _dims, Pose _pose) {
        dims = _dims;
        pose = _pose;
    }

    // compute vertices

    std::vector<cv::Vec3f> generateCoords() {
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

    std::vector<cv::Vec3i> generateNormalCoordIndices() {
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

    std::vector<cv::Vec3f> generateNormals() {
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

    // compute triangle surfaces

    std::vector<cv::Vec3i> generateCoordIndices() {
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

    std::vector<cv::Vec3f> generateColorCoords() {
        std::vector<cv::Vec3f> colors(6);
        colors[0] = cv::Point3f(1, 0, 0);
        colors[1] = cv::Point3f(0, 1, 0);
        colors[2] = cv::Point3f(0, 0, 1);
        colors[3] = cv::Point3f(1, 0, 0); // not red
        colors[4] = cv::Point3f(0, 1, 0); // not red
        colors[5] = cv::Point3f(0, 0, 1);
        return colors;
    }

    std::vector<cv::Vec3i> generateColorCoordIndices() {
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

private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    //Box(const Box& ref);
    //Box& operator=(const Box& ref);
    Pose pose;
    cv::Point3f dims; // length, width, height
};

#endif /* __cplusplus */

#endif /* SHAPEGRAMMARLIBRARY_HPP */

