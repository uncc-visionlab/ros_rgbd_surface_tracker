/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ShapeGrammarLibrary.hpp
 * Author: arwillis
 *
 * Created on July 11, 2017, 9:35 PM
 */

#ifndef SHAPEGRAMMARLIBRARY_HPP
#define SHAPEGRAMMARLIBRARY_HPP

#include <vector>

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

class Box {
public:

    Box() {
    }

    Box(cv::Vec3f _dims, Pose _pose) {
        dims = _dims;
        pose = _pose;
    }

    // compute vertices

    std::vector<cv::Point3f> generateVertices() {
        std::vector<cv::Point3f> pts(8);
        pts[0] = cv::Point3f(-dims.x / 2, -dims.y / 2, -dims.z / 2);
        pts[1] = cv::Point3f(dims.x / 2, -dims.y / 2, -dims.z / 2);
        pts[2] = cv::Point3f(dims.x / 2, dims.y / 2, -dims.z / 2);
        pts[3] = cv::Point3f(-dims.x / 2, dims.y / 2, -dims.z / 2);
        pts[4] = cv::Point3f(-dims.x / 2, -dims.y / 2, dims.z / 2);
        pts[5] = cv::Point3f(dims.x / 2, -dims.y / 2, dims.z / 2);
        pts[6] = cv::Point3f(dims.x / 2, dims.y / 2, dims.z / 2);
        pts[7] = cv::Point3f(-dims.x / 2, dims.y / 2, dims.z / 2);
        return pts;
    }
    
    // compute triangle surfaces

    std::vector<cv::Point3i> generateFaces() {
        std::vector<cv::Point3i> tris(12);
        tris[0] = cv::Point3i(0, 5, 4);
        tris[1] = cv::Point3i(5, 0, 1);
        tris[2] = cv::Point3i(1, 6, 5);
        tris[3] = cv::Point3i(6, 1, 2);
        tris[4] = cv::Point3i(2, 7, 6);
        tris[5] = cv::Point3i(7, 2, 3);
        tris[6] = cv::Point3i(3, 4, 7);
        tris[7] = cv::Point3i(4, 3, 0);
        tris[8] = cv::Point3i(4, 6, 7);
        tris[9] = cv::Point3i(6, 4, 5);
        tris[10] = cv::Point3i(1, 3, 2);
        tris[11] = cv::Point3i(3, 1, 0);
        return tris;
    }

private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    Box(const Box& ref);
    Box& operator=(const Box& ref);
    Pose pose;
    cv::Point3f dims; // length, width, height
};

#endif /* __cplusplus */

#endif /* SHAPEGRAMMARLIBRARY_HPP */

