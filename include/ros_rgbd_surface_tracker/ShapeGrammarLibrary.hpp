/* 
 * File:   ShapeGrammarLibrary.hpp
 * Author: arwillis
 *
 * Created on July 11, 2017, 9:35 PM
 */

#ifndef SHAPEGRAMMARLIBRARY_HPP
#define SHAPEGRAMMARLIBRARY_HPP

#include <vector>

#include <boost/shared_ptr.hpp>

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
    typedef boost::shared_ptr<Shape> Ptr;

    virtual ~Shape() {
    }
    virtual std::vector<cv::Vec3f> generateCoords() = 0;
    virtual std::vector<cv::Vec3i> generateCoordIndices() = 0;
    virtual std::vector<cv::Vec3f> generateNormals() = 0;
    virtual std::vector<cv::Vec3i> generateNormalCoordIndices() = 0;
    virtual std::vector<cv::Vec3f> generateColorCoords() = 0;
    virtual std::vector<cv::Vec3i> generateColorCoordIndices() = 0;
protected:
    Pose pose;
};

class Box : public Shape {
public:
    typedef boost::shared_ptr<Box> Ptr;
    Box() {
    }

    Box(cv::Vec3f _dims, Pose _pose) {
        dims = _dims;
        pose = _pose;
    }

    std::vector<cv::Vec3f> generateCoords();

    std::vector<cv::Vec3i> generateCoordIndices();

    std::vector<cv::Vec3f> generateNormals();

    std::vector<cv::Vec3i> generateNormalCoordIndices();

    std::vector<cv::Vec3f> generateColorCoords();

    std::vector<cv::Vec3i> generateColorCoordIndices();

private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    //Box(const Box& ref);
    //Box& operator=(const Box& ref);
    cv::Point3f dims; // length, width, height
};

class Cylinder : public Shape {
public:
    typedef boost::shared_ptr<Cylinder> Ptr;
    Cylinder() {
    }

    Cylinder(float _radius, float _height, Pose _pose) {
        r = _radius;
        h = _height;
        pose = _pose;
    }

    std::vector<cv::Vec3f> generateCoords() {
        static int N = DEFAULT_RESOLUTION;
        return generateCoords(N);
    }

    std::vector<cv::Vec3f> generateCoords(int N);

    std::vector<cv::Vec3i> generateCoordIndices() {
        static int N = DEFAULT_RESOLUTION;
        return generateCoordIndices(N);
    }

    std::vector<cv::Vec3i> generateCoordIndices(int N);

    std::vector<cv::Vec3f> generateNormals();

    std::vector<cv::Vec3i> generateNormalCoordIndices();

    std::vector<cv::Vec3f> generateColorCoords();

    std::vector<cv::Vec3i> generateColorCoordIndices();

private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    //Cylinder(const Cylinder& ref);
    //Cylinder& operator=(const Cylinder& ref);
    static constexpr float DEFAULT_RESOLUTION = 16;
    float r, h; // length, width, height
};

#endif /* __cplusplus */

#endif /* SHAPEGRAMMARLIBRARY_HPP */

