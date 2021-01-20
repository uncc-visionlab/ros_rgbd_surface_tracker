/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PlaneImageCompressor.hpp
 * Author: arwillis
 *
 * Created on January 19, 2021, 10:08 PM
 */

#ifndef PLANEIMAGECOMPRESSOR_HPP
#define PLANEIMAGECOMPRESSOR_HPP

#include <vector>
#include <math.h>

#include <opencv2/core/mat.hpp>

class PlaneImageCompressor {
#define PI 3.14159265f
    uint8_t nbitsAZ, nbitsEL, nbitsD;
    uint32_t az_scalef, el_scalef, d_scalef;
    static const int MAX_DISTANCE = 10;

    // Azimuth is a value [-pi, pi]
    // Elevation is a value [0, pi/2]
    // D(istance) is a value [0, MAX_DISTANCE)
    // IMPORTANT: input can never be equal to MAX_DISTANCE
public:

    void compressVec4f(cv::Vec4f vec, std::vector<int8_t>& byteVec, int& vecIdx) {
        //std::cout << "compress(4f) " << "vec = " << vec << std::endl;
        float azf = atan2(vec[1], vec[0]);
        float elf = acos(-vec[2]);
        float df = vec[3];
        //std::cout << std::dec << "compress(float) " << "az = " << azf << " el = " << elf << " d = " << df << std::endl;
        uint32_t az = (uint32_t) (((azf / PI) + 1.0f) * (az_scalef >> 1));
        uint32_t el = (uint32_t) ((elf * 2.0f / PI) * el_scalef);
        uint32_t d = (uint32_t) ((vec[3] / MAX_DISTANCE) * d_scalef);
        //std::cout << std::dec << "compress(int8) " << "az = " << az << " el = " << el << " d = " << d << std::endl;
        uint32_t compVal = (d << (nbitsAZ + nbitsEL)) | (el << nbitsAZ) | az;
        //std::cout << std::hex << "compress(int32) " << "compVal = " << compVal << std::endl;
        byteVec.push_back((int8_t) ((compVal & 0xff000000) >> 24));
        byteVec.push_back((int8_t) ((compVal & 0x00ff0000) >> 16));
        byteVec.push_back((int8_t) ((compVal & 0x0000ff00) >> 8));
        byteVec.push_back((int8_t) ((compVal & 0x000000ff) >> 0));
        //std::cout << std::hex << "compress(int8) " << "bytes ("
        //        << (uint16_t) byteVec[vecIdx] << ", " << (uint16_t) byteVec[vecIdx + 1] << ", " 
        //        << (uint16_t) byteVec[vecIdx + 2] << ", " << (uint16_t) byteVec[vecIdx + 3] << ")" << std::endl;
        vecIdx += 4;
    }

    void decompressVec4f(std::vector<int8_t> byteVec, int& vecIdx, cv::Vec4f& vec) {
        //std::cout << std::hex << "compress(int8) " << "bytes ("
        //        << (uint16_t) byteVec[vecIdx] << ", " << (uint16_t) byteVec[vecIdx + 1] << ", "
        //        << (uint16_t) byteVec[vecIdx + 2] << ", " << (uint16_t) byteVec[vecIdx + 3] << ")" << std::endl;
        uint32_t compVal = 0;
        compVal |= (uint8_t) byteVec[vecIdx++];
        compVal <<= 8;
        compVal |= (uint8_t) byteVec[vecIdx++];
        compVal <<= 8;
        compVal |= (uint8_t) byteVec[vecIdx++];
        compVal <<= 8;
        compVal |= (uint8_t) byteVec[vecIdx++];
        //uint32_t compVal = (((uint32_t)byteVec[vecIdx]) << 24) | (((uint32_t)byteVec[vecIdx + 1]) << 16) | 
        //        (((uint32_t)byteVec[vecIdx + 2]) << 8) | (((uint32_t)byteVec[vecIdx + 3]) << 0);
        //std::cout << std::hex << "decompress(int32) " << "compVal = " << compVal << std::endl;
        uint32_t d = compVal >> (nbitsAZ + nbitsEL);
        uint32_t el = (compVal << nbitsD) >> (nbitsD + nbitsAZ);
        uint32_t az = (compVal << (nbitsD + nbitsEL)) >> (nbitsD + nbitsEL);
        //std::cout << std::dec << "decompress(int) " << "az = " << az << " el = " << el << " d = " << d << std::endl;
        float azf = ((((float) az) / (az_scalef >> 1)) - 1.0f) * PI;
        float elf = (((float) el) * PI) / (2.0f * el_scalef);
        float df = (float) (d * MAX_DISTANCE) / d_scalef;
        //std::cout << std::dec << "compress(float) " << "az = " << azf << " el = " << elf << " d = " << df << std::endl;
        vec[0] = sin(elf) * cos(azf);
        vec[1] = sin(elf) * sin(azf);
        vec[2] = -cos(elf);
        vec[3] = df;
        //std::cout << "decompress(4f) " << "vec = " << vec << std::endl;
    }
public:

    PlaneImageCompressor(int _nbitsAZ, int _nbitsEL, int _nbitsD) :
    nbitsAZ(_nbitsAZ), nbitsEL(_nbitsEL), nbitsD(_nbitsD) {
        az_scalef = pow(2, nbitsAZ) - 1;
        el_scalef = pow(2, nbitsEL) - 1;
        d_scalef = pow(2, nbitsD) - 1;
    }

    virtual ~PlaneImageCompressor() {
    }

    std::vector<int8_t> compress(cv::Mat_<cv::Vec4f> planeImage) {
        std::vector<int8_t> byteVec;
        int vecIdx = 0;
        for (int r = 0; r < planeImage.rows; r++) {
            cv::Vec4f* ptr = planeImage.ptr<cv::Vec4f>(r);
            for (int c = 0; c < planeImage.cols; c++) {
                compressVec4f(ptr[c], byteVec, vecIdx);
            }
        }
        return byteVec;
    }

    cv::Mat decompress(std::vector<int8_t> byteVec, int rows, int cols) {
        cv::Mat_<cv::Vec4f> planeImage(rows, cols);
        int vecIdx = 0;
        for (int r = 0; r < planeImage.rows; r++) {
            // We obtain a pointer to the beginning of row r
            cv::Vec4f* ptr = planeImage.ptr<cv::Vec4f>(r);
            for (int c = 0; c < planeImage.cols; c++) {
                decompressVec4f(byteVec, vecIdx, ptr[c]);
            }
        }
        return planeImage;
    }

    static cv::Vec4f generatePlaneVector() {
        cv::Vec4f plane;
        cv::randu(plane, cv::Scalar(-1.0), cv::Scalar(1.0));
        plane[2] = (plane[2] > 0) ? -plane[2] : plane[2];
        float normf = sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
        plane[0] /= normf;
        plane[1] /= normf;
        plane[2] /= normf;
        plane[3] = (plane[3] < 0) ? -MAX_DISTANCE * plane[3] : MAX_DISTANCE * plane[3];
        //plane[3] = (plane[3] == MAX_DISTANCE) ? 0 : plane[3];
        return plane;
    }

    static cv::Mat generatePlaneMatrix(int rows, int cols) {
        cv::theRNG().state = cv::getTickCount();
        cv::Mat_<cv::Vec4f> matrix(rows, cols);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                cv::Vec4f plane = generatePlaneVector();
                matrix.at<cv::Vec4f>(r, c) = plane;
            }
        }
        return matrix;
    }
};

#endif /* PLANEIMAGECOMPRESSOR_HPP */

