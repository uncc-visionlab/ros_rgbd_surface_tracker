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

#ifndef PLANEIMAGEQUANTIZER_HPP
#define PLANEIMAGEQUANTIZER_HPP

#include <vector>
#include <math.h>

#include <opencv2/core/mat.hpp>

class PlaneImageQuantizer {
#define PI 3.14159265f
    uint8_t nbitsAZ, nbitsEL, nbitsD;
    uint32_t az_scalef, el_scalef, d_scalef;
    static const int MAX_DISTANCE = 10;
    uint32_t streamPos;
    std::vector<float> azimuthLUT_sin;
    std::vector<float> azimuthLUT_cos;
    std::vector<float> elevationLUT_sin;
    std::vector<float> elevationLUT_cos;
    // Azimuth is a value [-pi, pi]
    // Elevation is a value [0, pi/2]
    // D(istance) is a value [0, MAX_DISTANCE)
public:

    void compressVec4f(const cv::Vec4f& vec, std::vector<int8_t>& byteVec, uint32_t& vecIdx) {
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

    void decompressVec4f(const std::vector<int8_t>& byteVec, uint32_t& vecIdx, cv::Vec4f& vec) {
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
        //float azf = ((((float) az) / (az_scalef >> 1)) - 1.0f) * PI;
        //float elf = (((float) el) * PI) / (2.0f * el_scalef);
        float df = (float) (d * MAX_DISTANCE) / d_scalef;
        //std::cout << std::dec << "compress(float) " << "az = " << azf << " el = " << elf << " d = " << df << std::endl;
        //vec[0] = sin(elf) * cos(azf);
        //vec[1] = sin(elf) * sin(azf);
        //vec[2] = -cos(elf);
        vec[0] = elevationLUT_sin[el] * azimuthLUT_cos[az];
        vec[1] = elevationLUT_sin[el] * azimuthLUT_sin[az];
        vec[2] = -elevationLUT_cos[el];
        vec[3] = df;
        //std::cout << "decompress(4f) " << "vec = " << vec << std::endl;
    }
public:

    PlaneImageQuantizer(int _nbitsAZ, int _nbitsEL, int _nbitsD) :
    nbitsAZ(_nbitsAZ), nbitsEL(_nbitsEL), nbitsD(_nbitsD) {
        az_scalef = pow(2, nbitsAZ) - 1;
        el_scalef = pow(2, nbitsEL) - 1;
        d_scalef = pow(2, nbitsD) - 1;
        for (uint32_t az = 0; az <= az_scalef; az++) {
            float azf = ((((float) az) / (az_scalef >> 1)) - 1.0f) * PI;
            azimuthLUT_cos.push_back(cos(azf));
            azimuthLUT_sin.push_back(sin(azf));
        }
        for (uint32_t el = 0; el <= el_scalef; el++) {
            float elf = (((float) el) * PI) / (2.0f * el_scalef);
            elevationLUT_cos.push_back(cos(elf));
            elevationLUT_sin.push_back(sin(elf));
        }
    }

    virtual ~PlaneImageQuantizer() {
    }

    uint32_t getPos() {
        return streamPos;
    }

    void quantize(cv::Mat_<cv::Vec4f> planeImage, std::vector<int8_t>& byteVec, uint32_t initialBytePos = 0) {
        streamPos = initialBytePos;
        for (int r = 0; r < planeImage.rows; r++) {
            cv::Vec4f* ptr = planeImage.ptr<cv::Vec4f>(r);
            for (int c = 0; c < planeImage.cols; c++) {
                compressVec4f(ptr[c], byteVec, streamPos);
            }
        }
    }

    cv::Mat dequantize(const std::vector<int8_t>& byteVec, int rows, int cols, uint32_t initialBytePos = 0) {
        cv::Mat_<cv::Vec4f> planeImage(rows, cols);
        streamPos = initialBytePos;
        for (int r = 0; r < planeImage.rows; r++) {
            // We obtain a pointer to the beginning of row r
            cv::Vec4f* ptr = planeImage.ptr<cv::Vec4f>(r);
            for (int c = 0; c < planeImage.cols; c++) {
                decompressVec4f(byteVec, streamPos, ptr[c]);
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

#endif /* PLANEIMAGEQUANTIZER_HPP */

