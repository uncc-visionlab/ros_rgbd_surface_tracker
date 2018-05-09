/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PixelColorClassifier.hpp
 * Author: John Papadakis
 *
 * Created on May 8, 2018, 2:38 PM
 */

#ifndef PIXELCOLORCLASSIFIER_HPP
#define PIXELCOLORCLASSIFIER_HPP

#include <cmath>
#include <tuple>
#include <map>

#include <opencv2/core.hpp>

#include <colored_sphere_detector/Gaussian.hpp>

enum class Color {
    OTHER, RED, GREEN, BLUE, YELLOW, ORANGE
};

class PixelColorClassifier {
public:
    
    std::map<Color, Gaussian<float, 3>> color_class_map = {
            {Color::RED, Gaussian<float, 3>(cv::Vec3f(0.6762, 0.1513, 0.1850), cv::Matx33f(
                    0.0134, 0.0052, 0.0064, 
                    0.0052, 0.0038, 0.0042, 
                    0.0064, 0.0042, 0.0054))},
            {Color::GREEN, Gaussian<float, 3>(cv::Vec3f(0.1387, 0.4116, 0.2718), cv::Matx33f(
                    0.0066, 0.0080, 0.0080, 
                    0.0080, 0.0193, 0.0152, 
                    0.0080, 0.0152, 0.0134))},
            {Color::BLUE, Gaussian<float, 3>(cv::Vec3f(0.0659, 0.3986, 0.7374), cv::Matx33f(
                    0.0113, 0.0083, 0.0034,
                    0.0083, 0.0193, 0.0186, 
                    0.0034, 0.0186, 0.0230))},
            {Color::YELLOW, Gaussian<float, 3>(cv::Vec3f(0.8320, 0.7906, 0.2898), cv::Matx33f(
                    0.0154, 0.0174, 0.0073,
                    0.0174, 0.0202, 0.0088,
                    0.0073, 0.0088, 0.0149))},
            {Color::ORANGE, Gaussian<float, 3>(cv::Vec3f(0.8017, 0.2349, .1267), cv::Matx33f(
                    0.0133, 0.0070, 0.0019,
                    0.0070, 0.0070, 0.0042,
                    0.0019, 0.0042, 0.0041))},
        };
        
    float colorful_threshold = .09; // minimum magnitude of the vector rejection of the pixel color vector onto the intensity vector (1, 1, 1)
    float color_likelihood_threshold = -8; // minimum likelihood value for pixel to not be classified as other
    
    cv::Mat classifyPixelColors(const cv::Mat& rgb_image) {
        return this->classifyPixelColors(rgb_image, color_class_map, colorful_threshold, color_likelihood_threshold);
    }
    
    static cv::Mat classifyPixelColors(const cv::Mat& rgb_image, 
            const std::map<Color, Gaussian<float, 3>>& color_class_map,
            float colorful_threshold, 
            float color_likelihood_threshold) {
                
        cv::Mat1b colorful(rgb_image.rows, rgb_image.cols);
        cv::Mat1b color_classes(rgb_image.rows, rgb_image.cols);
        
        // Create orthonormal basis in the (1, 1, 1) plane
        cv::Vec3f v1_perp = (1/std::sqrt(2.0))*cv::Vec3f(-1, 1, 0);
        cv::Vec3f v2_perp = (1/std::sqrt(6.0))*cv::Vec3f(1, 1, -2);
        cv::Matx23f orthogonal_projection = {
                v1_perp(0), v1_perp(1), v1_perp(2), 
                v2_perp(0), v2_perp(1), v2_perp(2)
        };
        
        std::map<Color, Gaussian<float, 2>> projected_color_class_map;
        for (const std::pair<Color, Gaussian<float, 3>>& color : color_class_map) {
            projected_color_class_map[color.first] = Gaussian<float, 3>::transform(color.second, orthogonal_projection);
        }
        
        rgb_image.forEach<cv::Vec3f>(
            [&](const cv::Vec3f& pixel, const int* position) -> void {
                size_t row = position[0];
                size_t col = position[1];
                
                cv::Vec2f pixel_vector = orthogonal_projection*(pixel/255.0);
                float pixel_magnitude = cv::norm<float, 2, 1>(pixel_vector);
                
                if (pixel_magnitude > colorful_threshold) {
                    colorful.at<int8_t>(row, col) = true;
                    
                    Color pixel_color = Color::OTHER;
                    float max_color_likelihood = -std::numeric_limits<float>::infinity();
                    for (const std::pair<Color, Gaussian<float, 2>>& color : projected_color_class_map) {
                        const Gaussian<float, 2>& color_class = color.second;
                        
                        float color_likelihood = color_class.evaluateLogLikelihood(pixel_vector);
//                        
                        
                        if (color_likelihood > max_color_likelihood) {
                            max_color_likelihood = color_likelihood;
                            pixel_color = color.first;
                        }
                        
                    }
                    
                    if (max_color_likelihood > color_likelihood_threshold) {
                        color_classes.at<int8_t>(row, col) = static_cast<size_t>(pixel_color);
                    } else {
                        color_classes.at<int8_t>(row, col) = static_cast<size_t>(Color::OTHER);
                    }
                    
                } else {
                    colorful.at<int8_t>(row, col) = false;
                    color_classes.at<int8_t>(row, col) = static_cast<size_t>(Color::OTHER);
                }
            }
        );
        
        return color_classes;
    }
    
};

#endif /* PIXELCOLORCLASSIFIER_HPP */

