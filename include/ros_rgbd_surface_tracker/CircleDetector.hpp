/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CircleDetector.hpp
 * Author: John Papadakis
 *
 * Created on May 8, 2018, 2:54 PM
 */

#ifndef CIRCLEDETECTOR_HPP
#define CIRCLEDETECTOR_HPP

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CircleDetector {
public:
    
    float bounding_box_ratio_threshold = .92; // ratio between the shortest side to the longest side of the bounding box, range 0 to 1
    float min_radius = 6; // minimum radius of candidate circle in pixels
    float max_radius = 50; // maximum radius of candidate circle in pixels
    float circular_fill_ratio_threshold = .8; // ratio of number of pixels within candidate circle and expected circle area, range 0 to 1
    float component_area_ratio_threshold = .9; // ratio of number of pixels within candidate circle and total component area, range 0 to 1
    
    struct CircleDetection {
        float x;
        float y;
        float radius;
        std::vector<cv::Point2i> locations;
    };
    
    std::vector<CircleDetection> detectCircles(const cv::Mat1b& binary_image) {
        return std::move(this->detectCircles(binary_image, 
                    bounding_box_ratio_threshold, 
                    min_radius, max_radius, 
                    circular_fill_ratio_threshold, 
                    component_area_ratio_threshold)
                );
    }
    
    static std::vector<CircleDetection> detectCircles(const cv::Mat1b& binary_image, 
            float bounding_box_ratio_threshold, float min_radius, float max_radius, 
            float circular_fill_ratio_threshold, float component_area_ratio_threshold) {
        
        std::vector<CircleDetection> detections;
        cv::Mat labeled_image;
        cv::Mat stats;
        cv::Mat centroids;
        cv::connectedComponentsWithStats(binary_image, labeled_image, stats, centroids);

        for (size_t label = 1; label < stats.rows; ++label) {
            // Validate each connected component bounding box

            int component_area =  stats.at<int>(label, cv::CC_STAT_AREA);
            int bb_width = stats.at<int>(label, cv::CC_STAT_WIDTH);
            int bb_height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
            float bb_ratio = (bb_width > bb_height) ? 
                static_cast<float>(bb_height)/bb_width : static_cast<float>(bb_width)/bb_height;

            if (bb_ratio > bounding_box_ratio_threshold) {
                // Bounding box is square enough, compute candidate circle and check data against it

                float circle_radius = (bb_width + bb_height)/4.0f;

                if (circle_radius > min_radius and circle_radius < max_radius) {

                    cv::Point2f circle_center(bb_width/2.0f, bb_height/2.0f);
                    float circle_radius_sq = std::pow(circle_radius, 2);
                    float circle_area = pi*circle_radius_sq;

                    // Get nonzero (colorful) pixel locations within bounding box
                    int bb_x = stats.at<int>(label, cv::CC_STAT_LEFT);
                    int bb_y = stats.at<int>(label, cv::CC_STAT_TOP);
                    cv::Rect bb_roi = cv::Rect(bb_x, bb_y, bb_width, bb_height);
                    std::vector<cv::Point2i> xypoints;
                    cv::findNonZero(binary_image(bb_roi), xypoints);
                    cv::Mat xypoints_mat(xypoints, false); // note: points to same data as xypoints
                    xypoints_mat = xypoints_mat.reshape(1);
                    cv::Mat xypoints_float;
                    xypoints_mat.convertTo(xypoints_float, CV_32F);

                    // Check that the number of pixels inside circle is close to area of the circle,
                    // also check that enough of component pixels are inside circle vs out
                    cv::Mat zero_centered_points(xypoints_float.rows, xypoints_float.cols, CV_32FC1);
                    zero_centered_points.col(0) = xypoints_float.col(0) - circle_center.x;
                    zero_centered_points.col(1) = xypoints_float.col(1) - circle_center.y;
                    cv::Mat point_radii_sq;
                    cv::reduce(zero_centered_points.mul(zero_centered_points), point_radii_sq, 1, CV_REDUCE_SUM);
                    float area_points_inside_circle = cv::countNonZero(point_radii_sq <= circle_radius_sq);
                    float circular_fill_ratio = area_points_inside_circle/circle_area;
                    float component_area_ratio = area_points_inside_circle/component_area;

                    if (circular_fill_ratio > circular_fill_ratio_threshold and 
                            component_area_ratio > component_area_ratio_threshold) {

                        CircleDetection circle;
                        circle.x = circle_center.x + bb_x;
                        circle.y = circle_center.y + bb_y;
                        circle.radius = circle_radius;
                        xypoints_mat.col(0) = xypoints_mat.col(0) + bb_x;
                        xypoints_mat.col(1) = xypoints_mat.col(1) + bb_y;
                        circle.locations = std::move(xypoints);
                        detections.push_back(std::move(circle));

                    }

                }

            }

        }
        
        return detections;

    }

private:
    
    static constexpr double pi = std::acos(-1.0);
    
};

#endif /* CIRCLEDETECTOR_HPP */

