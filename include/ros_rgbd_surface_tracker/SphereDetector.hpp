/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SphereDetector.hpp
 * Author: John Papadakis
 *
 * Created on May 8, 2018, 2:59 PM
 */

#ifndef SPHEREDETECTOR_HPP
#define SPHEREDETECTOR_HPP

#include <tuple>

#include <boost/make_shared.hpp>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

class SphereDetector {
public:
    
    size_t min_points_for_fitting = 10;
    size_t iterations = 1000;
    float ransac_model_distance_threshold = .01; // distance from the spherical model within which point is considered an inlier
    float min_radius = .02; // meters
    float max_radius = .045; // meters
    float inlier_percentage_threshold = .6; // percentage of data within distance threshold of the refined model, used to accept or reject detection
    
    struct SphereDetection {
        float x;
        float y;
        float z;
        float radius;
        float confidence;
    };
    
    std::pair<bool, SphereDetection> detectSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr points) {
        
        bool sphere_detected = false;
        SphereDetector::SphereDetection sphere;
        
        if (points->size() > min_points_for_fitting) {
                
            sphere = this->fitSphericalModelRANSAC(points);

            if (sphere.confidence > inlier_percentage_threshold) {
                sphere_detected = true;
            }
            
        }
        
        return std::make_pair(sphere_detected, sphere);
        
    }
    
    SphereDetection fitSphericalModelRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr points) {
        return this->fitSphericalModelRANSAC(points, min_radius, 
                max_radius, ransac_model_distance_threshold, iterations);
    }
    
    static SphereDetection fitSphericalModelRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr points, 
            float min_radius, float max_radius, float ransac_model_distance_threshold, size_t iterations) {
        
        pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sphere_model = 
                boost::make_shared<pcl::SampleConsensusModelSphere<pcl::PointXYZ>>(points);
        sphere_model->setRadiusLimits(min_radius, max_radius);
        
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sphere_model);
        
        ransac.setDistanceThreshold(ransac_model_distance_threshold);
        
        ransac.setMaxIterations(iterations);
        ransac.computeModel();
        
        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);
        int inliers = ransac.inliers_.size();
        
        // Percentage of points within distance threshold of model
        float inlier_ratio = inliers/static_cast<float>(points->size());
        
//        std::cout << "Sphere coeffs: " << coeffs.transpose() << ", confidence: " << inlier_ratio << "\n";
        
        SphereDetection sphere;
        sphere.x = coeffs[0];
        sphere.y = coeffs[1];
        sphere.z = coeffs[2];
        sphere.radius = coeffs[3];
        sphere.confidence = inlier_ratio;
        
        return sphere;
        
    }
    
};

#endif /* SPHEREDETECTOR_HPP */

