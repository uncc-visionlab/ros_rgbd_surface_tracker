/* 
 * File:   point_reconstruction.hpp
 * Author: John Papadakis
 *
 * Created on March 16, 2018, 1:42 PM
 */

#ifndef POINT_RECONSTRUCTION_HPP
#define POINT_RECONSTRUCTION_HPP

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include <boost/make_shared.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace gfd {
    
    template <typename scalar_t>
    static cv::Point3_<scalar_t> reconstruct(cv::Point2i pixel, scalar_t depth, const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        scalar_t x = depth*(pixel.x - image_center.x)/focal_length.x;
        scalar_t y = depth*(pixel.y - image_center.y)/focal_length.y;
        return cv::Point3_<scalar_t>(x, y, depth);
    }
    
    template <typename scalar_t>
    static std::vector<cv::Point3_<scalar_t>> reconstruct(const std::vector<cv::Point2i>& pixels, const cv::Mat_<scalar_t>& depth_image, const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        
        std::vector<cv::Point3_<scalar_t>> points;
        points.reserve(pixels.size());
        
        for (const cv::Point2i& pixel : pixels) {
            size_t index = pixel.y*depth_image.cols + pixel.x;
            
            const scalar_t& z = reinterpret_cast<scalar_t*>(depth_image.data)[index];
            if (not std::isnan(z)) {
                scalar_t x = z*(pixel.x - image_center.x)/focal_length.x;
                scalar_t y = z*(pixel.y - image_center.y)/focal_length.y;
                points.emplace_back(x, y, z);
            }
            
        }
        
        return points;
        
    }
    
    template <typename scalar_t>
    static std::vector<cv::Point3_<scalar_t>> reconstruct(const cv::Mat_<scalar_t>& depth_image, const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        
        std::vector<cv::Point3_<scalar_t>> points;
        points.reserve(depth_image.total());
        
        assert(depth_image.isContinuous());
        size_t pixel_y, pixel_x;
        scalar_t* z_ptr = reinterpret_cast<scalar_t*>(depth_image.data);
        
        for (pixel_y = 0; pixel_y < depth_image.rows; ++pixel_y) {
            for (pixel_x = 0; pixel_x < depth_image.cols; ++pixel_x, z_ptr++) {
                const scalar_t& z = *z_ptr;
                
                if (not std::isnan(z)) {
                    scalar_t x = z*(pixel_x - image_center.x)/focal_length.x;
                    scalar_t y = z*(pixel_y - image_center.y)/focal_length.y;
                    points.emplace_back(x, y, z);
                } else {
                    points.emplace_back(z, z, z);
                }
                
            }
            
        }
            
        return points;
        
    }
    
    template <typename scalar_t>
    static std::vector<cv::Point3_<scalar_t>> reconstructParallelized(const cv::Mat_<scalar_t>& depth_image, const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        
        assert(depth_image.isContinuous());
        std::vector<cv::Point3_<scalar_t>> points;
        scalar_t nan = std::numeric_limits<scalar_t>::quiet_NaN();
        points.resize(depth_image.total(), cv::Point3_<scalar_t>(nan, nan, nan));
        scalar_t width = depth_image.cols;
        
        depth_image.forEach(
            [&points, &width, &focal_length, &image_center](const float& z, const int* position) -> void {
                size_t pixel_y = position[0];
                size_t pixel_x = position[1];
                size_t index = pixel_y*width + pixel_x;
                
                if (not std::isnan(z)) {
                    scalar_t x = z*(pixel_x - image_center.x)/focal_length.x;
                    scalar_t y = z*(pixel_y - image_center.y)/focal_length.y;
                    points[index] = cv::Point3_<scalar_t>(x, y, z);
                }
            }
        );
            
        return points;
        
    }
    
    template <typename scalar_t>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructPCL(const std::vector<cv::Point2i>& pixels, 
            const cv::Mat_<scalar_t>& depth_image, const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->is_dense = true;
        cloud->reserve(pixels.size());
        
        for (const cv::Point2i& pixel : pixels) {
            size_t index = pixel.y*depth_image.cols + pixel.x;
            
            const scalar_t& z = reinterpret_cast<scalar_t*>(depth_image.data)[index];
            if (not std::isnan(z)) {
                scalar_t x = z*(pixel.x - image_center.x)/focal_length.x;
                scalar_t y = z*(pixel.y - image_center.y)/focal_length.y;
                cloud->points.emplace_back(x, y, z);
            }
            
        }
        
        return cloud;
        
    }
    
    template <typename scalar_t>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructPCL(const cv::Mat_<scalar_t>& depth_image, const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->width = depth_image.cols;
        cloud->height = depth_image.rows;
        cloud->is_dense = false;
        cloud->reserve(depth_image.total());
        
        assert(depth_image.isContinuous());
        size_t pixel_y, pixel_x, index;
        
        for (pixel_y = 0; pixel_y < depth_image.rows; ++pixel_y) {
            for (pixel_x = 0; pixel_x < depth_image.cols; ++pixel_x) {
                index = pixel_y*depth_image.cols + pixel_x;
            
                const scalar_t& z = reinterpret_cast<scalar_t*>(depth_image.data)[index];
                
                if (not std::isnan(z)) {
                    scalar_t x = z*(pixel_x - image_center.x)/focal_length.x;
                    scalar_t y = z*(pixel_y - image_center.y)/focal_length.y;
                    cloud->points.emplace(cloud->begin() + index, x, y, z);
                } else {
                    cloud->points.emplace(cloud->begin() + index, z, z, z);
                }
                
            }
        }
        
        return cloud;
        
    }
    
    template <typename scalar_t>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructPCLParallelized(const cv::Mat_<scalar_t>& depth_image, const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->width = depth_image.cols;
        cloud->height = depth_image.rows;
        cloud->is_dense = false;
        scalar_t nan = std::numeric_limits<scalar_t>::quiet_NaN();
        cloud->points.resize(depth_image.total(), pcl::PointXYZ(nan, nan, nan));

        depth_image.forEach(
            [&cloud, &focal_length, &image_center](const scalar_t& z, const int* position) -> void {
                size_t pixel_y = position[0];
                size_t pixel_x = position[1];
                size_t index = pixel_y*cloud->width + pixel_x;
                
                if (not std::isnan(z)) {
                    scalar_t x = z*(pixel_x - image_center.x)/focal_length.x;
                    scalar_t y = z*(pixel_y - image_center.y)/focal_length.y;
                    cloud->points[index] = pcl::PointXYZ(x, y, z);
                }
            }
        );
        
        return cloud;
    }
    
    template <typename scalar_t>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructPCLParallelized(
            const std::vector<cv::Point2i>& pixel_locations, const cv::Mat_<scalar_t>& depth_image, 
            const cv::Point_<scalar_t>& focal_length, const cv::Point_<scalar_t>& image_center) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->is_dense = false;
        scalar_t nan = std::numeric_limits<scalar_t>::quiet_NaN();
        cloud->points.resize(depth_image.total(), pcl::PointXYZ(nan, nan, nan));
        cv::Mat locations_mat(pixel_locations, false); // convert from vector without copy
        
        locations_mat.forEach<cv::Point2i>(
            [&focal_length, &image_center, &depth_image, &cloud](const cv::Point2i& pixel, const int* position) -> void {
                size_t index = position[0];
                const scalar_t& z = reinterpret_cast<scalar_t*>(depth_image.data)[pixel.y*depth_image.cols + pixel.x];
                
                if (not std::isnan(z)) {
                    scalar_t x = z*(pixel.x - image_center.x)/focal_length.x;
                    scalar_t y = z*(pixel.y - image_center.y)/focal_length.y;
                    cloud->points[index] = pcl::PointXYZ(x, y, z);
                }
                
            }
            
        );
        
        return cloud;
    }
    
}

#endif /* POINT_RECONSTRUCTION_HPP */

