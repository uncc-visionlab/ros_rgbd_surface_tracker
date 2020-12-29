/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   surface_fitting.hpp
 * Author: arwillis
 *
 * Created on July 13, 2017, 12:47 PM
 */

#ifndef SURFACE_FITTING_HPP
#define SURFACE_FITTING_HPP

#ifdef __cplusplus

#include <vector>

#include <opencv2/core.hpp>

#include <boost/make_shared.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <ros_rgbd_surface_tracker/opencv_geom_uncc.hpp>

namespace cv {
    namespace uncc {

        class SurfaceFitting {
            
            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneExplicitLeastSquares(const scalar_t* points, size_t num_points, size_t stride) {

//                cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::traits::Type<scalar_t>::value);
//                cv::Mat _Z = cv::Mat::zeros(num_points, 1, cv::traits::Type<scalar_t>::value);
                cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::DataType<scalar_t>::type);
                cv::Mat _Z = cv::Mat::zeros(num_points, 1, cv::DataType<scalar_t>::type);
                scalar_t* M = _M.ptr<scalar_t>();
                scalar_t* Z = _Z.ptr<scalar_t>();

                for (size_t ptIdx = 0; ptIdx < num_points; ++ptIdx) {
                    size_t pt_begin = stride*ptIdx;
                    M[3 * ptIdx] = points[pt_begin];
                    M[3 * ptIdx + 1] = points[pt_begin + 1];
                    M[3 * ptIdx + 2] = 1.0;
                    Z[ptIdx] = points[pt_begin + 2];
                }
                cv::Mat _MtM = _M.t() * _M;
                cv::Mat _planeCoeffs = _MtM.inv() * _M.t() * _Z;
                scalar_t* planeCoeffs = _planeCoeffs.ptr<scalar_t>();

                cv::Plane3_<scalar_t> plane3;
                plane3.x = planeCoeffs[0];
                plane3.y = planeCoeffs[1];
                plane3.z = -1;
                plane3.d = planeCoeffs[2];
                scalar_t normScale = 1.0 / sqrt(plane3.x * plane3.x + plane3.y * plane3.y + plane3.z * plane3.z);
                plane3.scale(normScale);

                return plane3;

            }
            
            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneImplicitLeastSquares(const scalar_t* points, size_t num_points, size_t stride) {

                cv::Plane3_<scalar_t> plane3;

//                cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::traits::Type<scalar_t>::value);
                cv::Mat _M = cv::Mat::zeros(num_points, 3, cv::DataType<scalar_t>::type);
                scalar_t* M = _M.ptr<scalar_t>();
                cv::Point3f centroid(0, 0, 0);
                for (size_t ptIdx = 0; ptIdx < num_points; ++ptIdx) {
                    size_t pt_begin = stride*ptIdx;
                    centroid.x += points[pt_begin];
                    centroid.y += points[pt_begin + 1];
                    centroid.z += points[pt_begin + 2];
                }
                centroid.x /= num_points;
                centroid.y /= num_points;
                centroid.z /= num_points;
                for (size_t ptIdx = 0; ptIdx < num_points; ++ptIdx) {
                    size_t pt_begin = stride*ptIdx;
                    M[3 * ptIdx] = points[pt_begin] - centroid.x;
                    M[3 * ptIdx + 1] = points[pt_begin + 1] - centroid.y;
                    M[3 * ptIdx + 2] = points[pt_begin + 2] - centroid.z;
                }
                cv::Mat _MtM = _M.t() * _M;
                cv::Mat eigVals, eigVecs;
                cv::eigen(_MtM, eigVals, eigVecs);
                cv::Mat _planeCoeffs = eigVecs.row(2).t();

                plane3.x = _planeCoeffs.at<scalar_t>(0);
                plane3.y = _planeCoeffs.at<scalar_t>(1);
                plane3.z = _planeCoeffs.at<scalar_t>(2);
                plane3.d = -(plane3.x * centroid.x + plane3.y * centroid.y + plane3.z * centroid.z);
                
                plane3.scale((plane3.d > 0) ? -1.0 : 1.0);

                return plane3;

            }
            
        public:
            
            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneExplicitLeastSquares(const std::vector<cv::Point3_<scalar_t>>& points) {
                return fitPlaneExplicitLeastSquares(reinterpret_cast<const scalar_t*>(points.data()), points.size(), 3);
            }

            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneExplicitLeastSquares(const Eigen::Matrix<scalar_t, 4, Eigen::Dynamic, Eigen::ColMajor>& points_homogeneous) {
                return fitPlaneExplicitLeastSquares(points_homogeneous.data(), points_homogeneous.cols(), 4);
            }
            
            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneImplicitLeastSquares(const std::vector<cv::Point3_<scalar_t>>& points) {
                return fitPlaneImplicitLeastSquares(reinterpret_cast<const scalar_t*>(points.data()), points.size(), 3);
            }

            template <typename scalar_t>
            static cv::Plane3_<scalar_t> fitPlaneImplicitLeastSquares(const Eigen::Matrix<scalar_t, 4, Eigen::Dynamic, Eigen::ColMajor>& points_homogeneous) {
                return fitPlaneImplicitLeastSquares(points_homogeneous.data(), points_homogeneous.cols(), 4);
            }
            
            static cv::Plane3f fitPlaneRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr points, double distance_threshold = .01, size_t max_iterations = 1000, bool refine = true) {
        
                pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model = 
                        boost::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(points);
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);

                ransac.setDistanceThreshold(distance_threshold);
                ransac.setMaxIterations(max_iterations);
                ransac.computeModel();

                if (refine) {
                    ransac.refineModel();
                }

                Eigen::VectorXf coeffs;
                ransac.getModelCoefficients(coeffs);
        //        std::vector<int> inlier_indicies;
        //        ransac.getInliers(inlier_indicies);

                cv::Plane3f plane(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
                plane.scale((plane.z > 0) ? -1.0 : 1.0);

                return plane;

            }
            
            static std::vector<cv::Plane3f> planeExtractionRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr points, double distance_threshold = .01, size_t max_iterations = 1000, bool refine = true) {

                std::vector<cv::Plane3f> planes;

                pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
                pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
                pcl::SACSegmentation<pcl::PointXYZ> segmentation;
                pcl::ExtractIndices<pcl::PointXYZ> extract;

                segmentation.setModelType(pcl::SACMODEL_PLANE);
                segmentation.setMethodType(pcl::SAC_RANSAC);
                segmentation.setDistanceThreshold(distance_threshold);
                segmentation.setOptimizeCoefficients(refine);
                segmentation.setMaxIterations(max_iterations);

                // Create pointcloud to publish inliers
        //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                size_t original_size = points->size();
                size_t num_planes = 0;
                float min_percentage = .10;
                while (points->size() > .5*original_size) {
                    // Fit a plane
                    segmentation.setInputCloud(points);
                    segmentation.segment(*inliers, *coefficients);

                    if (inliers->indices.size() < min_percentage*original_size) {
                         VERBOSE_SURFTRACK(std::cout << "plane only explains " 
                                 << float(inliers->indices.size())/original_size;)
                        break;
                    }

                    planes.emplace_back(coefficients->values[0], coefficients->values[1], 
                            coefficients->values[2], coefficients->values[3]);
                    cv::Plane3f& plane = planes.back();
                    plane.scale((plane.z > 0) ? -1.0 : 1.0);

                    VERBOSE_SURFTRACK(std::cout << "Fit plane: " << plane << 
                            ", percentage of points: " << 
                            float(inliers->indices.size())/original_size << std::endl);

                    // Extract inliers
                    extract.setInputCloud(points);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    pcl::PointCloud<pcl::PointXYZ> filtered_points;
                    extract.filter(filtered_points); // result contains the outliers
                    points->swap(filtered_points);

                    num_planes++;
                }

                // Publish points
        //        sensor_msgs::PointCloud2 cloud_publish;
        //        pcl::toROSMsg(*output_cloud, cloud_publish);
        //        cloud_publish.header = msg->header;
        //        _pub_inliers.publish(cloud_publish);

                return planes;

            }
            
        };
    } /* namespace uncc */
} /* namespace cv */

#endif /* __cplusplus */
#endif /* SURFACE_FITTING_HPP */

