/* 
 * File:   SphereDetector.hpp
 * Author: John Papadakis
 *
 * Created on January 25, 2018, 3:15 PM
 */

#ifndef COLOREDSPHEREDETECTOR_HPP
#define COLOREDSPHEREDETECTOR_HPP

#include <vector>
#include <tuple>
#include <map>
#include <cassert>

#include <boost/make_shared.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_types.h>

#include <colored_sphere_detector/PixelColorClassifier.hpp>
#include <colored_sphere_detector/CircleDetector.hpp>
#include <colored_sphere_detector/SphereDetector.hpp>

class ColoredSphereDetector {
public:
        
    PixelColorClassifier pixel_color_classifier;
    CircleDetector circle_detector;
    SphereDetector sphere_detector;
    
    bool visualize = false;

    // Margin of pixels ignored on input image
    size_t margin_x = 0; 
    size_t margin_y = 0;
    
    // Camera parameters needed for point reconstruction
    cv::Point2f focal_length = cv::Point2f(570.34, 570.34);
    cv::Point2f image_center = cv::Point2f(320, 240);
    
    struct ColoredCircleDetection : CircleDetector::CircleDetection {
        Color color;
        ColoredCircleDetection() = default;
        ColoredCircleDetection(const CircleDetector::CircleDetection& circle, Color color) : 
            CircleDetector::CircleDetection(circle), color(color) {}
        
    };
    
    struct ColoredSphereDetection : SphereDetector::SphereDetection {
        Color color;
        ColoredSphereDetection() = default;
        ColoredSphereDetection(const SphereDetector::SphereDetection& sphere, Color color) : 
            SphereDetector::SphereDetection(sphere), color(color) {}
        
    };
    
    ColoredSphereDetector() {    
        
        pixel_color_classifier.color_class_map =  std::map<Color, Gaussian<float, 3>>{
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
        
        pixel_color_classifier.colorful_threshold = .09;
        pixel_color_classifier.color_likelihood_threshold = -8;

        circle_detector.bounding_box_ratio_threshold = .92;
        circle_detector.min_radius = 6; // pixels
        circle_detector.max_radius = 50; // pixels
        circle_detector.circular_fill_ratio_threshold = .8; 
        circle_detector.component_area_ratio_threshold = .9;

        sphere_detector.min_points_for_fitting = 10;
        sphere_detector.ransac_model_distance_threshold = .01;
        sphere_detector.min_radius = .02; // meters
        sphere_detector.max_radius = .045; // meters
        sphere_detector.inlier_percentage_threshold = .6;
        
    }
    
    virtual ~ColoredSphereDetector() {
    }
    
    std::vector<ColoredSphereDetection> rgbd_callback(const cv::Mat& color_input, const cv::Mat& depth_input, const cv::Matx33f& camera_matrix) {
        
        cv::Mat rgb_input;
        cv::cvtColor(color_input, rgb_input, CV_BGR2RGB);
        this->setCameraParametersFromMatrix(camera_matrix);
        
        return this->detect(rgb_input, depth_input);
        
    }
    
    std::vector<ColoredSphereDetection> detect(const cv::Mat& rgb_input, const cv::Mat& depth_input) {
        
        // Assumes rgb_input is has channels RGB in order
        assert(rgb_input.channels() == 3);
        
        // Trim down input image by margin, median blur, convert to float if needed
        cv::Rect roi(margin_x, margin_y, rgb_input.cols - margin_x, rgb_input.rows - margin_y);
        cv::Mat rgb_image = rgb_input(roi).clone();
        cv::medianBlur(rgb_image, rgb_image, 5);
        if (rgb_image.depth() != CV_32F) {
            rgb_image.convertTo(rgb_image, CV_32F);
        }
        
        std::vector<ColoredCircleDetection> colored_circles = detectAppearance(rgb_image);
        
         if (visualize) {
             
            cv::Mat output = rgb_input.clone();
            if (output.type() != CV_8UC3) {
                output.convertTo(output, CV_8UC3);
            }
            this->visualizeCircleDetections(colored_circles, output);
            
        }
        
        std::vector<ColoredSphereDetection> colored_spheres = detectGeometry(colored_circles, depth_input);
        
        colored_sphere_detections = colored_spheres;
        
        return colored_spheres;
        
    }
    
    std::vector<ColoredCircleDetection> detectAppearance(const cv::Mat& rgb_image) {
        
        cv::Mat color_classified_image = pixel_color_classifier.classifyPixelColors(rgb_image);
        
        std::vector<ColoredCircleDetection> colored_circles;
        for (const std::pair<Color, Gaussian<float, 3>>& entry : pixel_color_classifier.color_class_map) {
            // For each color class, compute color mask and run connected components on mask
            
            Color color = entry.first;
            cv::Mat color_mask = color_classified_image == static_cast<size_t>(color);
            std::vector<CircleDetector::CircleDetection> circles = circle_detector.detectCircles(color_mask);
            
            for (CircleDetector::CircleDetection& circle : circles) {
                circle.x = circle.x + margin_x;
                circle.y = circle.y + margin_y;
                colored_circles.emplace_back(circle, color);
            }
            
        }
        
        if (visualize) {
            cv::imshow("Color Classification", this->imagesc(color_classified_image));
        }
        
        return colored_circles;
        
    }
    
    std::vector<ColoredSphereDetection> detectGeometry(std::vector<ColoredCircleDetection> colored_circles, const cv::Mat& depth_image) {
        
        std::vector<ColoredSphereDetection> colored_spheres;
        colored_spheres.reserve(colored_circles.size());
        
        for (const ColoredCircleDetection& colored_circle : colored_circles) {
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr points = 
                    this->reprojectToCloud(colored_circle.locations, depth_image, focal_length, image_center);
            
            bool sphere_detected;
            SphereDetector::SphereDetection sphere;
            
            std::tie(sphere_detected, sphere) = sphere_detector.detectSphere(points); // std::optional if c++17
            
            if (sphere_detected) {
                colored_spheres.emplace_back(sphere, colored_circle.color);
            }
            
        }
        
        return colored_spheres;
        
    }
    
    static std::vector<cv::Point3f> reproject(const std::vector<cv::Point2i>& pixel_locations, 
            const cv::Mat& depth_image, const cv::Point2f& focal_length, const cv::Point2f& center) {
        
        const float& fx = focal_length.x;
        const float& fy = focal_length.y;
        const float& cx = center.x;
        const float& cy = center.y;
        
        std::vector<cv::Point3f> points;
        points.reserve(pixel_locations.size());
        
        for (const cv::Point2i& pixel : pixel_locations) {
            
            const float& z = depth_image.at<float>(pixel.y, pixel.x);
            if (not std::isnan(z)) {
                float x = z*(pixel.x - cx)/fx;
                float y = z*(pixel.y - cy)/fy;
                points.emplace_back(x, y, z);
            }
            
        }
        
        return points;
        
    }
    
    static pcl::PointCloud<pcl::PointXYZ>::Ptr reprojectToCloud(const std::vector<cv::Point2i>& pixel_locations, 
            const cv::Mat& depth_image, const cv::Point2f& focal_length, const cv::Point2f& image_center) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->is_dense = true;
        cloud->reserve(pixel_locations.size());
        
        for (const cv::Point2i& pixel : pixel_locations) {
            
            const float& z = depth_image.at<float>(pixel.y, pixel.x);
            if (not std::isnan(z)) {
                float x = z*(pixel.x - image_center.x)/focal_length.x;
                float y = z*(pixel.y - image_center.y)/focal_length.y;
                cloud->points.emplace_back(x, y, z);
            }
            
        }
        
        cloud->width = cloud->points.size();
        
        return cloud;
        
    }
    
    const std::vector<ColoredSphereDetection>& getColoredSphereDetections() {
        return colored_sphere_detections;
    }
    
private:
    
    std::vector<ColoredSphereDetection> colored_sphere_detections;
    
    void setCameraParametersFromMatrix(const cv::Matx33f& camera_matrix) {
        cv::Point2f focal_length(camera_matrix(0, 0), camera_matrix(1, 1));
        cv::Point2f image_center(camera_matrix(0, 2), camera_matrix(1, 2));
        this->focal_length = focal_length;
        this->image_center = image_center;
    }
    
    void visualizeCircleDetections(const std::vector<ColoredCircleDetection>& detections, const cv::Mat& rgb_image) const {
        
        for (const ColoredCircleDetection& detection : detections) {
            cv::Vec3f colorvec = 255*pixel_color_classifier.color_class_map.at(detection.color).getMean();
            cv::circle(rgb_image, cv::Point2f(detection.x, detection.y), detection.radius, 
                    cv::Scalar(colorvec[0], colorvec[1], colorvec[2]), 2, 1);
            cv::putText(rgb_image, toString(detection.color) + ", " + std::to_string(detection.radius), 
                    cv::Point2i(detection.x - detection.radius, detection.y - detection.radius - 3), 
                    cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(colorvec[0], colorvec[1], colorvec[2]));
        }

        cv::cvtColor(rgb_image, rgb_image, CV_RGB2BGR);
        cv::imshow("Detections", rgb_image);
        cv::waitKey(1);
        
    }
    
    static cv::Mat imagesc(const cv::Mat& input) {
        double min;
        double max;
        cv::minMaxIdx(input, &min, &max);
        
        cv::Mat scaled;
        float scale = 255.0/(max - min);
        input.convertTo(scaled, CV_8UC1, scale, -min*scale);

        cv::Mat false_color;
        cv::applyColorMap(scaled, false_color, cv::COLORMAP_JET);
        
        return false_color;
    }
    
    static std::string toString(Color color) {
        
        switch(color) {
            
            case Color::RED:
                return "red";
                break;
            case Color::GREEN:
                return "green";
                break;
            case Color::BLUE:
                return "blue";
                break;
            case Color::ORANGE:
                return "orange";
                break;
            case Color::YELLOW:
                return "yellow";
                break;
            case Color::OTHER:
                return "other";
                break;
            default:
                throw std::invalid_argument("Invalid color to string conversion requested!");
                
        }
        
    }
    
    static std::tuple<cv::Mat, cv::Mat, cv::Mat> computeMeanAndCovariance(const cv::Mat& points) {
        size_t n = points.rows;
        
        cv::Mat mean;
        cv::reduce(points, mean, 0, CV_REDUCE_AVG);
        cv::Mat zero_centered_points = points - cv::Mat1f::ones(n, 1)*mean;
        cv::Mat cov = (1.0f/(n - 1))*zero_centered_points*zero_centered_points.t();
        
        return std::make_tuple(mean, cov, zero_centered_points);
    }
    
    template <typename scalar_t, int size>
    static cv::Vec<scalar_t, size> vectorProjection(const cv::Vec<scalar_t, size>& a, const cv::Vec<scalar_t, size>& b) {
        return a.dot(b)*cv::normalize(b);
    }
    
    template <typename scalar_t, int size>
    static cv::Vec<scalar_t, size> vectorRejection(const cv::Vec<scalar_t, size>& a, const cv::Vec<scalar_t, size>& b) {
        return a - vectorProjection(a, b);
    }
    
    template <typename scalar_t, int rows, int cols>
    cv::Matx<scalar_t, rows, cols> normalizeColumns(const cv::Matx<scalar_t, rows, cols>& input_matrix) const {
        cv::Matx<scalar_t, rows, cols> normalized;
        
        for (int col = 0; col < cols; ++col) {
            // would love a simple Matx -> Vec conversion
            cv::Matx<scalar_t, rows, 1> output_col = (1.0/cv::norm(input_matrix.col(col)))*input_matrix.col(col);
            for (int row = 0; row < rows; ++row) {
                normalized(row, col) = output_col(row);
            }
        }
        
        return normalized;
    }
    
};

#endif /* COLOREDSPHEREDETECTOR_HPP */