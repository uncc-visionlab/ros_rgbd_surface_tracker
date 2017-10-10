#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include <rgbd_drivers_uncc/rgbd_driver.hpp>
#include <rgbd_drivers_uncc/rgbd_tracker.hpp>
#include <rgbd_drivers_uncc/rgbd_tracker_dvo.hpp>

#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>


int main(int argc, char **argv) {
    int numBagfiles = argc - 1;
    if (numBagfiles < 1) {
        std::cout << "Please specify the input bag file as an argument." << std::endl;
        return 0;
    }

    ros::Time::init();

    std::cout << "Opening bag file " << 1
            << ": " << argv[1] << " for reading ...." << std::endl;
    RGBD_BagFile_Driver bagReader(argv[1]);

    RgbdCameraTracker::Ptr rgbdCameraTrackerPtr = cv::rgbd::RgbdSurfaceTracker::create();
    //RgbdCameraTracker::Ptr rgbdCameraTrackerPtr = dvo_ros::DVOCameraDenseTracker::create();
    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback(
            boost::bind(&RgbdCameraTracker::callback, rgbdCameraTrackerPtr, _1, _2, _3, _4));

    bagReader.setCallback(callback);

    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    //topics.push_back(std::string("/camera/rgb/input_image"));
    topics.push_back(std::string("/camera/rgb/camera_info"));
    //topics.push_back(std::string("/camera/depth_registered/input_image"));
    topics.push_back(std::string("/camera/depth/image"));
    bagReader.setTopics(topics);

    ros::Time myTime = ros::Time::now();
    bagReader.readRGBDMessages();

    std::cout << "Closing bag file " << 1
            << ": " << argv[1] << " for reading ...." << std::endl;
}