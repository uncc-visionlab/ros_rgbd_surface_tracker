
#include <rgbd_drivers_uncc/rgbd_driver.hpp>
#include <rgbd_drivers_uncc/rgbd_driver_client.hpp>

int main(int argc, char **argv) {
    RGBD_OpenCV_ARM_Driver openni2_drvr;

    // hook up driver to surface tracking class
    RgbdDriverClient::Ptr rgbdClientDriverPtr = DemoRgbdDriverClient::create();
    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback(
            boost::bind(&RgbdDriverClient::callback, rgbdClientDriverPtr, _1, _2, _3, _4));

    //RgbdCameraTracker::Ptr rgbdCameraTrackerPtr = cv::rgbd::RgbdSurfaceTracker::create();
    //RgbdCameraTracker::Ptr rgbdCameraTrackerPtr = dvo_ros::DVOCameraDenseTracker::create();
    //boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback(
    //        boost::bind(&RgbdCameraTracker::callback, rgbdCameraTrackerPtr, _1, _2, _3, _4));

    openni2_drvr.setCallback(callback);

    openni2_drvr.initialize();

    openni2_drvr.camera_thread(NULL);

    openni2_drvr.shutdown();
    return 0;
}