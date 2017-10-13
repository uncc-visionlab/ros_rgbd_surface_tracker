#include <rgbd_drivers_uncc/rgbd_driver.hpp>
#include <uncc_rgbd_slam/rgbd_tracker.hpp>
#include <uncc_rgbd_slam/rgbd_tracker_dvo.hpp>

#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

extern volatile bool run;

//#define USE_THREADS_FOR_DRIVER

int main(int argc, char **argv) {
    RGBD_OpenCV_Driver openni2_drvr;

    // hook up driver to surface tracking class
    RgbdCameraTracker::Ptr rgbdCameraTrackerPtr = cv::rgbd::RgbdSurfaceTracker::create();
    //RgbdCameraTracker::Ptr rgbdCameraTrackerPtr = dvo_ros::DVOCameraDenseTracker::create();
    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback(
            boost::bind(&RgbdCameraTracker::callback, rgbdCameraTrackerPtr, _1, _2, _3, _4));

    openni2_drvr.setCallback(callback);
    openni2_drvr.initialize();
    run = true;
#ifdef USE_THREADS_FOR_DRIVER
    pthread_t runner;
    pthread_create(&runner, NULL, &OpenNI2Driver::camera_thread_static, &openni2_drvr);
    while (true) {
        usleep(100000);
    }
    void* result;
    run = false;
    pthread_join(runner, &result);
#else
    openni2_drvr.camera_thread(NULL);
#endif
    openni2_drvr.shutdown();
    return 0;
}