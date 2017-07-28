// Sample_Depth_UVC_Viewer.cpp : 
//

//#include "stdafx.h"

#include <openni2/OpenNI.h>


#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sys/time.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

using namespace std;
using namespace cv;

using namespace openni;


cv::Mat ocv_rgbframe;
cv::Mat ocv_depthframe_uint16;
cv::Mat ocv_depthframe_float;
cv::Mat ocv_depth_vis;

Device device;

cv::Mat rgb_distortionCoeffs;
cv::Mat rgb_cameraMatrix;
cv::Size rgb_imSize;

boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback_;

void setCallback(boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&)> const &cb) {
    callback_ = cb;
}

void invokeCallback() {
    callback_(ocv_rgbframe, ocv_depthframe_float, rgb_distortionCoeffs, rgb_cameraMatrix);
}

void convertToFloats(cv::Mat ocv_depthframe_uint16, cv::Mat &ocv_depthframe_float, float scalef) {
    int width = ocv_depthframe_float.cols;
    int height = ocv_depthframe_float.rows;
    float bad_point = std::numeric_limits<float>::quiet_NaN();
    uint16_t* ocv_depthframe_uint16_ptr = (uint16_t *) ocv_depthframe_uint16.data;
    float* ocv_depthframe_float_ptr = (float *) ocv_depthframe_float.data;
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            if (ocv_depthframe_uint16_ptr[row * width + col] == 0) {
                ocv_depthframe_float_ptr[row * width + col] = bad_point;
            } else { // mm to m for kinect
                ocv_depthframe_float_ptr[row * width + col] = ocv_depthframe_uint16_ptr[row * width + col] * scalef;
            }
        }
    }
}

float getDepthUnitInMeters(const PixelFormat& format) {
    switch (format) {
        case PIXEL_FORMAT_DEPTH_100_UM: return 1.f / 10000.f;
        case PIXEL_FORMAT_DEPTH_1_MM: return 1.f / 1000.f;

        default:
            //ntk_error("OpenNI2: Unhandled pixel format: %s\n", getPixelFormatName(format));
            return 1.f;
    }
}
int _frame_skip = 0;

int main(int argc, char** argv) {

    // hook up driver to surface tracking class
    cv::rgbd::RgbdSurfaceTracker rgbdSurfTracker;
    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback(
            boost::bind(&cv::rgbd::RgbdSurfaceTracker::callback, &rgbdSurfTracker, _1, _2, _3, _4));
    callback_ = callback;

    VideoFrameRef oniDepthImg;
    Status result = STATUS_OK;
    result = OpenNI::initialize();

    // open device  
    result = device.open(openni::ANY_DEVICE);

    //// create depth stream 
    VideoStream oniDepthStream;
    result = oniDepthStream.create(device, openni::SENSOR_DEPTH);

    // set depth video mode
    VideoMode modeDepth;
    modeDepth.setResolution(640, 480);
    modeDepth.setFps(30);
    modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
    //modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
    oniDepthStream.setVideoMode(modeDepth);
    oniDepthStream.setMirroringEnabled(false);
    device.setImageRegistrationMode(ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    VideoCapture cap;
    //cap.open(cv::CAP_OPENNI_BGR_IMAGE);
    cap.open(0);
    if (!cap.isOpened()) {
        std::cout << "Cannot access RGB camera!" << std::endl;
        return false;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    // start depth stream
    result = oniDepthStream.start();

    bool rgbFrameReady = false, depthFrameReady = false;

    int count = 0;
    int num_frames_stat = 64;
    struct timeval previous_time;
    gettimeofday(&previous_time, 0);
    uint64_t last_stamp = 0;
    PixelFormat format;
    float scalef;

    cv::Mat di_distortionCoeffs = cv::Mat::zeros(5, 1, CV_32F);
    cv::Mat di_cameraMatrix(3, 3, CV_32F);
    cv::Size di_imSize(0, 0);

    rgb_distortionCoeffs = cv::Mat::zeros(5, 1, CV_32F);
    rgb_cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);

    while (true) {
        bool new_frame = false;
        uint64_t current_stamp = 0;

        Mat frame, gray;
        cap >> frame;
        if (frame.empty())
            break;
        ocv_rgbframe = frame.clone();
        //cv::cvtColor(ocv_rgbframe, ocv_rgbframe, CV_BGR2RGB);

        if (_frame_skip == 0 || !(count % (_frame_skip))) {
            rgbFrameReady = true;
            rgb_imSize.width = ocv_rgbframe.cols;
            rgb_imSize.height = ocv_rgbframe.rows;
            rgb_cameraMatrix.at<float>(0, 0) = rgb_imSize.width / (2 * tan(oniDepthStream.getHorizontalFieldOfView() / 2)); //fx
            rgb_cameraMatrix.at<float>(0, 2) = rgb_imSize.width / 2; //cx
            rgb_cameraMatrix.at<float>(1, 1) = rgb_imSize.height / (2 * tan(oniDepthStream.getVerticalFieldOfView() / 2)); //fy
            rgb_cameraMatrix.at<float>(1, 2) = rgb_imSize.height / 2; //cy
            rgb_cameraMatrix.at<float>(2, 2) = 1;
        }

        if (oniDepthStream.readFrame(&oniDepthImg) == STATUS_OK) {
            const openni::DepthPixel* depthImageBuffer;
            depthImageBuffer = (const openni::DepthPixel*)oniDepthImg.getData();
            if (ocv_depthframe_uint16.cols != oniDepthImg.getWidth() ||
                    ocv_depthframe_uint16.rows != oniDepthImg.getHeight()) {
                ocv_depthframe_uint16.create(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1);
                ocv_depthframe_float.create(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_32FC1);
            }
            memcpy(ocv_depthframe_uint16.data, depthImageBuffer, oniDepthImg.getHeight() * oniDepthImg.getWidth() * sizeof (uint16_t));

            // convert to floats 
            format = oniDepthStream.getVideoMode().getPixelFormat();
            scalef = getDepthUnitInMeters(format);
            convertToFloats(ocv_depthframe_uint16, ocv_depthframe_float, scalef);

            current_stamp = oniDepthImg.getTimestamp();
            if (current_stamp - last_stamp > 5000) {
                count++;
                new_frame = true;
            }
            last_stamp = current_stamp;

            if (_frame_skip == 0 || !(count % (_frame_skip))) {
                depthFrameReady = true;
                di_imSize.width = oniDepthImg.getWidth();
                di_imSize.height = oniDepthImg.getHeight();
                di_cameraMatrix.at<float>(0, 0) = di_imSize.width / (2 * tan(oniDepthStream.getHorizontalFieldOfView() / 2)); //fx
                di_cameraMatrix.at<float>(0, 2) = di_imSize.width / 2; //cx
                di_cameraMatrix.at<float>(1, 1) = di_imSize.height / (2 * tan(oniDepthStream.getVerticalFieldOfView() / 2)); //fy
                di_cameraMatrix.at<float>(1, 2) = di_imSize.height / 2; //cy
                di_cameraMatrix.at<float>(2, 2) = 1;
            }

            if (new_frame) {
                cvtColor(ocv_rgbframe, gray, CV_RGB2GRAY);
                cv::Mat c8BitDepth, c24Bit;
                ocv_depthframe_uint16.convertTo(c8BitDepth, CV_8U, 255.0 / (8000));
                Mat dst;
                addWeighted(c8BitDepth, 0.5, gray, 0.5, 0, dst);
                cv::imshow("Orbbec", dst);
            }
        }
        waitKey(10);

        if (rgbFrameReady && depthFrameReady) {
            // call callback function
            if (callback_) {
                invokeCallback();
            }
            rgbFrameReady = false;
            depthFrameReady = false;
            new_frame = false;
        }

        if (!(count % num_frames_stat) && new_frame) {
            struct timeval current_time, interval;
            gettimeofday(&current_time, 0);
            timersub(&current_time, &previous_time, &interval);
            previous_time = current_time;
            double fps = num_frames_stat / (interval.tv_sec + 1e-6 * interval.tv_usec);
            printf("running at %lf fps\n", fps);
            fflush(stdout);
        }
    }

    oniDepthStream.destroy();

    device.close();
    OpenNI::shutdown();
    return 0;
}

