#include <stdio.h>
#include <iostream>
#include <stdlib.h> 
#include <string>
#include <sys/time.h>
#include <unistd.h>

#include <openni2/OpenNI.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

using namespace openni;
using namespace std;

volatile bool run;

//#define USE_THREADS_FOR_DRIVER

class OpenNI2Driver {
private:
    openni::VideoStream** streams;
    VideoStream depth;
    VideoStream rgb;


    int _frame_skip;
    int gain_status;

    cv::Mat ocv_rgbframe;
    cv::Mat ocv_depthframe_uint16;
    cv::Mat ocv_depthframe_float;
    cv::Mat ocv_depth_vis;
    Device device;

    cv::Mat rgb_distortionCoeffs;
    cv::Mat rgb_cameraMatrix;
    cv::Size rgb_imSize;

    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    OpenNI2Driver(const OpenNI2Driver& ref);
    OpenNI2Driver& operator=(const OpenNI2Driver& ref);

    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback_;

public:
    typedef boost::shared_ptr<OpenNI2Driver> Ptr;

    OpenNI2Driver() : _frame_skip(0), gain_status(0) {
    }

    virtual ~OpenNI2Driver() {
    };

    void setCallback(boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&)> const &cb) {
        callback_ = cb;
    }

    void invokeCallback() {
        callback_(ocv_rgbframe, ocv_depthframe_float, rgb_distortionCoeffs, rgb_cameraMatrix);
    }

    int initialize() {
        int _depth_mode = -1;
        int _device_num = -1;
        string _device_uri = "NA";
        int _rgb_mode = -1;
        int _registration = 0;
        int _sync = 0;
        int _exposure = -1;
        int _gain = -1;

        printf("starting\n");
        fflush(stdout);
        _device_num = 0;
        _sync = 0;
        _registration = 1;
        //_depth_mode = -1; // SHOW DEPTH MODES
        //_rgb_mode = -1; // SHOW RGB MODES
        //_depth_mode = 4; // 4: 640x480, 30 fps, 100 format (1mm / depth count)
        _depth_mode = 5; // 4: 640x480, 30 fps, 101 format (100um / depth count)
        _rgb_mode = 9; // 9: 640x480, 30 fps, 200 format (RGB8888 format)
        printf("Launched with params:\n");
        printf("_device_num:= %d\n", _device_num);
        printf("_device_uri:= %s\n", _device_uri.c_str());
        printf("_sync:= %d\n", _sync);
        printf("_registration:= %d\n", _registration);
        printf("_depth_mode:= %d\n", _depth_mode);
        printf("_rgb_mode:= %d\n", _rgb_mode);
        printf("_frame_skip:= %d\n", _frame_skip);
        printf("_exposure:= %d\n", _exposure);
        printf("_gain:= %d\n", _gain);

        fflush(stdout);

        if (_frame_skip <= 0)
            _frame_skip = 1;

        //OPENNI2 STUFF
        //===================================================================
        streams = new openni::VideoStream*[2];
        streams[0] = &depth;
        streams[1] = &rgb;

        Status rc = OpenNI::initialize();
        if (rc != STATUS_OK) {
            printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
            fflush(stdout);
            return 1;
        }

        // enumerate the devices
        openni::Array<openni::DeviceInfo> device_list;
        openni::OpenNI::enumerateDevices(&device_list);

        if (_device_uri.compare("NA")) {

            string dev_uri("NA");

            for (int i = 0; i < device_list.getSize(); i++) {
                if (!string(device_list[i].getUri()).compare(0, _device_uri.size(), _device_uri)) {
                    dev_uri = device_list[i].getUri();
                    break;
                }
                std::cout << device_list[i].getUri();
            }
            if (!dev_uri.compare("NA")) {
                cerr << "cannot find device with uri starting for: " << _device_uri << endl;
            }
            rc = device.open(dev_uri.c_str());
        } else {

            if (_device_num < 0) {
                cerr << endl << endl << "found " << device_list.getSize() << " devices" << endl;
                for (int i = 0; i < device_list.getSize(); i++)
                    cerr << "\t num: " << i << " uri: " << device_list[i].getUri() << endl;
            }

            if (_device_num >= device_list.getSize() || _device_num < 0) {
                cerr << "device num: " << _device_num << " does not exist, aborting" << endl;
                openni::OpenNI::shutdown();
                return 0;
            }

            rc = device.open(device_list[_device_num].getUri());
        }

        if (rc != STATUS_OK) {
            printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
            fflush(stdout);
            return 2;
        }


        if (_depth_mode >= 0) {
            if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
                rc = depth.create(device, SENSOR_DEPTH);
                if (rc != STATUS_OK) {
                    printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
                    fflush(stdout);
                    return 3;
                }
            }
        }

        if (_rgb_mode >= 0) {
            if (device.getSensorInfo(SENSOR_COLOR) != NULL) {
                rc = rgb.create(device, SENSOR_COLOR);
                if (rc != STATUS_OK) {
                    printf("Couldn't create rgb stream\n%s\n", OpenNI::getExtendedError());
                    fflush(stdout);
                    return 3;
                }
            }
        }

        if (_depth_mode < 0 && _rgb_mode < 0) {
            cout << "Depth modes" << endl;
            const openni::SensorInfo* sinfo = device.getSensorInfo(openni::SENSOR_DEPTH); // select index=4 640x480, 30 fps, 1mm
            const openni::Array< openni::VideoMode>& modesDepth = sinfo->getSupportedVideoModes();

            printf("Enums data:\nPIXEL_FORMAT_DEPTH_1_MM = 100,\nPIXEL_FORMAT_DEPTH_100_UM = 101,\nPIXEL_FORMAT_SHIFT_9_2 = 102,\nPIXEL_FORMAT_SHIFT_9_3 = 103,\nPIXEL_FORMAT_RGB888 = 200,\nPIXEL_FORMAT_YUV422 = 201,\nPIXEL_FORMAT_GRAY8 = 202,\nPIXEL_FORMAT_GRAY16 = 203,\nPIXEL_FORMAT_JPEG = 204,\nPIXEL_FORMAT_YUYV = 205,\n\n");
            cout << "Depth modes" << endl;
            for (int i = 0; i < modesDepth.getSize(); i++) {
                printf("%i: %ix%i, %i fps, %i format\n", i, modesDepth[i].getResolutionX(), modesDepth[i].getResolutionY(), modesDepth[i].getFps(), modesDepth[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM = 101
            }

            cout << "Rgb modes" << endl;
            const openni::SensorInfo* sinfoRgb = device.getSensorInfo(openni::SENSOR_COLOR); // select index=4 640x480, 30 fps, 1mm
            const openni::Array< openni::VideoMode>& modesRgb = sinfoRgb->getSupportedVideoModes();
            for (int i = 0; i < modesRgb.getSize(); i++) {
                printf("%i: %ix%i, %i fps, %i format\n", i, modesRgb[i].getResolutionX(), modesRgb[i].getResolutionY(), modesRgb[i].getFps(), modesRgb[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM
            }
            depth.stop();
            depth.destroy();
            rgb.stop();
            rgb.destroy();
            device.close();
            OpenNI::shutdown();
            exit(1);
        }

        if (_depth_mode >= 0) {
            rc = depth.setVideoMode(device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes()[_depth_mode]);
            depth.setMirroringEnabled(false);
            rc = depth.start();
        }
        if (_rgb_mode >= 0) {
            rc = rgb.setVideoMode(device.getSensorInfo(SENSOR_COLOR)->getSupportedVideoModes()[_rgb_mode]);
            rgb.setMirroringEnabled(false);


            rgb.getCameraSettings()->setAutoExposureEnabled(true);
            rgb.getCameraSettings()->setAutoWhiteBalanceEnabled(true);

            cerr << "Camera settings valid: " << rgb.getCameraSettings()->isValid() << endl;
            rc = rgb.start();
        }

        if (_depth_mode >= 0 && _rgb_mode >= 0 && _sync == 1) {
            rc = device.setDepthColorSyncEnabled(true);
            if (rc != STATUS_OK) {
                printf("Couldn't enable de  pth and rgb images synchronization\n%s\n",
                        OpenNI::getExtendedError());
                exit(2);
            }
        }


        if (_depth_mode >= 0 && _rgb_mode >= 0 && _registration == 1) {
            device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        }
        return -1;
    }

    void shutdown() {
        depth.stop();
        depth.destroy();
        rgb.stop();
        rgb.destroy();
        device.close();
        OpenNI::shutdown();

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

    static void* camera_thread_static(void* driver) {
        return ((OpenNI2Driver*) driver)->camera_thread(NULL);
    }

    void* camera_thread(void*) {
        VideoFrameRef frame;
        VideoFrameRef rgbframe;
        int changed_stream;
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

        while (run) {
            bool new_frame = false;
            uint64_t current_stamp = 0;
            openni::OpenNI::waitForAnyStream(streams, 2, &changed_stream);
            switch (changed_stream) {
                    //DEPTH
                case 0:
                    depth.readFrame(&frame);
                    const openni::DepthPixel* depthImageBuffer;
                    depthImageBuffer = (const openni::DepthPixel*)frame.getData();
                    if (ocv_depthframe_uint16.cols != frame.getWidth() ||
                            ocv_depthframe_uint16.rows != frame.getHeight()) {
                        ocv_depthframe_uint16.create(frame.getHeight(), frame.getWidth(), CV_16UC1);
                        ocv_depthframe_float.create(frame.getHeight(), frame.getWidth(), CV_32FC1);
                    }
                    memcpy(ocv_depthframe_uint16.data, depthImageBuffer, frame.getHeight() * frame.getWidth() * sizeof (uint16_t));

                    // convert to floats 
                    format = depth.getVideoMode().getPixelFormat();
                    scalef = getDepthUnitInMeters(format);
                    convertToFloats(ocv_depthframe_uint16, ocv_depthframe_float, scalef);
                    cv::normalize(ocv_depthframe_uint16, ocv_depth_vis, 0, 255, CV_MINMAX);
                    ocv_depth_vis.convertTo(ocv_depth_vis, CV_8UC3);
                    cv::imshow("DEPTH", ocv_depth_vis);
                    cv::waitKey(3);
                    if (_frame_skip == 0 || !(count % (_frame_skip))) {
                        depthFrameReady = true;
                        di_imSize.width = frame.getWidth();
                        di_imSize.height = frame.getHeight();
                        di_cameraMatrix.at<float>(0, 0) = di_imSize.width / (2 * tan(depth.getHorizontalFieldOfView() / 2)); //fx
                        di_cameraMatrix.at<float>(0, 2) = di_imSize.width / 2; //cx
                        di_cameraMatrix.at<float>(1, 1) = di_imSize.height / (2 * tan(depth.getVerticalFieldOfView() / 2)); //fy
                        di_cameraMatrix.at<float>(1, 2) = di_imSize.height / 2; //cy
                        di_cameraMatrix.at<float>(2, 2) = 1;
                        //depth.stop();
                    }
                    break;

                    //RGB
                case 1:
                    rgb.readFrame(&rgbframe);
                    if (rgbframe.isValid()) {
                        const openni::RGB888Pixel* rgbImageBuffer;
                        rgbImageBuffer = (const openni::RGB888Pixel*)rgbframe.getData();
                        if (ocv_rgbframe.cols != rgbframe.getWidth() ||
                                ocv_rgbframe.rows != rgbframe.getHeight()) {
                            ocv_rgbframe.create(rgbframe.getHeight(), rgbframe.getWidth(), CV_8UC3);
                        }
                        memcpy(ocv_rgbframe.data, rgbImageBuffer, 3 * rgbframe.getHeight() * rgbframe.getWidth() * sizeof (uint8_t));

                        cv::cvtColor(ocv_rgbframe, ocv_rgbframe, CV_BGR2RGB); //this will put colors right
                        cv::imshow("RGB", ocv_rgbframe);
                        cv::waitKey(3);
                    }
                    if (!rgbframe.isValid())
                        break;
                    current_stamp = rgbframe.getTimestamp();

                    if (current_stamp - last_stamp > 5000) {
                        count++;
                        new_frame = true;
                    }
                    last_stamp = current_stamp;
#if 0
                    if (_gain >= 0) {
                        if (count > 64 && gain_status == 0) {
                            rgb.getCameraSettings()->setAutoExposureEnabled(false);
                            rgb.getCameraSettings()->setAutoWhiteBalanceEnabled(false);
                            rgb.getCameraSettings()->setExposure(1);
                            rgb.getCameraSettings()->setGain(100);
                            gain_status = 1;
                        } else if (count > 128 && gain_status == 1) {
                            rgb.getCameraSettings()->setExposure(_exposure);
                            rgb.getCameraSettings()->setGain(_gain);
                            gain_status = 2;
                        }
                    }
#endif           
                    if (_frame_skip == 0 || !(count % (_frame_skip))) {
                        rgbFrameReady = true;
                        rgb_imSize.width = rgbframe.getWidth();
                        rgb_imSize.height = rgbframe.getHeight();
                        rgb_cameraMatrix.at<float>(0, 0) = rgb_imSize.width / (2 * tan(rgb.getHorizontalFieldOfView() / 2)); //fx
                        rgb_cameraMatrix.at<float>(0, 2) = rgb_imSize.width / 2; //cx
                        rgb_cameraMatrix.at<float>(1, 1) = rgb_imSize.height / (2 * tan(rgb.getVerticalFieldOfView() / 2)); //fy
                        rgb_cameraMatrix.at<float>(1, 2) = rgb_imSize.height / 2; //cy
                        rgb_cameraMatrix.at<float>(2, 2) = 1;
                        //rgb.stop();
                    }
                    break;
                default:
                    printf("Error in wait\n");
            }
            if (rgbFrameReady && depthFrameReady) {
                // call callback function
                if (callback_) {
                    invokeCallback();
                }
                rgbFrameReady = false;
                depthFrameReady = false;
                //depth.start();
                //rgb.start();
            }
            if (!(count % num_frames_stat) && new_frame) {
                struct timeval current_time, interval;
                gettimeofday(&current_time, 0);
                timersub(&current_time, &previous_time, &interval);
                previous_time = current_time;
                double fps = num_frames_stat / (interval.tv_sec + 1e-6 * interval.tv_usec);
                printf("** FRAMERATE STATS **** --> Running at %lf fps\n", fps);
                fflush(stdout);
            }
        }
    }
};

int main(int argc, char **argv) {
    OpenNI2Driver openni2_drvr;

    // hook up driver to surface tracking class
    cv::rgbd::RgbdSurfaceTracker rgbdSurfTracker;
    boost::function<void (cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&) > callback(
            boost::bind(&cv::rgbd::RgbdSurfaceTracker::callback, &rgbdSurfTracker, _1, _2, _3, _4));

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

