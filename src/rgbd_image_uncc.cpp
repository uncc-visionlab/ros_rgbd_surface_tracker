/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
namespace cv {
    namespace rgbd {

        // force construction of the static class member variable
        DepthIntegralImages RgbdImage::iImgs;
        
        bool RgbdImage::staticDataOK() {
            return cv::rgbd::RgbdImage::iImgs.getHeight() == height &&
                    cv::rgbd::RgbdImage::iImgs.getWidth() == width;
        }
    } /* namespace rgbd */
} /* namespace cv */

