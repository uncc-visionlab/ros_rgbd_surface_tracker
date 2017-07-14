/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <vector>

#include <opencv2/core.hpp>

#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

namespace cv {
    namespace rgbd {

        void SurfaceDetector::detect(const cv::rgbd::RgbdImage& rgbd_img,
                std::vector<AlgebraicSurfacePatch>& geometries,
                cv::Mat mask) const {
            rgbd_img.computeNormals();
            
            cv::Size sz(rgbd_img.getDepth().size());
            int lv_f = 4;
            // *** Pad image such that width and height are powers of 2^level on all scales (except last)
            int padw = 0, padh = 0;
            int scfct = std::pow(2, lv_f); // enforce restless division by this number on coarsest scale
            //if (hasinfile) scfct = pow(2,lv_f+1); // if initialization file is given, make sure that size is restless divisible by 2^(lv_f+1) !
            int div = sz.width % scfct;
            if (div > 0) padw = scfct - div;
            div = sz.height % scfct;
            if (div > 0) padh = scfct - div;
//            if (padh > 0 || padw > 0) {
//                copyMakeBorder(img_ao_mat, img_ao_mat, floor((float) padh / 2.0f), ceil((float) padh / 2.0f), floor((float) padw / 2.0f), ceil((float) padw / 2.0f), cv::BORDER_REPLICATE);
//                copyMakeBorder(img_bo_mat, img_bo_mat, floor((float) padh / 2.0f), ceil((float) padh / 2.0f), floor((float) padw / 2.0f), ceil((float) padw / 2.0f), cv::BORDER_REPLICATE);
//            }
//            sz = img_ao_mat.size(); // padded image size, ensures divisibility by 2 on all scales (except last)


        }
    } /* namespace rgbd */
} /* namespace cv */