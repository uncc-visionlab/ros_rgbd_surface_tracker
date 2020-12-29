/* file : asf_mapready.i */

/* name of module to use*/
//%module(package="asf_mapready") asf_mapready
%module uncc_rgbd
%{
    /* Every thing in this file is being copied in  
     wrapper file. We include the C header file necessary 
     to compile the interface */
    // in src/asf/asf.h
    //#define MAGIC_UNSET_CHAR '?'
    //#define MAGIC_UNSET_STRING "???"
#include "../include/ros_rgbd_surface_tracker/rgbd_image_uncc.hpp" 
    /* variable declaration*/
RgbdImage(const cv::Mat &_img_I, const cv::Mat &_img_Z, // Mat& _img_L, 
                    float _cx, float _cy, float _f)
    //double myvar; 
    //extern META_DDR_STRUCT meta_ddr_structs[NUM_META_DDR_STRUCTS];
%}
//%include "../include/calibrate.h"
//%include "../include/asf_meta.h"
//%include "../include/asf.h"
//%include "../include/ceos.h"
//%include "../include/ceos_io.h"
//%include "../include/get_ceos_names.h"
//%include "../include/geolocate.h"
//%ignore new_bin_state;
//%ignore delete_bin_state; 
//%rename new_bin_state new_bin_state2;
//%rename delete_bin_state delete_bin_state2;
//%include "../src/libasf_import/decoder.h"

%include cpointer.i
%pointer_functions(int, intp);
//%pointer_functions(readPulseFunc, readPulseFuncp);
//%pointer_functions(meta_parameters, meta_parametersp);
%pointer_functions(double, doublep);
%include <carrays.i>
%array_functions(int, int_array1d);
%array_functions(double, double_array1d);
%array_functions(int *, int_array2d);
%array_functions(double *, double_array2d);
/* Some callback functions */
//%callback("%(uppercase)s");
//void ERS_readNextPulse(bin_state *s, iqType *iqBuf, char *inName, char *outName);
//void JRS_readNextPulse(bin_state *s, iqType *iqBuf, char *inName, char *outName);
//void RSAT_readNextPulse(bin_state *s,iqType *iqBuf, char *inName, char *outName);
//%nocallback;
//%constant(void (*readPulseFunc)(bin_state *s,iqType *iqBuf, char *inN, char *outN)) EERS_readNextCeosPulse = ERS_readNextCeosPulse;
//% constant void ERS_readNextPulse(bin_state *s, iqType *iqBuf, char *inName, char *outName);
//%ignore new_bin_state;
//%ignore delete_bin_state; 
//%rename(new_bin_state) renamed_new_bin_state; 
//%rename(new_bin_state) nnew_bin_state; 

/* explicitly list functions and variables to be interfaced */
//double myvar; 
bool RgbdImage::computeNormals();

/* or if we want to interface all functions then we can simply 
   include header file like this -  
   %include "gfg.h" 
 */
