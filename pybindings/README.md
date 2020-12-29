Excerpt from learnopencv modified to work with OpenCV 3.2

[How to Convert your OpenCV C++ code Into a Python Module](http://www.learnopencv.com/how-to-convert-your-opencv-c-code-into-a-python-module)

mkdir ovex
cd ovex
mkdir src
cp ../opencv-3.1.0/modules/python/src2/{pycompat.hpp,cv2.cpp,gen2.py,hdr_parser.py} ./
cp ../learnopencv-master/pymodule/src/* src/
cp ../learnopencv-master/pymodule/headers.txt ./

Prepare to use pycompat.hpp, cv2.cpp in opencv-3.1.0 source code,gen2.py, hdr_parser.py file, use the opencv c++ source file in the blog above, take your time

Modify gen2.py where you see # ARW 12/23/2020
In gen2.py do a global search an replace of "pyopencv2" with ${modulePrefix}
In strings do "..."+ modulePrefix +"..."
All invocations of substitute() have the following for the first argument .substitute(modulePrefix=modulePrefix, ...
Console commands from root directory (where headers.txt is)


mkdir build
./gen2.py build ./headers.txt 

Modify the cv2.cpp file
define macros
#define PYOPENCV_TO  pybv_to
#define PYOPENCV_FROM  pybv_from
Global replace pyopencv_to -> PYOPENCV_TO
Global replace pyopencv_from -> PYOPENCV_FROM
change the #include files to reference your files in the build folder
fix the MKTYPE2(NAME) macros and substitute your module name

mkdir build
python3 gen2.py build headers.txt 
g++ -shared -rdynamic -g -O3 -Wall -Wno-unused-function -Wno-unused-variable -fPIC cv3.cpp src/bvmodule.cpp -DMODULE_STR=bv -DMODULE_PREFIX=pybv -DNDEBUG -DPY_MAJOR_VERSION=3 `pkg-config --cflags --libs opencv`  `python3-config --includes --ldflags` -I . -I/usr/lib/python3/dist-packages/numpy/core/include -I build -L`/usr/bin/python3-config --exec-prefix`/lib -o bv.so 
python3
import bv
dir(bv)

mkdir build
python2.7 gen2.py build headers.txt 
g++ -shared -rdynamic -g -O3 -Wall -Wno-unused-function -Wno-unused-variable -fPIC cv3.cpp src/bvmodule.cpp -DMODULE_STR=bv -DMODULE_PREFIX=pybv -DNDEBUG -DPY_MAJOR_VERSION=2 `pkg-config --cflags --libs opencv`  `python2.7-config --includes --ldflags` -I . -I/usr/lib/python2.7/dist-packages/numpy/core/include -I build -L`/usr/bin/python2.7-config --exec-prefix`/lib -o bv.so 
python2.7
import bv
dir(bv)
