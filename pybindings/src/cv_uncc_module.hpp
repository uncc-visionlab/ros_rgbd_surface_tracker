#ifndef BVMODULE_H
#define BVMODULE_H

// header file contents go here...

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace cv_uncc {
    namespace rgbd {
	CV_EXPORTS_W void fillHoles(Mat &mat);

	class CV_EXPORTS_W Filters 
	{
	public:
		CV_WRAP Filters();
		CV_WRAP void edge(InputArray im, OutputArray imedge);
	};
    }
}
#endif // BVMODULE_H

