#ifndef __FILTERED_SGBM_H__
#define __FILTERED_SGBM_H__

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

class FilteredSGBM{
	private:
		Mat left_disp, right_disp;
		Ptr<StereoSGBM> left_matcher;
		Ptr<StereoMatcher> right_matcher;
		Ptr<DisparityWLSFilter> wls_filter;
	public:
		FilteredSGBM();
		void compute(const Mat& left, const Mat& right, Mat& filtered_disp, Mat* raw_disp=nullptr);
};

#endif
