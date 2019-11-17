#ifndef __RECTIFIER_H__
#define __RECTIFIER_H__

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;

class Rectifier{
	private:
		Mat Q;
		Mat c_l_m1, c_l_m2, c_r_m1, c_r_m2;
	public:
		Rectifier(const string& param_l, const string& param_r); // from yaml
		//Rectifier(const string& topic); //ROS topic
		//Rectifier(); // load from parameter server
		void apply(const Mat& l, const Mat& r, Mat& rect_l, Mat& rect_r);
		void convert(const Mat& disp, Mat& dist);

};

#endif /* __RECTIFIER_H__ */
