#include <opencv2/core/core.hpp>
#include <vector>
#include "irl_stereo/ObjectCriteria.h"

using namespace cv;

ObjectCriteria::ObjectCriteria(){
	area_range.min = 0.003;
	area_range.max = 1.0;
	dist_range.min = 0.0;
	dist_range.max = 1.5;

	MinMaxRange<Vec3b> hsv1;
	hsv1.min = Vec3b(0,0,0);
	hsv1.max = Vec3b(180,255,255);

	MinMaxRange<Vec3b> hsv2;
	hsv2.min = Vec3b(0,0,0);
	hsv2.max = Vec3b(180,255,255);

	hsv_range.push_back(hsv1);
	hsv_range.push_back(hsv2);
}
