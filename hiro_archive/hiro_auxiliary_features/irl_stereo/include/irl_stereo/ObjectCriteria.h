#ifndef __OBJECT_CRITERIA_H__
#define __OBJECT_CRITERIA_H__

#include <vector>
#include <opencv2/core/core.hpp>

using namespace cv;

template<typename T>
struct MinMaxRange{
	T min;
	T max;
};

struct ObjectCriteria{
	ObjectCriteria();

	// color
	std::vector<MinMaxRange<Vec3b>> hsv_range;

	// distance
	MinMaxRange<float> dist_range;

	MinMaxRange<float> area_range;
};

#endif
