/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#include "common_sensors/footprint_filter.h"
#include "common_sensors/nan_to_inf_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"

#if ROS_VERSION_MINIMUM(1, 13, 0)
PLUGINLIB_EXPORT_CLASS(sensor_filter::FootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(sensor_filter::NanToInfFilter, filters::FilterBase<sensor_msgs::LaserScan>)
#else
PLUGINLIB_REGISTER_CLASS(sensor_filter/FootprintFilter, sensor_filter::FootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_REGISTER_CLASS(sensor_filter/NanToInfFilter, sensor_filter::NanToInfFilter, filters::FilterBase<sensor_msgs::LaserScan>)
#endif
