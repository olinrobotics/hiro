#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include <dynamic_reconfigure/server.h>
#include "irl_stereo/ObjectCriteria.h"
#include "irl_stereo/IRLStereoConfig.h"
#include "irl_stereo/IRLStereoCoarseConfig.h"
#include "irl_stereo/IRLStereoFineConfig.h"
#include "irl_stereo/IRLFramesConfig.h"


using namespace cv;
using namespace std;

class Configuration{
	private:
		ros::NodeHandle nh_global;
		ros::NodeHandle nh_coarse;
		ros::NodeHandle nh_fine;
		ros::NodeHandle nh_frames;

	public:
		irl_stereo::IRLStereoFineConfig c_f;
		irl_stereo::IRLStereoCoarseConfig c_c;
		irl_stereo::IRLFramesConfig c_fr;
	private:
		dynamic_reconfigure::Server<irl_stereo::IRLStereoConfig> s_g;
		dynamic_reconfigure::Server<irl_stereo::IRLStereoFineConfig> s_f;
		dynamic_reconfigure::Server<irl_stereo::IRLStereoCoarseConfig> s_c;
		dynamic_reconfigure::Server<irl_stereo::IRLFramesConfig> s_fr;

	public:
		Configuration();
		ObjectCriteria params;

		bool verbose;
		bool pcl;
		bool coarse;
		float solidity;

		void global_cb(irl_stereo::IRLStereoConfig& config, uint32_t level);
		void coarse_cb(irl_stereo::IRLStereoCoarseConfig& config, uint32_t level);
		void fine_cb(irl_stereo::IRLStereoFineConfig& config, uint32_t level);
		void frames_cb(irl_stereo::IRLFramesConfig& config, uint32_t level);
};

#endif
