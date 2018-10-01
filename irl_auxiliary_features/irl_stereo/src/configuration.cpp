
/* STD */
#include <iostream>
#include <string>
#include <vector>

/* OpenCV */
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

/* ROS */
#include "ros/ros.h"
#include "ros/package.h"
#include "irl_stereo/configuration.h"

Configuration::Configuration():
	nh_global("~"), nh_coarse("~/coarse"), nh_fine("~/fine"), nh_frames("~/frames"),
	s_g(nh_global), s_c(nh_coarse), s_f(nh_fine), s_fr(nh_frames)
{

	// global
	dynamic_reconfigure::Server<irl_stereo::IRLStereoConfig>::CallbackType f_g;
	f_g = boost::bind(&Configuration::global_cb, this, _1, _2);
	s_g.setCallback(f_g);

	//coarse
	dynamic_reconfigure::Server<irl_stereo::IRLStereoCoarseConfig>::CallbackType f_c;
	f_c = boost::bind(&Configuration::coarse_cb, this, _1, _2);
	s_c.setCallback(f_c);

	// fine	
	dynamic_reconfigure::Server<irl_stereo::IRLStereoFineConfig>::CallbackType f_f;
	f_f = boost::bind(&Configuration::fine_cb, this, _1, _2);
	s_f.setCallback(f_f);

	dynamic_reconfigure::Server<irl_stereo::IRLFramesConfig>::CallbackType f_fr;
	f_fr = boost::bind(&Configuration::frames_cb, this, _1, _2);
	s_fr.setCallback(f_fr);
}

void Configuration::global_cb(irl_stereo::IRLStereoConfig& config, uint32_t level){
	verbose = config.verbose;

	if(verbose){
		//namedWindow("left", WINDOW_AUTOSIZE);
		//namedWindow("right", WINDOW_AUTOSIZE);
		//namedWindow("raw_disp", WINDOW_AUTOSIZE);
		//namedWindow("disp", WINDOW_AUTOSIZE);
		//namedWindow("filtered", WINDOW_AUTOSIZE);
	}else{
		destroyAllWindows();
	}

	pcl = config.pcl;

	if(coarse != config.coarse){
		coarse = config.coarse;
		if(coarse){
			coarse_cb(this->c_c, 0);
		}else{
			fine_cb(this->c_f, 0);
		}
	}
	solidity = config.solidity;
}

void Configuration::coarse_cb(irl_stereo::IRLStereoCoarseConfig& config, uint32_t level){
	this->c_c = config;
	if( !coarse)
		return;

	auto& hsv1_l = params.hsv_range[0].min;
	auto& hsv1_h = params.hsv_range[0].max;
	auto& hsv2_l = params.hsv_range[1].min;
	auto& hsv2_h = params.hsv_range[1].max;

	switch(config.color){
		case irl_stereo::IRLStereoCoarse_Red: // Red
			// H
			hsv1_l[0] = 0;
			hsv1_h[0] = 10;
			hsv2_l[0] = 165;
			hsv2_h[0] = 180;

			// S
			hsv1_l[1] = hsv2_l[1] = 210;
			hsv1_h[1] = hsv2_h[1] = 255;

			// V
			hsv1_l[2] = hsv2_l[2] = 100;
			hsv1_h[2] = hsv2_h[2] = 200;
			break;
		case irl_stereo::IRLStereoCoarse_Green: // Green
			// H
			hsv1_l[0] = hsv2_l[0] = 45;
			hsv1_h[0] = hsv2_h[0] = 80;

			// S
			hsv1_l[1] = hsv2_l[1] = 40;
			hsv1_h[1] = hsv2_h[1] = 190;

			// V
			hsv1_l[2] = hsv2_l[2] = 140;
			hsv1_h[2] = hsv2_h[2] = 215;
			break;
		case irl_stereo::IRLStereoCoarse_Blue: // Blue
			// H
			hsv1_l[0] = hsv2_l[0] = 100;
			hsv1_h[0] = hsv2_h[0] = 120;

			// S
			hsv1_l[1] = hsv2_l[1] = 120;
			hsv1_h[1] = hsv2_h[1] = 200;

			// V
			hsv1_l[2] = hsv2_l[2] = 0;
			hsv1_h[2] = hsv2_h[2] = 100;
			break;
		case irl_stereo::IRLStereoCoarse_Orange: // Orange
			// H
			hsv1_l[0] = hsv2_l[0] = 0;
			hsv1_h[0] = hsv2_h[0] = 16;

			// S
			hsv1_l[1] = hsv2_l[1] = 145;
			hsv1_h[1] = hsv2_h[1] = 255;

			// V
			hsv1_l[2] = hsv2_l[2] = 150;
			hsv1_h[2] = hsv2_h[2] = 255;
			break;
		case irl_stereo::IRLStereoCoarse_Yellow: // Yellow
			// H
			hsv1_l[0] = hsv2_l[0] = 20;
			hsv1_h[0] = hsv2_h[0] = 30;

			// S
			hsv1_l[1] = hsv2_l[1] = 95;
			hsv1_h[1] = hsv2_h[1] = 190;

			// V
			hsv1_l[2] = hsv2_l[2] = 100;
			hsv1_h[2] = hsv2_h[2] = 255;
			break;
		case irl_stereo::IRLStereoCoarse_Purple: // Violet
			// H
			hsv1_l[0] = hsv2_l[0] = 130;
			hsv1_h[0] = hsv2_h[0] = 150;

			// S
			hsv1_l[1] = hsv2_l[1] = 80;
			hsv1_h[1] = hsv2_h[1] = 150;

			// V
			hsv1_l[2] = hsv2_l[2] = 0;
			hsv1_h[2] = hsv2_h[2] = 100;
			break;
		case irl_stereo::IRLStereoCoarse_Pink: // Hot Pink
			// H
			hsv1_l[0] = hsv2_l[0] = 160;
			hsv1_h[0] = hsv2_h[0] = 180;

			// S
			hsv1_l[1] = hsv2_l[1] = 160;
			hsv1_h[1] = hsv2_h[1] = 220;

			// V
			hsv1_l[2] = hsv2_l[2] = 150;
			hsv1_h[2] = hsv2_h[2] = 255;
			break;
		case irl_stereo::IRLStereoCoarse_Black: // Black
			// H
			hsv1_l[0] = hsv2_l[0] = 0;
			hsv1_h[0] = hsv2_h[0] = 180;

			// S
			hsv1_l[1] = hsv2_l[1] = 0;
			hsv1_h[1] = hsv2_h[1] = 255;

			// V
			hsv1_l[2] = hsv2_l[2] = 0;
			hsv1_h[2] = hsv2_h[2] = 35;
			break;

	}

	switch(config.size){
		case irl_stereo::IRLStereoCoarse_Small:
			params.area_range.min = 0.0004;
			params.area_range.max = 0.001;
			break;
		case irl_stereo::IRLStereoCoarse_Medium:
			params.area_range.min = 0.001;
			params.area_range.max = 0.002;
			break;
		case irl_stereo::IRLStereoCoarse_Large:
			params.area_range.min = 0.002;
			params.area_range.max = 0.004;
			break;
		case irl_stereo::IRLStereoCoarse_ExtraLarge:
			params.area_range.min = 0.008;
			params.area_range.max = 0.02;
			break;
	}

	// not exactly exposed :)
	params.dist_range.min = 0.0;
	params.dist_range.max = 2.0;
}

void Configuration::fine_cb(irl_stereo::IRLStereoFineConfig& config, uint32_t level){

	this->c_f = config;
	if(coarse)
		return;

	auto& hsv1_l = params.hsv_range[0].min;
	auto& hsv1_h = params.hsv_range[0].max;
	auto& hsv2_l = params.hsv_range[1].min;
	auto& hsv2_h = params.hsv_range[1].max;
	
	if(config.h_l < config.h_h){
		hsv1_l[0] = hsv2_l[0] = config.h_l;
		hsv1_h[0] = hsv2_h[0] = config.h_h;
	}else{
		// wrap around
		hsv1_l[0] = 0;
		hsv1_h[0] = config.h_h;

		hsv2_l[0] = config.h_l;
		hsv2_h[0] = 180;
	}

	hsv1_l[1] = config.s_l;
	hsv1_h[1] = config.s_h;
	hsv1_l[2] = config.v_l;
	hsv1_h[2] = config.v_h;

	hsv2_l[1] = config.s_l;
	hsv2_h[1] = config.s_h;
	hsv2_l[2] = config.v_l;
	hsv2_h[2] = config.v_h;

	params.area_range.min = config.min_area;
	params.area_range.max = config.max_area;
	params.dist_range.min = config.min_dist;
	params.dist_range.max = config.max_dist;
}

void Configuration::frames_cb(irl_stereo::IRLFramesConfig& config, uint32_t level){
	this->c_fr = config;
	if(!config.left)
		destroyWindow("left");
	if(!config.right)
		destroyWindow("right");
	if(!config.disp)
		destroyWindow("disp");
	if(!config.raw_disp)
		destroyWindow("raw_disp");
	if(!config.filtered)
		destroyWindow("filtered");
}
