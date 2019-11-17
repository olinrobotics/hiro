#include <string>
#include <iostream>

#include "irl_stereo/rectifier.h"

Mat get_matrix(const YAML::Node& node){
	int cols = node["cols"].as<int>();
	int rows = node["rows"].as<int>();
	auto m_base = Mat_<float>(rows,cols);
	std::vector<float> data;
	//data.reserve(cols*rows);

	for (const auto& d : node["data"]){
		data.push_back(d.as<float>());	
	}
	auto m = Mat(rows,cols,CV_32F);
	memcpy(m.data, data.data(), data.size() * sizeof(float));
	return m;
}

void get_matrices(const YAML::Node& config, Mat& m, Mat& d, Mat& r, Mat& p){
	m = get_matrix(config["camera_matrix"]);
	d = get_matrix(config["distortion_coefficients"]);
	r = get_matrix(config["rectification_matrix"]);
	p = get_matrix(config["projection_matrix"]);
}

Rectifier::Rectifier(const string& param_l, const string& param_r){

	YAML::Node config_l = YAML::LoadFile(param_l);
	YAML::Node config_r = YAML::LoadFile(param_r);

	Mat m_l, d_l, r_l, p_l, \
		m_r, d_r, r_r, p_r;

	get_matrices(config_l, m_l, d_l, r_l, p_l);
	get_matrices(config_r, m_r, d_r, r_r, p_r);

	int width = config_l["image_width"].as<int>();
	int height = config_l["image_height"].as<int>();

	double fx = p_l.at<float>(0,0);
	double cx = p_l.at<float>(0,2);
	double cy = p_l.at<float>(1,2);
	double fy = p_l.at<float>(1,1);
	double Tx = p_r.at<float>(0,3) / p_r.at<float>(0,0);
	double rcx = p_r.at<float>(0,2);

	//printf("fx: %f; cx: %f; cy: %f; fy: %f; Tx: %f; rcx: %f\n", fx,cx,cy,fy,Tx,rcx);

	Q = (Mat_<double>(4,4,CV_64F) << \
		fy*Tx, 0, 0, -fy*cx*Tx, \
		0, fx*Tx, 0, -fx*cy*Tx, \
		0, 0,     0,  fx*fy*Tx, \
		0, 0,   -fy,  fy*(cx-rcx));

	cv::initUndistortRectifyMap(m_l,d_l,r_l,p_l,Size(width,height),CV_32F, c_l_m1, c_l_m2);
	cv::initUndistortRectifyMap(m_r,d_r,r_r,p_r,Size(width,height),CV_32F, c_r_m1, c_r_m2);
}

void Rectifier::apply(const Mat& l, const Mat& r, Mat& rect_l, Mat& rect_r){
	remap(l,rect_l,c_l_m1,c_l_m2,INTER_LINEAR); 
	remap(r,rect_r,c_r_m1,c_r_m2,INTER_LINEAR); 
}

void Rectifier::convert(const Mat& disp, Mat& dist){
	Mat disp_f;
	disp.convertTo(disp_f,CV_32F, 1/16., 0.0);
	cv::reprojectImageTo3D(disp_f, dist, Q, true, CV_32F);
}
//int main(){
//
//	Rectifier r(
//			"./left_camera.yaml",
//			"./right_camera.yaml"
//			);
//
//	return 0;
//}
