#include "irl_stereo/f_sgbm.h"

FilteredSGBM::FilteredSGBM(){

	//int wsize = 3;
	//int max_disp = 128;
	//float lambda = 8000.0;
	//float sigma = 1.5;
	int wsize = 11;
	int max_disp = 128;
	float lambda = 8000.0;
	float sigma = 1.5;
	int P1 = 200;
	int P2 = 400;
	int disp12MaxDiff = 0;
	int prefilterCap = 9;
	int uniquenessRatio = 15;
	int speckleWindowSize = 100;
	int speckleRange = 4;

	left_matcher = StereoSGBM::create(0,max_disp,wsize,P1,P2,disp12MaxDiff,prefilterCap,uniquenessRatio,speckleWindowSize,speckleRange,StereoSGBM::MODE_SGBM_3WAY);
	wls_filter = createDisparityWLSFilter(left_matcher);
	right_matcher = createRightMatcher(left_matcher);

	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);

}
void FilteredSGBM::compute(const Mat& left, const Mat& right, Mat& filtered_disp, Mat* raw_disp){

	double matching_time = (double)getTickCount();
	left_matcher->compute(left, right,left_disp);
	right_matcher->compute(right,left, right_disp);
	matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();


	double filtering_time = (double)getTickCount();
	wls_filter->filter(left_disp,left,filtered_disp,right_disp);
	filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();

	//printf("Match : %.2f, Filter : %.2f\n", matching_time, filtering_time);
	if(raw_disp != nullptr){
		left_disp.copyTo(*raw_disp);
	}
	//left_disp.copyTo(filtered_disp);
	//Mat conf_map = Mat(left.rows,left.cols,CV_8U);
	//conf_map = wls_filter->getConfidenceMap();

	// Get the ROI that was used in the last filter call:
	//Rect ROI = wls_filter->getROI();
	return;
}
