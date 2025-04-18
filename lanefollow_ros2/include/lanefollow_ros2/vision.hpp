#ifndef _VISION_HPP_
#define _VISION_HPP_

#include "opencv2/opencv.hpp"
#define GAIN 0.3
#define INIT_SPEED 100
using namespace std;
using namespace cv;
namespace dahun {
	void Image_Pretreatment(Mat& frame);
	Point2d find_object(Mat& labels, Mat& stats, Mat& centroids, Mat& frame,Point2d& prev_pt, Point2d crnt_pt);
	int get_error(Point2d cpt,Point2d center);
	void Cruve_Checker(int& cnt, int error, bool& Check);
	void line_Catch(Point2d& prev_ptl, Point2d crnt_ptl, Point2d& prev_ptr, Point2d crnt_ptr, bool& Catch);
}
#endif