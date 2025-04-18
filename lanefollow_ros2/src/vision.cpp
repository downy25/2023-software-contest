#include "lanefollow_ros2/vision.hpp"
namespace dahun {
	void Image_Pretreatment(Mat& frame) {
		frame = frame(Rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4));
		cvtColor(frame, frame, COLOR_BGR2GRAY);
		frame = frame + (100 - mean(frame)[0]);
		threshold(frame, frame,145, 255, THRESH_BINARY);
	}
	Point2d find_object(Mat& labels, Mat& stats, Mat& centroids, Mat& frame, Point2d& prev_pt, Point2d crnt_pt) {
		int cnt = connectedComponentsWithStats(frame, labels, stats, centroids);
		cvtColor(frame, frame, COLOR_GRAY2BGR);

		vector <double> distance;
		double mindistance;
		int min_index;
		double pt_distance;
		
		circle(frame, prev_pt, 2, Scalar(255, 0, 0), 2);
		
		if (cnt > 1) {
			//if(center){prev_pt=Point2d(frame.cols/2,frame.rows/2); center=false;}
			for (int i = 1; i < cnt; i++) {
				double* cpt = centroids.ptr<double>(i);
				pt_distance = sqrt(pow(cpt[0]-prev_pt.x,2)+pow(cpt[1]-prev_pt.y,2));
				distance.push_back(pt_distance);
			}
			mindistance = *min_element(distance.begin(), distance.end());
			min_index = min_element(distance.begin(), distance.end()) - distance.begin();
			crnt_pt = Point2d(centroids.at<double>(min_index + 1, 0), centroids.at<double>(min_index + 1, 1));
			cout<<mindistance<<endl;
			if (mindistance > 100) crnt_pt = prev_pt;
			distance.clear();
		}
		else {
			crnt_pt = prev_pt;
		}
		prev_pt = crnt_pt;

		circle(frame, crnt_pt, 2, Scalar(0, 0, 255), 2);
		return crnt_pt;
	}
	int get_error(Point2d cpt,Point2d center) {
		int error=(center.x-cpt.x)*GAIN;
		return error;
	}
	void Cruve_Checker(int& cnt, int error, bool& Check){  // 곡선인지 커브인지
		if ((abs(error) < 12)) {  // 양쪽 바퀴값을 더해서 12 보다 작으면 직선으로 판단
			cnt++;
			if ((0 < cnt && cnt < 90)) Check = true;
			else { Check = false; }
		}
		else { Check = false; cnt = 0; }
	}
	void line_Catch(Point2d& prev_ptl, Point2d crnt_ptl, Point2d& prev_ptr, Point2d crnt_ptr, bool& Catch) {
		if ((prev_ptl != crnt_ptl) && (prev_ptr != crnt_ptr)) Catch = true;
		else Catch = false;
	}
}