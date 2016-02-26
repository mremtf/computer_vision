#ifndef OPTICAL_FLOW_
#define OPTICAL_FLOW_
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>

class OpticalFlow {
	public:
		OpticalFlow(cv::Size window_size, double min_eigen_threshold);
		~OpticalFlow();	
	protected: 
		bool compute_flow_difference(cv::Mat& input, cv::Mat& output);
		bool generate_flow_image(cv::Mat& vector_img, cv::Mat& image);
	private:
	cv::Size _window_Size;
	double min_eigen_threshold;
};

#endif
