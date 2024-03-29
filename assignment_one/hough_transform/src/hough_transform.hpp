#ifndef HOUGH_TRANSFORM_H_
#define HOUGH_TRANSFORM_H_

#include <vector>
#include <cstdint>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
struct Accumulator {
	unsigned int* accumulator;
	unsigned int width;
	unsigned int height;
};

class hough_transform{

	public:
		hough_transform(cv::Size img_dims);
		~hough_transform();
		
		int transform(cv::Mat& img, uint8_t edge_threshold);
		std::vector<std::pair<cv::Point,cv::Point> > find_lines(size_t threshold, ssize_t local_maximum_radius);
		const Accumulator accumulator();
		
	protected:
		cv::Mat _img;
		Accumulator _accu;
		cv::Size img_dims;
		
};

#endif
