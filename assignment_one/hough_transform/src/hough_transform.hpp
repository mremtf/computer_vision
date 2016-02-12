#ifndef HOUGH_TRANSFORM_H_
#define HOUGH_TRANSFORM_H_

#include <vector>
#include <cstdint>

struct Accumulator {
	unsigned int* accumulator;
	unsigned int width;
	unsigned int height;
};

class hough_transform{

	public:
		hough_transform();
		~hough_transform();
		
		int transform(cv::Mat& img, uint8_t edge_threshold);
		std::vector<std::pair<cv::Point,cv::Point> > find_lines(size_t threshold);
		const Accumulator accumlator(size_t width, size_t height);
		
	private:
		cv::Mat _img;
		Accumulator _accu;
		
};

#endif
