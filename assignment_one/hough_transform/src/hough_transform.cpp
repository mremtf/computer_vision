#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cmath>
#include <vector>
#include <utility>

#include "hough_transform.hpp"

#define DEGREE2RADIAN 0.017f
// functions prototypes
/*		HoughTransform();
		~HoughTransform();
		
		int Transform(cv::Mat& img);
		std::vector<cv::Point> find_lines(size_t threshold);
		const Accumulator accumlator(size_t width, size_t height);
		
	private:
		cv::Mat img;
		Acculator accumlator;
	unsigned int* accumulator;
	unsigned int width;
	unsigned int height;
*/


hough_transform::hough_transform(cv::Size2i& img_dims) {
	this->_accu.accumulator = nullptr;
	this->_accu.width = 0;
	this->_accu.height = 0;
	this->img_dims = img_dims;
}

hough_transform::~hough_transform() {
	
	if (_accu.accumulator) {
		delete[] _accu.accumulator;
	}	
}


int hough_transform::transform(cv::Mat& img, uint8_t edge_threshold) {
	if (!img.data) {
		return -1;	
	}
	auto height = img.rows;
	auto width = img.cols;
	// number of bins
	this->_accu.width = 180;
	// the number of pixels to create
	double hough_height = ((sqrt(2.0) * (double)(height > width ? height : width )) / 2.0);
	this->_accu.height = hough_height * 2.0;

	this->_accu.accumulator = new unsigned int[this->_accu.height * this->_accu.width]();

	// loop through image and calculate the polar coordinates of the image
	for (auto r = 0; r < img.rows; ++r) {
		for (auto c = 0; c < img.cols; ++c) {
			// Is this a valid edge to use
			if (img.at<uchar>(r,c) > edge_threshold) {
				for (auto t = 0; t < 180; ++t) {
					//double r = ( ((double)x - center_x) * cos((double)t * DEG2RAD)) + (((double)y - center_y) * sin((double)t * DEG2RAD));
					// calculate polar coordinate and bucket into hough space
					double r = ( ((double)c) * cos((double)t * DEGREE2RADIAN)) + (((double)r) * sin((double)t * DEGREE2RADIAN));
					this->_accu.accumulator[(unsigned int) (std::round(r + this->_accu.height) * 180) + t]++;
				}	
			}
		}
	}
	
	return 0;	
}

std::vector<std::pair<cv::Point,cv::Point> > hough_transform::find_lines(size_t threshold, ssize_t local_maximum_radius) {
	std::vector<std::pair<cv::Point,cv::Point> > points;
	if (this->_accu.accumulator) {
		return points;
	}
	
	/* apply a threshold restriction on the accumulator values
		 convert from polar to cartesian
	*/
	for (auto r = 0; r < this->accu.height; ++r) {
		for (auto c = 0; c < this->accu.width; ++c) {
			if (this->_accu.accumulator[(r*this->_accu_width) + c] >= threshold) {
				auto max = this->_accu.accumulator[(r*this->_accu_width) + c];
				// voting for local maximum
				for (auto lr = -local_maximum_radius; lr <= local_maximum_radius; ++lr) {
					for (auto lc = -local_maximum_radius; lc <= local_maximum_radius; ++lc) {
						// valid index 
						if (lr + r >= 0 && lc+c >=0 && lr+r < this->_accu.height && lc+c < this->_accu.width) {
							// look for max value
							auto val = this->_accu.accumulator[((r + lr) *this->_accu.width) + (c + lc)] 
							if ( val > max) {
								max = val;
								lr = lc = 5; // escape the local search window
							}	
						}	
					}
				}
				
				if (max > this->_accu.accumulator[(r*this->_accu_width) + c]) {
					continue; // we found a new maximum, move to the next bucket	
				}

				cv::Point p1, p2;
				// convert back into cartesian points
				if (c >= 45 && c <= 135) {
					p1.x = 0;
					p1.y = ((double)(r-(this->_accu.height/2)) - ((p1.x - (this->img_dims.width/2) ) * cos(c * DEGREE2RADIAN))) / sin(c * DEGREE2RADIAN) + (this->img_dims.height / 2);
					p2.x = this->img_dims.width - 0;
					p2.y = ((double)(r-(this->_accu.height/2)) - ((p2.x - (this->img_dims.width/2) ) * cos(c * DEGREE2RADIAN))) / sin(c * DEGREE2RADIAN) + (this->img_dims.height / 2);
				}

				else {	
					p1.y = 0;
					p1.x = ((double)(r-(this->_accu.height/2)) - ((p1.y - (_this->img_dims.height/2) ) * sin(c * DEGREE2RADIAN))) / cos(c * DEG2RAD) + (this->img_dims.width / 2); 
					p2.y = this->img_dims.width - 0;
					p2.x = ((double)(r-(this->_accu.height/2)) - ((p2.y - (_this->img_dims.height/2) ) * sin(c * DEGREE2RADIAN))) / cos(c * DEG2RAD) + (this->img_dims.width / 2);
				}
				lines.push_back(std::make_pair(p1,p2));
			}			
		}
	}


	return lines;
}

const Accumulator hough_transform::accumlator(size_t width, size_t height) {
	return this->_accu;
}

