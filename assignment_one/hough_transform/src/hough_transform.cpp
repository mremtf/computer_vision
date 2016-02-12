#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cmath>
#include <vector>
#include <utility>

#include "hough_transform.hpp"

#define DEGREE2RADIAN 0.017f

hough_transform::hough_transform(cv::Size img_dims) {
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

	double center_row = height/2;
	double center_col = width/2;

	// loop through image and calculate the polar coordinates of the image
	for (auto r = 0; r < img.rows; ++r) {
		for (auto c = 0; c < img.cols; ++c) {
			// Is this a valid edge to use
			if (img.at<uchar>(r,c) > edge_threshold) {
				for (auto t = 0; t < 180; ++t) {
					double pc = ( ((double)c - center_col) * cos((double)t * DEGREE2RADIAN)) + (((double)r - center_row) * sin((double)t * DEGREE2RADIAN));
					// calculate polar coordinate and bucket into hough space
					//double r = ( ((double)c) * cos((double)t * DEGREE2RADIAN)) + (((double)r) * sin((double)t * DEGREE2RADIAN));
					this->_accu.accumulator[(int) ((std::round(pc + hough_height) * 180.0)) + t]++;
				}	
			}
		}
	}
	
	return 0;	
}

std::vector<std::pair<cv::Point,cv::Point> > hough_transform::find_lines(size_t threshold, ssize_t local_maximum_radius) {
	std::vector<std::pair<cv::Point,cv::Point> > lines;
	if (this->_accu.accumulator == nullptr) {
		return lines;
	}
	
	/* apply a threshold restriction on the accumulator values
		 convert from polar to cartesian
	*/
	for (unsigned int r = 0; r < this->_accu.height; ++r) {
		for (unsigned int  c = 0; c < this->_accu.width; ++c) {
			if (this->_accu.accumulator[(r*this->_accu.width) + c] >= threshold) {
				auto max = this->_accu.accumulator[(r*this->_accu.width) + c];
				bool local_max_found = false;
				// voting for local maximum
				for (auto lr = -local_maximum_radius; lr <= local_maximum_radius; ++lr) {
					for (auto lc = -local_maximum_radius; lc <= local_maximum_radius; ++lc) {
						// valid index 
						if (lr + r >= 0 && lc+c >=0 && lr+r < this->_accu.height && lc+c < this->_accu.width) {
							// look for max value
							auto val = this->_accu.accumulator[((r + lr) *this->_accu.width) + (c + lc)]; 
							if ( val > max) {
								max = val;
								lr = lc = 5; // escape the local search window
								local_max_found = true;
							}	
							else if (val == max) {
								if (lc == 0 && lr == 0) {
									continue;
								}
							}
						}	
					}
				}
				if (local_max_found) {
					continue;
				}				

				cv::Point p1, p2;
				// convert back into cartesian points
				if (c >= 45 && c <= 135) {
					// //y = (r - x cos(t)) / sin(t)
					p1.x = 0;
					p1.y = ((double)(r-(this->_accu.height/2)) - ((p1.x - (this->img_dims.width/2) ) * cos(c * DEGREE2RADIAN))) / sin(c * DEGREE2RADIAN) + (this->img_dims.height / 2);
					p2.x = this->img_dims.width - 0;
					p2.y = ((double)(r-(this->_accu.height/2)) - ((p2.x - (this->img_dims.width/2) ) * cos(c * DEGREE2RADIAN))) / sin(c * DEGREE2RADIAN) + (this->img_dims.height / 2);
				}

				else {	
					// //x = (r - y sin(t)) / cos(t)
					p1.y = 0;
					p1.x = ((double)(r-(this->_accu.height/2)) - ((p1.y - (this->img_dims.height/2) ) * sin(c * DEGREE2RADIAN))) / cos(c * DEGREE2RADIAN) + (this->img_dims.width / 2); 
					p2.y = this->img_dims.height - 0;
					p2.x = ((double)(r-(this->_accu.height/2)) - ((p2.y - (this->img_dims.height/2) ) * sin(c * DEGREE2RADIAN))) / cos(c * DEGREE2RADIAN) + (this->img_dims.width / 2);
				}
				lines.push_back(std::make_pair(p1,p2));
			}			
		}
	}


	return lines;
}

const Accumulator hough_transform::accumulator() {
	return this->_accu;
}

