#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cmath>
#include <vector>
#include <utility>
#include <cmath>
#include <cstdlib>

#include "hough_transform.hpp"

#define DEGREE2RADIAN 0.017453293 

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
	unsigned char* img_data = img.data;
	unsigned height = img.rows;
	unsigned width = img.cols;
	// number of bins
	this->_accu.width = 180;
	// the number of pixels to create
	double hough_height = ((sqrt(2.0) * (double)(height > width ? height : width )) / 2.0);
	this->_accu.height = hough_height * 2.0;

	this->_accu.accumulator = new unsigned[this->_accu.height * this->_accu.width];
	std::fill( this->_accu.accumulator, this->_accu.accumulator + (this->_accu.height * this->_accu.width), 0 );

	double center_row = height/2;
	double center_col = width/2;

	// loop through image and calculate the polar coordinates of the image
	for (unsigned r = 0; r < height; r++) {
		for (unsigned c = 0; c < width; c++) {
			// Is this a valid edge to use
			if (img_data[(r * width) + c] > 250 ) {
				for (unsigned t = 0; t < 180; t++) {
					double pc = ( ((double)c - center_col) * cos((double)t * DEGREE2RADIAN)) + (((double)r - center_row) * sin((double)t * DEGREE2RADIAN));
					// calculate polar coordinate and bucket into hough space
					const unsigned idx = ((round(pc + hough_height) * 180.0)) + t;
					this->_accu.accumulator[idx]++;
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
	for (unsigned int r = 0; r < this->_accu.height; r++) {
		for (unsigned int  c = 0; c < this->_accu.width; c++) {
			if (this->_accu.accumulator[(r*this->_accu.width) + c] >= threshold) {
				unsigned max = this->_accu.accumulator[(r*this->_accu.width) + c];
				// voting for local maximum
				for (int lr = -4; lr <= 4; lr++) {
					for (int lc = -4; lc <= 4; lc++) {
						// valid index 
						if (((int) (lr + r) >= 0 && (int) (lr+r) < this->_accu.height) && ((int) (lc+c) >=0 && (int) (lc+c) < this->_accu.width )) {
							// look for max value
							unsigned val = this->_accu.accumulator[((r + lr) *this->_accu.width) + (c + lc)]; 
							if ( val > max) {
								max = val;
								lr = lc = 5; // escape the local search window
							}	
						}	
					}
				}
				if (max > this->_accu.accumulator[(r * this->_accu.width) + c]) {
					continue;
				}				

				cv::Point p1;
				cv::Point p2;
				p1.x = p1.y = p2.x = p2.y = 0;
				// convert back into cartesian points
				if (c >= 45 && c <= 135) {
					// //y = (r - x cos(t)) / sin(t)
					p1.x = 0;
					p1.y = ((double)(r-(this->_accu.height/2.0)) - ((p1.x - (this->img_dims.width/2) ) * cos(c * DEGREE2RADIAN))) / sin(c * DEGREE2RADIAN) + (this->img_dims.height / 2.0);
					p2.x = this->img_dims.width - 0;
					p2.y = ((double)(r-(this->_accu.height/2.0)) - ((p2.x - (this->img_dims.width/2) ) * cos(c * DEGREE2RADIAN))) / sin(c * DEGREE2RADIAN) + (this->img_dims.height / 2.0);
				}
				else {	
					// //x = (r - y sin(t)) / cos(t)
					p1.y = 0;
					p1.x = ((double)(r-(this->_accu.height/2.0)) - ((p1.y - (this->img_dims.height/2) ) * sin(c * DEGREE2RADIAN))) / cos(c * DEGREE2RADIAN) + (this->img_dims.width / 2.0); 
					p2.y = this->img_dims.height - 0;
					p2.x = ((double)(r-(this->_accu.height/2.0)) - ((p2.y - (this->img_dims.height/2) ) * sin(c * DEGREE2RADIAN))) / cos(c * DEGREE2RADIAN) + (this->img_dims.width / 2.0);
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

