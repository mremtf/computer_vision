#ifndef _GAUSSIAN_PYRAMID_HPP_
#define _GAUSSIAN_PYRAMID_HPP_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

struct kernel_parameters {
	int ksize; // the radius of the kernel
	double sigma; // variance of the kernel
	int ktype; // the data type
};

class gaussian_pyramid {
	
	public:
	gaussian_pyramid(kernel_parameters& parameters);
	~gaussian_pyramid();
		
	bool build_pyramid(cv::Mat& image, size_t levels,std::vector<cv::Mat>& outImages); 
	protected:

	private:
	kernel_parameters parameters;

};
	
#endif
