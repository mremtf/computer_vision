#include "../include/gaussian_pyramid.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
// hidden function
cv::Mat getGaussianKernel(size_t k_size, double sigma);

gaussian_pyramid::gaussian_pyramid(kernel_parameters& parameters) {
	this->parameters = parameters; 
}

gaussian_pyramid::~gaussian_pyramid() {
	;
}

bool gaussian_pyramid::build_pyramid(cv::Mat& image, size_t levels,std::vector<cv::Mat>& outImages) {
	// validate parameters
	if (!image.data) {
		return false;
	}
	cv::Mat gaussian_kernel = getGaussianKernel(this->parameters.ksize, this->parameters.sigma);
	// repeat of the process of smoothing and downsampling
	cv::Mat curr = image;
	// push back current image	
	outImages.push_back(curr); 
	for (auto l = 1; l < levels; ++l) {
		// used for temporary storage
		cv::Mat dst;
		// applying filter over the current image.	
		cv::filter2D(curr, dst, -1 , gaussian_kernel, cv::Point( -1, -1 ), 0, cv::BORDER_DEFAULT );	
		// update the current src
		int new_width = dst.size().width;
		int new_height = dst.size().height;
		// shift down by powers of 2	
		if ((new_width >> 1) < 1 || (new_height >> 1) < 1) {
			std::cout << "GENERATED UPTO " << l << " levels" << std::endl;
			std::cout << "Cannot go less than 1x1" << std::endl;
			break;
		}
		cv::Size2i dsize(new_width >> 1,new_height >> 1);
		// down sample the image using resize
		resize(dst, curr, dsize, cv::INTER_NEAREST); 
		// add the new blurred image to the vector
		outImages.push_back(curr);
	}
	return true;
}

cv::Mat getGaussianKernel(size_t k_size, double sigma){
	cv::Mat gaussian_kernel(k_size,k_size,CV_64F);
	if (k_size == 3 && sigma == 1.0) {
		gaussian_kernel.at<double>(0,0) = 1.0/16.0;
		gaussian_kernel.at<double>(0,1) = 2.0/16.0;
		gaussian_kernel.at<double>(0,2) = 1.0/16.0;
		gaussian_kernel.at<double>(1,0) = 2.0/16.0;
		gaussian_kernel.at<double>(1,1) = 4.0/16.0;
		gaussian_kernel.at<double>(1,2) = 2.0/16.0;
		gaussian_kernel.at<double>(2,0) = 1.0/16.0;
		gaussian_kernel.at<double>(2,1) = 2.0/16.0;
		gaussian_kernel.at<double>(2,2) = 1.0/16.0;
	}
	else if (k_size == 5 && sigma == 1.0){
		// row 1
		gaussian_kernel.at<double>(0,0) = 1.0;
		gaussian_kernel.at<double>(0,1) = 4.0;
		gaussian_kernel.at<double>(0,2) = 6.0;
		gaussian_kernel.at<double>(0,3) = 4.0;
		gaussian_kernel.at<double>(0,4) = 1.0;
		// row 2
		gaussian_kernel.at<double>(1,0) = 4.0;
		gaussian_kernel.at<double>(1,1) = 16.0;
		gaussian_kernel.at<double>(1,2) = 24.0;
		gaussian_kernel.at<double>(1,3) = 16.0;
		gaussian_kernel.at<double>(1,4) = 4.0;
		// row 3
		gaussian_kernel.at<double>(2,0) = 6.0;
		gaussian_kernel.at<double>(2,1) = 24.0;
		gaussian_kernel.at<double>(2,2) = 36.0;
		gaussian_kernel.at<double>(2,3) = 24.0;
		gaussian_kernel.at<double>(2,4) = 6.0;
		// row 4
		gaussian_kernel.at<double>(3,0) = 4.0;
		gaussian_kernel.at<double>(3,1) = 16.0;
		gaussian_kernel.at<double>(3,2) = 24.0;
		gaussian_kernel.at<double>(3,3) = 16.0;
		gaussian_kernel.at<double>(3,4) = 4.0;
		// row 5
		gaussian_kernel.at<double>(4,0) = 1.0;
		gaussian_kernel.at<double>(4,1) = 4.0;
		gaussian_kernel.at<double>(4,2) = 6.0;
		gaussian_kernel.at<double>(4,3) = 4.0;
		gaussian_kernel.at<double>(4,4) = 1.0;	
		gaussian_kernel = gaussian_kernel*1.0/256;	
	}
	else {
		for (auto r = 0; r < gaussian_kernel.rows; ++r) {
			for (auto c = 0; c < gaussian_kernel.cols; ++c) {
				const double val = (1.0/(2.0*M_PI*(sigma*sigma)))*exp(-(((r*r) - (c*c))/(2*(sigma*sigma))));
				gaussian_kernel.at<double>(r,c) = val;	
			}
		}	
	}
	//std::cout << gaussian_kernel << std::endl;
	return gaussian_kernel;
}
