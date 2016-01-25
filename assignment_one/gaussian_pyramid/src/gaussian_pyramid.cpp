#include "../include/gaussian_pyramid.hpp"

#include <opencv2/imgproc/imgproc.hpp>

gaussian_pyramid::gaussian_pyramid(KernelParameters& parameters) {
	this->parameters = paramets; 
}

gaussian_pyramid::~gaussian_pyramid() {
	;
}


bool gaussian_pyramid::build_pyramid(cv::Mat& image, size_t levels,std::vector<cv::Mat>& outImage) {
	// validate parameters
	if (!image.data) {
		return false;
	}

  // repeat of the process of smoothing and downsampling
	cv::Mat src = image; 
	for (auto l = 0; l < levels; ++l) {
  	// used for temporary storage
		cv::mat dst;
  	// create the gaussian kernel
		cv::Mat gaussian_kernel = cv::etGaussianKernel(this->parameters.ksize, this->parameters.sigma, this->parameters.ktype);	
		// applying filter over the current image.
		cv::filter2D(src, dst, -1 , gaussian_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT );
		// store the gaussian smoothed image
		outImages.push_back(dst);
		// update the current src
		src = dst;	
		cv::size2i dsize = src.size();
		// down sample the image using resize
		dsize.width =>> 1; // shift down by 2
		dsize.height =>> 1; // shift down by 2
		resize(src, dst, dsize) 
	}
	return true;
}
