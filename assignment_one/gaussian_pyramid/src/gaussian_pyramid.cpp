#include "../include/gaussian_pyramid.hpp"

#include <opencv2/imgproc/imgproc.hpp>

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

  // repeat of the process of smoothing and downsampling
	cv::Mat src = image; 
	for (auto l = 0; l < levels; ++l) {
  	// used for temporary storage
		cv::Mat dst;
  	// create the gaussian kernel
		cv::Mat gaussian_kernel = cv::getGaussianKernel(this->parameters.ksize, this->parameters.sigma);	
		// applying filter over the current image.
		cv::filter2D(src, dst, -1 , gaussian_kernel, cv::Point( -1, -1 ), 0, cv::BORDER_DEFAULT );
		// store the gaussian smoothed image
		outImages.push_back(dst);
		// update the current src
		int src_width = dst.size().width;
		int src_height = dst.size().height;
		// shift down by powers of 2
		cv::Size2i dsize(src_width >> 1,src_height >> 1);
		// down sample the image using resize
		std::cout << "downsampled size" << dsize << std::endl;
		resize(dst, src, dsize, cv::INTER_NEAREST); 
	}
	return true;
}
