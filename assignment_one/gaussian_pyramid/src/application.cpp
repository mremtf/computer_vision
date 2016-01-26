#include <iostream>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <string>
#include "../include/gaussian_pyramid.hpp"

#define IMG_IDX 1
#define NUM_LVL_IDX 2
#define KERNEL_SIZE 3
#define KERNEL_SIGMA 4

using itr =  std::vector<cv::Mat>::iterator;


int main (int argc, char** argv) {
	if (argc <= 4) {
		std::cout << argv[0] << " <input image> <num levels> <kernel size> <sigma>"
		<< std::endl;
		return 0;
	}
	
	// load image
	cv::Mat image;
  image = cv::imread(argv[IMG_IDX], CV_LOAD_IMAGE_COLOR);
		
	// Check for invalid input
	if(! image.data )                              
  {
  	std::cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }
	// gaussian kernel paramters
	kernel_parameters params;
	// convert command line parameters	
	char* end = nullptr;
	params.ksize = strtol(argv[KERNEL_SIZE], &end, 10);
  params.sigma = strtof(argv[KERNEL_SIGMA],&end );	
	params.ktype = image.depth();
	size_t levels = strtol(argv[NUM_LVL_IDX], &end, 10);

	// Gaussian Object Creation
	gaussian_pyramid gp(params);

	// build the pyramid
	std::vector<cv::Mat> sampled_imgs; 
	if ( ! gp.build_pyramid(image,levels,sampled_imgs)) {
		std::cout << "Failed to build Pyramid" << std::endl;
		return -1;
	}
	std::stringstream ss;	
	std::string str;
	itr current = sampled_imgs.begin();	
	itr v_end = sampled_imgs.end();
	for (size_t i = 0; current != v_end; ++current, ++i) {
		ss << "pyramd_level_" << i << ".jpg";
		str = ss.str();
		imwrite( str.c_str(), *current );
		ss.str(std::string());	
  }
	
	return 0;	
}
