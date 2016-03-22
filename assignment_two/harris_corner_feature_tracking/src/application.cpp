/*
* Matthew England
*/
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;


// prototypes
cv::Mat corner_harris(cv::Mat& img, double threshold, std::vector<cv::Point>& corner_points);
void drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, 
								 cv::Scalar color= cv::Scalar(0,0,255), int radius=1, 
								 int thickness=0);

/*
* Calculate the derivate of the x direction of the imagei
* kernel -1 1 applied to both
*				 -1 1

*/
Mat get_fx(Mat &src1, Mat &src2){
    Mat fx;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel.at<double>(0, 0) = -1.0;
    kernel.at<double>(1, 0) = -1.0;

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel*.25);
    filter2D(src2, dst2, -1, kernel*.25);

    fx = dst1 + dst2;
    return fx;
}

/*
* Calculate the derivate of the y direction of the image
* 
* kernel -1 -1 applied to both
*				  1  1
*/
Mat get_fy(Mat &src1, Mat &src2){
    Mat fy;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel.at<double>(0, 0) = -1.0;
    kernel.at<double>(0, 1) = -1.0;

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel*.25);
    filter2D(src2, dst2, -1, kernel*.25);

    fy = dst1 + dst2;
    return fy;
}

/*
* Calculate the derivate of the time of the image
*
* kernel 1 -1 -1 applied to src1
					 -1 -1
*
* kernel 2  1  1 apllied to src 2
*					 	1  1 
*/
Mat get_ft(Mat &src1, Mat &src2){
    Mat ft;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel = kernel.mul(-1);

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel*.25);
    kernel = kernel.mul(-1);
    filter2D(src2, dst2, -1, kernel*.25);

    ft = dst1 + dst2;
    return ft;
}

/*
* Used for summation of the partial derivatives
*/
Mat get_window_sum (Mat &m, int window_radius) {
	size_t total_window_size = (window_radius + window_radius + 1) * (window_radius + window_radius + 1);
	cv::Mat res = cv::Mat::zeros(m.rows,m.cols,CV_64FC1);
	for (int r = window_radius; r < m.rows - window_radius; r++) {
		for (int  c = window_radius; c < m.cols - window_radius; c++) {
			// traversing local window to create a sum
			size_t cnt = 0;
			double val = 0.0;
			//std::cout << cnt << std::endl;
			for (int lr = -window_radius; lr <= window_radius; lr++) {
				for (int lc = -window_radius; lc <= window_radius; lc++) {
					// valid index 
					if (((lr + r) >= 0 && (lr+r) < m.rows) && ((lc+c) >=0 && (lc+c) < m.cols )) {
						// summing the window
						++cnt;
						val += m.at<double>(lr + r,lc + c);
					}
				}
			}
			if (cnt == total_window_size)	
				res.at<double>(r,c) = val; 
			else {
				res.at<double>(r,c) *= total_window_size;
			}
		}
	}
	return res;
}

/*
* Calculates the LK optical flow on a image
* takes the input images and the desired window radius
* returns the u and v generated from the image 
*/
void calc_lucas_kanade_optical_flow(Mat &img1, Mat &img2, int window_radius, Mat &u, Mat &v){

		// Calculate the first derivatives of the image for x,y, and time
    Mat fx = get_fx(img1, img2);
    Mat fy = get_fy(img1, img2);
    Mat ft = get_ft(img1, img2);

		// Calculate the partial derivates
    Mat fx2 = fx.mul(fx); //Ix^2
    Mat fy2 = fy.mul(fy); //Iy^2
    Mat fxfy = fx.mul(fy); // Ixy
    Mat fxft = fx.mul(ft); // Ixt
    Mat fyft = fy.mul(ft); // Iyt



    // Get the sum of the windows
		Mat sum_fx2 = get_window_sum(fx2,window_radius); // sum Ix^2
    Mat sum_fy2 = get_window_sum(fy2,window_radius); // sum Iy^2
    Mat sum_fxft = get_window_sum(fxft,window_radius); // sum Ixy
    Mat sum_fxfy = get_window_sum(fxfy,window_radius); // sum Ixt
    Mat sum_fyft = get_window_sum(fyft,window_radius); // sum Iyt

		// Does the least squares method	
    Mat tmp = sum_fx2.mul(sum_fy2) - sum_fxfy.mul(sum_fxfy);
   	// Calculate the U and V
		// solve for U first 
		u = sum_fxfy.mul(sum_fyft) - sum_fy2.mul(sum_fxft);
		// solve for V second
    v = sum_fxft.mul(sum_fxfy) - sum_fx2.mul(sum_fyft);
		// normalize the U and V matrix
    divide(u, tmp, u);
    divide(v, tmp, v);
}

// Create the gaussian pyramid of an image
//
//
vector<Mat> get_gaussian_pyramid(Mat &img, size_t pyramid_levels){
    vector<Mat> pyramid;
    pyramid.push_back(img);
    Mat down_sample;
    for(size_t i = 0; i < pyramid_levels - 1; i++){
        pyrDown(pyramid[pyramid.size() - 1], down_sample);
        pyramid.push_back(down_sample);
    }
    return pyramid;
}


bool multi_view_scaling (Mat &img1, Mat &img2, Mat &u, Mat &v, std::vector<Point>& tracked,int window_radius, size_t pyramid_levels, double threshold) {
    
		if (!img1.data || !img2.data) {
			return false;
    }
    // create the gaussian_pyramid
    vector<Mat> pyr1 = get_gaussian_pyramid(img1, pyramid_levels);
    vector<Mat> pyr2 = get_gaussian_pyramid(img2, pyramid_levels);
		
		

    Mat upsampled_u, upsampled_v;

    // start from the top of pyramid and apply the optical flow solution
    // and more down the pyramid



    for(size_t i = pyramid_levels - 1; i >= 0; i--) {
				// create temporary u and v for optical flow calculation
				// call harris corner detector
				std::vector<cv::Point> corner_points;
				Mat current_tracked = corner_harris(pyr1[i], threshold,corner_points);  
					
        Mat current_u = Mat::zeros(pyr1[i].rows, pyr1[i].cols, CV_64FC1);
        Mat current_v = Mat::zeros(pyr2[i].rows, pyr2[i].cols, CV_64FC1);
        calc_lucas_kanade_optical_flow(pyr1[i], pyr2[i], window_radius,current_u, current_v);

				// update the previous current vector motion with the previous 
				// vector motion
        if(i != pyramid_levels - 1){
            current_u += upsampled_u;
            current_v += upsampled_v;
        }
				// Bottom level of the pyramid
				// we are down feature tracking
        if(i == 0){
            u = current_u;
            v = current_v;
						tracked = corner_points;
            return true;
        }
	
				// apply the a gaussian bluring and upsample
        pyrUp(current_u, upsampled_u);
        pyrUp(current_v, upsampled_v);

        Mat map1(upsampled_u.size(), CV_32FC2);
        Mat map2(upsampled_u.size(), CV_32FC2);
				//Mat map3(upsampled_u.size(), CV_32FC2);	
				// generate the mapping points from current pyramid level into the next pyramid level
        for (auto y = 0; y < map1.rows; ++y){
            for (auto x = 0; x < map1.cols; ++x){
                Point2f f = Point2f((float)(upsampled_u.at<double>(y, x)), (float)(upsampled_v.at<double>(y, x)));
								map1.at<Point2f>(y, x) = Point2f(x + f.x / 2, y + f.y / 2);
                map2.at<Point2f>(y, x) = Point2f(x - f.x / 2, y - f.y / 2);	
                //map3.at<Point2f>(y, x) = Point2f(x - f.x / 2, y - f.y / 2);
            }
        }

				// update the current layer of image with warping
        Mat warped_pyr1, warped_pyr2;
				Mat wraped_harris;
        remap(pyr1[i - 1], warped_pyr1, map1, cv::Mat(), INTER_LINEAR);
        remap(pyr2[i - 1], warped_pyr2, map2, cv::Mat(), INTER_LINEAR);
				// remap for harris points
				//remap(pyr1[i - 1], wraped_harris, map3,cv::Mat(), INTER_LINEAR);
				// update the pyramid image
        warped_pyr1.copyTo(pyr1[i - 1]);
        warped_pyr2.copyTo(pyr2[i - 1]);
				//ss << "pyramid_level_" << i << ".jpg";
				//cv::imwrite(ss.str().c_str(), pyr1[i - 1]);
				//ss.str("");
    }

    return true;
}

/*
* Harris Corner Detection Functions
*/

cv::Mat corner_harris(cv::Mat& img, double threshold, std::vector<cv::Point>& corner_points) {

	double k = 0.04;

	cv::Mat blurred, Ix,Iy;
	int ddepth = CV_64F;
	// apply a guassian blur
	cv::GaussianBlur(img,blurred, cv::Size(5,5),1);
	// apply sobel derivative in y and x
	cv::Sobel(blurred, Ix, ddepth, 1, 0, 3, 1, 0, BORDER_DEFAULT );	
	cv::Sobel(blurred, Iy, ddepth, 0, 1, 3, 1, 0, BORDER_DEFAULT );	
	
	// compute products of derivates at every pixel
	cv::Mat Ix2 = Ix.mul(Ix);
	cv::Mat Iy2 = Iy.mul(Iy);
	cv::Mat Ixy = Ix.mul(Iy);


	// Partition the image into tiles
	const size_t img_rows = img.rows;
	const size_t img_cols = img.cols;
	Mat Gxy = cv::Mat::ones(img_rows, img_cols, CV_64FC1);
	Gxy.at<double>(0,0) = 1.0;
	Gxy.at<double>(0,1) = 4.0;
	Gxy.at<double>(0,2) = 6.0;
	Gxy.at<double>(0,3) = 4.0;
	Gxy.at<double>(0,4) = 1.0;
	// row 2
	Gxy.at<double>(1,0) = 4.0;
	Gxy.at<double>(1,1) = 16.0;
	Gxy.at<double>(1,2) = 24.0;
	Gxy.at<double>(1,3) = 16.0;
	Gxy.at<double>(1,4) = 4.0;
	// row 3
	Gxy.at<double>(2,0) = 6.0;
	Gxy.at<double>(2,1) = 24.0;
	Gxy.at<double>(2,2) = 36.0;
	Gxy.at<double>(2,3) = 24.0;
	Gxy.at<double>(2,4) = 6.0;
	// row 4
	Gxy.at<double>(3,0) = 4.0;
	Gxy.at<double>(3,1) = 16.0;
	Gxy.at<double>(3,2) = 24.0;
	Gxy.at<double>(3,3) = 16.0;
	Gxy.at<double>(3,4) = 4.0;
	// row 5
	Gxy.at<double>(4,0) = 1.0;
	Gxy.at<double>(4,1) = 4.0;
	Gxy.at<double>(4,2) = 6.0;
	Gxy.at<double>(4,3) = 4.0;
	Gxy.at<double>(4,4) = 1.0;	
	Gxy = Gxy*1.0/256.0;	
	
	cv::Mat Rvals = cv::Mat::zeros(img_rows, img_cols, CV_64FC1);
	cv::Mat corners = cv::Mat::zeros(img_rows, img_cols, CV_64FC1);
	cv::Mat H = cv::Mat::zeros(2, 2, CV_64FC1);

	cv::Mat Sx2, Sy2, Sxy;
	//sum the derivates
	filter2D(Ix2, Sx2, -1, Gxy);
	filter2D(Iy2, Sy2, -1, Gxy);
	filter2D(Ixy, Sxy, -1, Gxy);

	double R = 0;
	double t = 0;
	double d = 0;

	for (size_t r= 0; r < img_rows; ++r) {
		for (size_t c = 0; c < img_cols; ++c) {
			// populate the H matrix
      H.at<double>(0, 0) = Sx2.at<double>(r,c);
      H.at<double>(0, 1) = Sxy.at<double>(r,c);
      H.at<double>(1, 0) = Sxy.at<double>(r,c);
			H.at<double>(1, 1) = Sy2.at<double>(r,c);
			
       // Compute the response of the detector at each pixel
       t = H.at<double>(0,0) + H.at<double>(1,1);
			 d = H.at<double>(0,0) * H.at<double>(1,1);			

			 R = d - k * (t * t);
			 std::cout << R << std::endl;
			 //Threshold on value of R
			 if (R >= threshold) {
					Rvals.at<double>(r,c) = R;
			 }
		}
	}
 	//Mat features = Mat::zeros(img.rows, img.cols, CV_64FC1);

	for (size_t r = 0; r < img_rows; ++r) {
		for (size_t c = 0; c < img_cols; ++c) {
				// add points so easily to plot latter
				if (Rvals.at<double>(r,c)) {
					corner_points.push_back(cv::Point(r,c));
				}
		}
	}
	return Rvals;	
}

// Draw circles at feature point locations on an image
void drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, cv::Scalar color, int radius, int thickness) {

	std::vector<cv::Point>::const_iterator it= points.begin();

	  // for all corners
	  while (it!=points.end()) {

	  	// draw a circle at each corner location
	  	cv::circle(image,*it,radius,color,thickness);
	  	++it;
	  }
}


int main(int argc, char** argv){
	if (argc < 6) {
		std::cout << argv[0] << " <input image one > <input image two> <window radius> <pyramid_levels> <threshold> <test checker image>" << std::endl;
		return -1;
	}
	Mat img1;
	Mat img2;
	Mat in = imread(argv[1]);
	Mat out = in.clone();
	Mat out2 = in.clone();
	  /// Convert it to gray to 1 channel image
  cv::cvtColor( in, img1, cv::COLOR_RGB2GRAY );
	
	if (!img1.data) {
	std::cout << "NO IMAGE 1 TO READ" << std::endl;
	return -1;
	}
  Mat in2 = imread(argv[2]);
	if (!in2.data) {
	std::cout << "NO IMAGE 2 TO READ" << std::endl;
	return -1;
	}
	 /// Convert it to gray to 1 channel image
  cv::cvtColor( in2, img2, cv::COLOR_RGB2GRAY );


	int window_radius = atoi(argv[3]);
	int pyramid_levels = atoi(argv[4]);
	float threshold = atof(argv[5]);

	cv::Mat img3;
	// test harris corner alg
	if (argc == 7) {
		std::cout << "HERE" << std::endl;
  	Mat in3 = imread(argv[6]);
		if (!in3.data) {
		std::cout << "NO IMAGE 3 TO READ" << std::endl;
		return -1;
		}
		cv::Mat copy = in3.clone();
		/// Convert it to gray to 1 channel image
 		cv::cvtColor( in3, img3, cv::COLOR_RGB2GRAY );
  	img3.convertTo(img3, CV_64FC1, 1.0/255, 0);
		// call harris corner detector
		std::vector<cv::Point> corner_points;
		Mat features = corner_harris(img3, threshold,corner_points);  
		drawOnImage(copy,corner_points);
		cv::namedWindow("CHECKERBOARD", cv::WINDOW_AUTOSIZE);
		imshow("CHECKERBOARD",copy);
		waitKey(0);

	}

	

	// convert to 64 bit gray scale
  img1.convertTo(img1, CV_64FC1, 1.0/255, 0);
  img2.convertTo(img2, CV_64FC1, 1.0/255, 0);


  Mat u = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
  Mat v = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
	std::vector<cv::Point> corner_points;
	std::vector<cv::Point> corner_points2;

	Mat features1 = corner_harris(img2, threshold,corner_points);  
	cv::namedWindow("Start", cv::WINDOW_AUTOSIZE);	
	drawOnImage(out,corner_points);
	imshow("Start",out);
	waitKey(0);
	
	std::vector<Point> tracked;
	multi_view_scaling (img1, img2, u, v, tracked, window_radius, pyramid_levels, threshold);

	
	drawOnImage(out2,tracked);

	cv::namedWindow("POINTS", cv::WINDOW_AUTOSIZE);
	imshow("POINTS",out2);
	waitKey(0);



  return 0;
}
