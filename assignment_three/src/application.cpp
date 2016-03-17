/*
* Matthew England
**/

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

Mat ransac_homography(std::vector<Point2f>& obj, std::vector<Point2f>& scene);


int main( int argc, char** argv )
{
  if( argc != 5 )
  {  
	std::cout << " Usage: " << argv[0] << "<img1> <img2> <use open_cv> <min hessian for sift>" << std::endl; 
	return -1; 
	}
	// read in the images
  Mat img_left = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE ); // this is the second image taken
  Mat img_right = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE ); // this is the first image taken

	int use_cv = atoi(argv[3]);
	int minHessian = atoi(argv[4]);

  if( !img_left.data || !img_right.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //Detect the keypoints using SIFT Detector

  SiftFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_left, keypoints_right;
	// run the detector to extraction keypoints from both images
  detector.detect( img_left, keypoints_left );
  detector.detect( img_right, keypoints_right );

  // Calculate descriptors (feature vectors)
  SiftDescriptorExtractor extractor;

  Mat descriptors_left, descriptors_right;
	// compute descriptors for both images
  extractor.compute( img_right, keypoints_right, descriptors_right );
  extractor.compute( img_left, keypoints_left, descriptors_left );

  //Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
	// Run the Approxiamte Nearest Neighbor Algorithm
  matcher.match( descriptors_right, descriptors_left, matches );

  double max_dist = 0; double min_dist = 100;

  // Find the minimum distance and max distance between key points
  for( int i = 0; i < descriptors_right.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //Display only "good" matches 
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_right.rows; i++ )
  { 
		if( matches[i].distance < 3*min_dist )
     { 
				good_matches.push_back( matches[i]); 
    }
  }

  Mat img_matches;
  drawMatches( img_right, keypoints_right, img_left, keypoints_left,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  // Show detected matches
  imshow( "Good Matches Detection", img_matches );
	imwrite("GoodMathcesDetection.png", img_matches);

  //-- Localize the object
  std::vector<Point2f> left;
  std::vector<Point2f> right;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //Get the keypoints from the good matches
    left.push_back( keypoints_right[ good_matches[i].queryIdx ].pt );
    right.push_back( keypoints_left[ good_matches[i].trainIdx ].pt );
  }
	Mat H;
	if (use_cv) {
		// use opencv builtin homography
  	H = findHomography( right, left, CV_RANSAC );
	}
	else {
		H = ransac_homography(left,right);
	}

  waitKey(0);


	cv::Mat result;
	int max_img_rows = img_left.rows;
	if (img_left.rows < img_right.rows  ) {
		max_img_rows = img_right.rows;
	}
	// wrap the image with the found homography
	warpPerspective(img_left,result,H,cv::Size(img_left.cols+img_right.cols,max_img_rows));
 	// copy the other image into result 
	cv::Mat half(result,cv::Rect(0,0,img_right.cols,img_right.rows));	
	img_right.copyTo(half);

  imshow( "Result", result);

  waitKey(0);
	imwrite("Result.png", result);

  return 0;
}


// NOT COMPLETE WILL WORK ON IT
// DID not submit weird code
Mat ransac_homography (std::vector<Point2f>& obj, std::vector<Point2f>& scene) {
	Mat H;
	if (obj.empty() || scene.empty())
		return H;
	return H;
}
