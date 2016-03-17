#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

void readme();
Mat ransac_homography(std::vector<Point2f>& obj, std::vector<Point2f>& scene);

/** @function main */
int main( int argc, char** argv )
{
  if( argc != 5 )
  { readme(); return -1; }

  Mat img_left = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_right = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );
	
	//Mat copy_left = img_left;
	//Mat copy_right = img_right;

	int use_cv = atoi(argv[3]);
	int minHessian = atoi(argv[4]);

  if( !img_left.data || !img_right.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SIFT Detector

  SiftFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_left, keypoints_right;

  detector.detect( img_left, keypoints_left );
  detector.detect( img_right, keypoints_right );

  //-- Step 2: Calculate descriptors (feature vectors)
  SiftDescriptorExtractor extractor;

  Mat descriptors_left, descriptors_right;

  extractor.compute( img_right, keypoints_right, descriptors_right );
  extractor.compute( img_left, keypoints_left, descriptors_left );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_right, descriptors_left, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_right.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_right.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( img_right, keypoints_right, img_left, keypoints_left,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches Detection", img_matches );
	imwrite("GoodMathcesDetection.png", img_matches);

  //-- Localize the object
  std::vector<Point2f> left;
  std::vector<Point2f> right;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
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

	//imshow( "H", H);
	//waitKey(0);

	warpPerspective(img_left,result,H,cv::Size(img_left.cols+img_right.cols,max_img_rows));
  
	cv::Mat half(result,cv::Rect(0,0,img_right.cols,img_right.rows));	
	img_right.copyTo(half);

	//imshow("half", half);
  imshow( "Result", result);

  waitKey(0);
	imwrite("Result.png", result);

  return 0;
  }

  /** @function readme */
 void readme()
 { std::cout << " Usage: ./image_stitcher <img1> <img2> <use open_cv> <min hessian for sift>" << std::endl; }

Mat ransac_homography (std::vector<Point2f>& obj, std::vector<Point2f>& scene) {
	Mat H;
	if (obj.empty() || scene.empty())
		return H;
	return H;
}
