# Matthew England

# The application is C++ 11, OpenCV, and CMake

# location of executable
Stored in bin and build
# test files located in 
data/ 

# Usage 
input image: the image to apply the canny edge detector and do hough transform 
hough threshold: the lowest possible intensity for votes for a point
local maximum: the radius for local maximum window pass 4 for window size of 9
# running the program
./hough_transform <input image> <threshold> <local_maximum>

#the program will display images as well as write each stage of the detection process
