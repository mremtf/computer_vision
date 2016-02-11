# Matthew England

# The application is C++ 11, OpenCV, and CMake

# location of executable
Stored in bin and build
# test files located in 
data/ 

# Usage 
input image: the image to apply the canny edge detector 
low threshold: the lowest possible intensity for an edge
high threshold: the highest possible intensity to be considered a true edge 
sigma: the variance of blur on the image

# running the program
./canny_edge_detector <input image> <low threshold> <high threshold> <sigma>

#the program will display images as well as write each stage of the detection process
