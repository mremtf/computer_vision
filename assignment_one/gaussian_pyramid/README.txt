
The code will create a gaussian pyramid over an input image

REQUIREMENTS:

Needs cmake application to build the 
C++ 11 libraries

Use makefile to build gaussian pyramid application

The application is located in the build/ folder
the application is called gp

the test files are located in ../data/lena.jpg and ../data/zebra.jpb

USAGE:
./gp <input image> <num levels> <kernel size> <sigma>

input_image the image you want to build a pyramid from
num levels the number of levels to generate for the pyramid
kernel size the size of the gaussian kernel (only supports 3 or 5)
sigma the variance of the gaussian kernel (only supports 1.0)

OUPUT:

Write the images out to the same directory the gp app runs

