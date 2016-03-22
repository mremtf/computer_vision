

#USAGE

# input image one the image that you are checking for movement and corners
# input image two the next time in the time interval 
# window radius for the multi view
# pyramid levels number of level to feature track
# the harris threshold limit
# test checker image to see corner  pass ../data/checkerboard.jpg

./harris_corner_feature_tracking <input image one > <input image two> <window radius> <pyramid_levels> <threshold> <test checker image>
