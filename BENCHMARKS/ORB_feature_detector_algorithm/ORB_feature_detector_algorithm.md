# <center>Oriented FAST and Rotated BRIEF (ORB) Feature Detection Algorithm</center>

### __Introduction__
This algorithm is used for detecting and matching features in an image. It is a royalty-free and more computationally efficient alternative to SIFT and SURF, and is already implemented in the OpenCV library. However in this case, only the _feature detector_ part is going to be benchmarked. The test bench going to be used is an i5-8400 with 16 GB of RAM.

### __Parameters__
- __nfeatures__ : The maximum number of best features to retain, set to a very high value(10000) to retain all features in our case.

- __scaleFactor__ : Pyramid decimation ratio, greater than 1. Range [1.2, 1.5, 2, 3]

- __nlevels__ : The number of pyramid levels. The smallest level will have linear size equal to _input_image_linear_size/pow(scaleFactor, nlevels - firstLevel)_. Using the default value of 8.

- __edgeThreshold__ : This is size of the border where the features are not detected. It should roughly match the _patchSize_ parameter. Set to 50 pixels in our case.

- __firstLevel__ : The level of pyramid to put source image to. Previous layers are filled with upscaled source image. Using the default value of 0.

- __WTA_K__ : The number of points that produce each element of the oriented BRIEF descriptor. Range [2, 3, 4]

- __scoreType__ : This is the type of scoring to be used, either HARRIS_SCORE(default) or FAST_SCORE 

- __patchSize__ : Size of the patch used by the oriented BRIEF descriptor. Set equal to _edgeThreshold_.

- __fastThreshold__ : The fast thresholding parameter. Using default value of 20.

### __Test Cases__
A single video file, with a resolution of 1920x1080 @ 30 FPS (Total 750 frames) has been used as the test case. The file has been taken [this](https://motchallenge.net/vis/MOT17-13-SDP) online database (RAW sequence). Before use, it was converted from .webm format to .mp4, encoded using a FFMPEG codec recognized by OpenCV. 

### __Results__


### __Footnotes__
This is the [original paper](https://ieeexplore.ieee.org/document/6126544) that details this algorithm. This algorithm is quite efficient, and suitable for use on onboard computers. The _scaleFactor_ parameter has a significant effect on computational efficiency, and increasing it lessens the computational effort required, at the cost of quality of features (which might cause problems with feature matching in later stages). _scoreType_ doesn't have a lot of effect on computational power (FAST_SCORE only slightly improves it, but produces slightly less stable keypoints). 


This document is an example and hasn't been documented extensively, hence many of the variables in the _Parameters_ section have been set to their default values. In reality, you might want to vary every significant parameter to see what effect it has on the result. The key is striking a balance; you don't want to be wasting too much time on conducting tests and recording results by varying each and every parameter (especially those that have a negligible effect on your result) and at the same time you shouldn't conduct very few tests, which might be insufficient to get a clear picture and also optimise your performance. In any case, every relevant information must be given so that the tests can be recreated or the documentation can be made more extensive, if the need arises.