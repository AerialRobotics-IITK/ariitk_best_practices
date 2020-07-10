# Testing Algorithms

This gist details on how to correctly test algorithms and document their results. 

## Points to Remember

1) First and foremost, you should have clarity on what exactly is your algorithm trying to achieve. This means you should list down all variables that you are trying to minimize/maximize with your algorithm.  

2) List all parameters your algorithm requires. If you are absolutely sure that a particular parameter doesn't affect performance then you can omit it. 

3) Now you need to make test cases for your algorithm. Your battery of test cases should reasonably cover as many use cases as possible.

4) Next you should eliminate all variations in external factors that can affect test performance (this, obviously isn't fully in your control, but you should try as much as possible to make the test fair).

5) You can finally move onto testing now. Run your algorithm with different combinations of parameters (as detailed in step 2) on different test cases. Measure all the output variables of interest. 


#### _Example Algorithm test_
Say you want to test an algorithm for the detection of boxes, which finds contours using canny edge.

1) The first step would be to list down what all you are trying to achieve.
    - Minimising __false negatives__ (i.e. correct box not detected)
    
    - Minimising __false positives__ (i.e. incorrect box/other objects detected as boxes)

    - Minimising __computational power__

2) Prepare a list of parameters that your algorithm takes
    - __Threshold_1__ :- First threshold for the hysteresis procedure
    - __Threshold_2__ :- Second threshold for the hysteresis procedure
    - __Aperture Size__ :- Aperture Size for Sobel operator
    - __L2gradient__ :- Whether to use L<sub>2</sub> gradient or L<sub>1</sub> gradient.  
    There are many more parameters, for example the parameters of the gaussian blur. These parameters can be omitted from your test bench.

3) Make test cases. In this case, since you are testing an image processing algorithm, record/obtain video files covering as many angles, lighting conditions, and objects as possible.

4) Eliminate all variations in external factors, if any. For example, you need to make sure that you are using the video file and same computational resources, in between runs. Also close any background applications to prevent lag spikes. 

5) Now, using this video/bag file, run your tests using different variations of parameters listed in point 2. Record the number of false negatives, false positives(you can record this as the fraction of frames that gave false positives or false negatives), and computational power (CPU usage/time taken by your function/code) being used.
