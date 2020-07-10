# This is a custom python program for benchmarking the ORB feature detector that comes with OpenCV.
# It uses the line_profiler library for measuring time taken to run the getKeyPoints function, which is where all the 
# magic happens (i.e. where we actually detect the features in a given frame).
# It also plots this data, along with the number of features detected in each frame. This plotting is done with the help
# of matplotlib.

import os
import numpy as np
import cv2
from matplotlib import pyplot as plt
from line_profiler import LineProfiler
from itertools import product

# This function takes the create orb object and an image frame, and outputs the feature keypoints in the frame 
def getKeyPoints(orb, frame):
    kp = orb.detect(frame, None)
    kp, des = orb.compute(frame, kp)
    return kp

def main(program_dir, scale_factor, wta_k, score_type):
    # num_features_list stores the number of features detected in each frame.
    # time stores the time taken in miliseconds to get features in each frame, by the function getKeyPoints
    # num_frames stores the number of frames  
    num_features_list = []
    time = []
    num_frames = 0

    # Created the ORB feature matcher object, with the requisite arguments (some default some taken from command line)
    orb = cv2.ORB_create(nfeatures=10000, edgeThreshold=50, patchSize=50, fastThreshold=20, scaleFactor=scale_factor, WTA_K=wta_k ,scoreType=score_type)

    # Place your data set in the BENCHMARKS/ORB_feature_detector_algorithm/datasets directory. Make sure it is encoded in FFMPEG
    # format for OpenCV to use it.
    cap = cv2.VideoCapture(program_dir + '/datasets/Raw.mp4')
    if cap.isOpened() == False:
        print("Error opening video file.")
        return

    while cap.isOpened():
        ret, frame = cap.read()
        if np.shape(frame) == ():
            break
        num_frames = num_frames + 1

        # This section computes the time taken in each function call of getKeyPoints. Instead of calling getKeyPoints directly,
        # you call the function through a wrapper which measures the time taken for it's execution.        
        profile = LineProfiler()
        profile_wrapper = profile(getKeyPoints)
        kp = profile_wrapper(orb, frame)
        
        # This part is a bit tricky to understand. Basically, the data variable here is a dictionary. Here is an example of
        # this data variable {('ORB_feature.py', 6, 'getKeyPoints'): [(7, 1, 90299), (8, 1, 27768), (9, 1, 2)]} 
        # I also don't know why it is this complicated :((
        # The keys consist of tuples of the type ('path/to/python/program', line_number_of_function_definition, 'function_name').
        # The items consist of a list of tuples, each tuple corresponding to each line in the function definition
        # The tuple is of the type (line_number, number_of_times_the_line_ran, time_taken_in_microseconds)
        # Since I am benchmarking only one function, I used the first and only key in list of keys (data.keys()) as my key,
        # and added the time taken at each line using the last index of tuples corresponding to each line to get the total
        # time of execution of the function. I then stored it in the time list, in ms.
        data = profile.get_stats().timings
        time_taken = data[list(data.keys())[0]][0][2] + data[list(data.keys())[0]][1][2] + data[list(data.keys())[0]][2][2]
        time.append(time_taken/1000.0)
        num_features_list.append(len(kp))

        frame = cv2.drawKeypoints(frame, kp, None, color=(0, 255, 0), flags=0)

    cap.release()

    frame_list = range(1, num_frames+1)

    # This part prepares plots of the requisite data, and saves it in an SVG file, which is unique to the parameters passed.
    plt.figure(figsize=(25, 10))
    plt.plot(frame_list, num_features_list)
    plt.xlabel('Frame Number')
    plt.ylabel('Detected Features')
    plt.title('Plot of Detected Features in Each Frame')
    plt.tight_layout()
    plt.savefig(program_dir + '/plots/features_number_plot_' + str(scale_factor) + '_' + str(wta_k) + '_' + ('HARRIS' if score_type == cv2.ORB_HARRIS_SCORE else 'FAST') + '.svg')

    plt.clf()
    
    plt.figure(figsize=(25, 10))
    plt.plot(frame_list, time)
    plt.xlabel('Frame Number')
    plt.ylabel('Time Taken for Identifying Features(ms)')
    plt.title('Plot of Time Taken for Identifying Features in Each Frame')
    plt.tight_layout()
    plt.savefig(program_dir + '/plots/time_plot_' + str(scale_factor) + '_' + str(wta_k) + '_' + ('HARRIS' if score_type == cv2.ORB_HARRIS_SCORE else 'FAST') + '.svg')
    
    plt.close('all')

    # Returns the average time taken/frame and the average number of features detected/frame
    return sum(time)/len(time), sum(num_features_list)/len(num_features_list)

if __name__ == "__main__":    
    # This is list of the range of variables that you want to benchmark.
    # First list is the list of scaleFactor values to be tested.
    # Second list is the list of WTA_K values to be tested.
    # Third list is the list of scoreType values to be tested.
    list_args = [[1.2, 1.5, 2, 2.5], [2, 3, 4], [cv2.ORB_HARRIS_SCORE, cv2.ORB_FAST_SCORE]]
    
    # This produces all the possible combinations of these parameters, for benchmarking
    comb_args = list(product(*list_args))

    # os.path.dirname returns None if the current directory is .
    # To circumvent that this entire code block was written :(
    path = os.path.dirname(os.sys.argv[0])
    if path == '':
        path = '.'

    for i in comb_args:
        computation, features = main(path, i[0], i[1], i[2])

        # This really long print statement is for printing the results in a format a LaTeX table can understand, so that you can automatically port it to a LaTeX table
        # Automation 100 :)
        print(i[0], '&',\
              i[1], '&',\
              'HARRIS\\_SCORE' if i[2] == cv2.ORB_HARRIS_SCORE else 'FAST\\_SCORE', '&',\
              '%.2f'%(computation), '&',\
              '%.2f'%(features), '\\\\', \
              sep=' ')
