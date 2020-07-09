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

# This function takes the create orb object and an image frame, and outputs the feature keypoints in the frame 
def getKeyPoints(orb, frame):
    kp = orb.detect(frame, None)
    kp, des = orb.compute(frame, kp)
    return kp

def main():
    # num_features_list stores the number of features detected in each frame.
    # time stores the time taken in miliseconds to get features in each frame, by the function getKeyPoints
    # num_frames stores the number of frames  
    num_features_list = []
    time = []
    num_frames = 0

    # Created the ORB feature matcher object, with maximum number of detected features as the argument. You can either limit the 
    # number of features to save computation, or put a very high number, in which case it will detect as many features as it can.
    orb = cv2.ORB_create(nfeatures=10000)

    # Place your data set in the BENCHMARKS/ORB_feature_detector_algorithm/datasets directory. Make sure it is encoded in FFMPEG
    # format for OpenCV to use it.
    cap = cv2.VideoCapture(os.path.dirname(os.sys.argv[0]) + '/datasets/Raw.mp4')
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
        cv2.imshow("FEATURES", frame)

        if cv2.waitKey(25) & 0xff == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    frame_list = range(1, num_frames+1)

    # This part prepares plots of the requisite data, and saves it in an SVG file
    plt.figure(figsize=(25, 10))
    plt.plot(frame_list, num_features_list)
    plt.xlabel('Frame Number')
    plt.ylabel('Detected Features')
    plt.title('Plot of Detected Features in Each Frame')
    plt.tight_layout()
    plt.savefig(os.path.dirname(os.sys.argv[0]) + '/plots/features_number_plot.svg')

    plt.clf()
    
    plt.figure(figsize=(25, 10))
    plt.plot(frame_list, time)
    plt.xlabel('Frame Number')
    plt.ylabel('Time Taken for Identifying Features(ms)')
    plt.title('Plot of Time Taken for Identifying Features in Each Frame')
    plt.tight_layout()
    plt.savefig(os.path.dirname(os.sys.argv[0]) + '/plots/time_plot.svg')

if __name__ == "__main__":    
    main()