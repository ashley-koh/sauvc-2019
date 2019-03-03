#
# dataset/extract.py
# SAUVC
# Extracts the frames of the videos into images
#

import os
import cv2
import argparse
from multiprocessing import Pool, cpu_count

# Extract the frames of the video specified by the given video_filename into 
# the given directory out_dir.
# Writes frames in the path <out_dir>/<video_name>_<frame_index>.jpg
def extract_frames(video_filename, out_dir):
    # Extract video name frm full filename
    video_basename = os.path.basename(video_filename)
    video_name = video_basename[:video_filename.index(".")]

    # Extract frames using opencv2 video capture
    video_capture = cv2.VideoCapture(video_filename)
    # check if video_capture is opened correctly
    if video_capture is None or not video_capture.isOpened():
        raise ValueError("Could not open video file at path: " + video_filename)
    
    # Setup output directory
    if not os.path.exists(out_dir): os.mkdir(out_dir)
    
    i_frame = 0
    read_sucessful = True

    # Keep extracting frames until read failure
    while read_sucessful:
        read_sucessful, frame = video_capture.read()
        # write frame in format specified in function  docs
        cv2.imwrite("{}/{}__{}.jpg".format(out_dir, video_name, i_frame), frame)
        i_frame += 1

if __name__ == "__main__":
    ## Parse program options
    #parser = argparse.ArgumentParser(prog=__file__, description="extract video frames from video")
    #parser.add_argument('video', help='The filename of the video to extract frames from')
    #parser.add_argument('out_dir', help='the output directory of the extracted video frames')
    #options = parser.parse_args()

    ## Perform extraction 
    #extract_frames(options.video, options.out_dir)

    # get classes from directories
