#
# dataset/extract.py
# SAUVC
# Extracts the frames of the videos into images
#

import os
import cv2
import argparse
from multiprocessing import Pool, cpu_count
from glob import glob

# Extract the frames of the video specified by the given video_filename into 
# the given directory out_dir.
# Writes frames in the path <out_dir>/<video_name>_<frame_index>.jpg 
# note if frame at path already exists will overwrite frame
# if verbose is specified as true wlll print progress infomation
def extract_frames(video_filename, out_dir, verbose=True):
    # Extract video name frm full filename
    video_basename = os.path.basename(video_filename)
    video_name = video_basename[:video_filename.index(".")]

    # Extract frames using opencv2 video capture
    video_capture = cv2.VideoCapture(video_filename)
    # check if video_capture is opened correctly
    if video_capture is None or not video_capture.isOpened():
        raise ValueError("Could not open video file at path: " + video_filename)
    # obtain number of frames in video
    n_frames = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
                   
    # Setup output directory
    if not os.path.exists(out_dir): os.mkdir(out_dir)
    
    # Keep extracting frames until read failure
    for i_frame in range(n_frames):
        read_sucessful, frame = video_capture.read()
        if not read_sucessful:
            raise ValueError("FATAL to read frame: {} ".format(i_frame))
        # write frame in format specified in function  docs
        cv2.imwrite("{}/{}__{}.jpg".format(out_dir, video_name, i_frame), frame)
    
        if verbose:
            # display progress infomation
            progress = i_frame / n_frames * 100 # compute progress percentage
            print("extracted", video_name, " progress: {:.2f}%".format(progress))

if __name__ == "__main__":
    ## Parse program options
    parser = argparse.ArgumentParser(prog=__file__, description="extract video frames from video dataset")
    parser.add_argument('-s', help='The filename of the video to extract frames from')
    options = parser.parse_args()


    ## Perform extraction 
    # Create pictures directory
    pics_dir = "data/images"
    if not os.path.exists(pics_dir): os.mkdir(pics_dir)
    
    # perform extraction on multiple processes
    def extractor(vid_info):
        i_vid, vid_path = vid_info
        print("extractor: prcoessing video: ", i_vid)
        data_dir, video_dir, class_name, video_filename = vid_path.split("/")
        extract_frames(vid_path, out_dir="{}/{}".format(pics_dir, class_name))

    vid_infos = list(enumerate(glob("data/video/*/*")))[:1]
    print("to extract {} videos".format(len(vid_infos)))
    
    pool = Pool(processes=cpu_count())
    pool.map(extractor, vid_infos)
