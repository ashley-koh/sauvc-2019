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
from shutil import rmtree

# Extract the frames of the video specified by the given video_filename into 
# the given directory out_dir.
# Writes frames in the path <out_dir>/<video_name>_<frame_index>.jpg 
# note if frame at path already exists will overwrite frame
# if verbose is specified as true wlll print progress infomation
# sampling rate specifies ratio of the no of frames extracted to the total number 
# of frames
def extract_frames(video_filename, out_dir, sampling_rate, verbose=True):
    # Extract video name frm full filename
    video_basename = os.path.basename(video_filename)
    video_name = video_basename[:video_basename.index(".")]
    print("bname: ", video_basename, " video_name: ", video_name)

    # Extract frames using opencv2 video capture
    video_capture = cv2.VideoCapture(video_filename)
    # check if video_capture is opened correctly
    if video_capture is None or not video_capture.isOpened():
        raise ValueError("Could not open video file at path: " + video_filename)

    # Determine the interval in which to sample frames
    # obtain number of frames in video
    total_frames = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
    n_frames = int(total_frames * sampling_rate) # no of frames to sample
    n_interval = int(total_frames / n_frames) # no of frame to process per sampling step
    if verbose: print("to read {} frames".format(n_frames))
                   
    # Setup output directory
    if not os.path.exists(out_dir): os.mkdir(out_dir)
    
    # Keep extracting frames until read failure
    current_frame = 0
    for i_frame in range(0, total_frames, n_interval):
        # read frames, skiping frames not included in the smaple
        while current_frame <= i_frame:
            read_sucessful, frame = video_capture.read()
            current_frame += 1
        if not read_sucessful:
            raise ValueError("FATAL to read frame: {} ".format(i_frame))

        # write frame in format specified in function  docs
        cv2.imwrite("{}/{}__{}.jpg".format(out_dir, video_name, i_frame), frame)
    
        if verbose:
            # display progress infomation
            progress = i_frame / total_frames * 100 # compute progress percentage
            print("extracted", video_name, " progress: {:.2f}%".format(progress))

if __name__ == "__main__":
    ## Parse program options
    parser = argparse.ArgumentParser(prog=__file__, description="extract video frames from video dataset")
    parser.add_argument('-s', help='The filename of the video to extract frames from')
    options = parser.parse_args()

    ## Perform extraction 
    # Create pictures directory
    pics_dir = "data/images"
    if os.path.exists(pics_dir): rmtree(pics_dir)
    if not os.path.exists(pics_dir): os.mkdir(pics_dir)
    
    # perform extraction on multiple processes
    def extractor(vid_info):
        i_vid, vid_path = vid_info
        print("extractor: prcoessing video: ", i_vid)
        data_dir, video_dir, class_name, video_filename = vid_path.split("/")
        extract_frames(vid_path, out_dir="{}/{}".format(pics_dir, class_name),
                       sampling_rate=0.01)

    vid_infos = list(enumerate(glob("data/video/*/*")))[:1]
    print("to extract {} videos".format(len(vid_infos)))
    
    pool = Pool(processes=cpu_count())
    pool.map(extractor, vid_infos)
