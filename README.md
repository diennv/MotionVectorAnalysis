This module detects a frame contains moving objects utilizing motion vector 
information from mpeg4 frame.

# Extracting motion vector

To extract motion vector "+export_mvs" options need to be set when ffmpeg codec is
initilized.


``` 
AVDictionary *opts = nullptr;
av_dict_set(&opts, "flags2", "+export_mvs", 0);
avcodec_open2(vstrm->codec, vcodec, &opts);
```

When a frame is decoded with `avcodec_decode_video2()` or `avcodec_receive_frame()`, 
the motion vector becomes available to use in the sideband data structure
, which can be obtained with `av_frame_get_side_data()`.
```
AVFrameSideData* sd = av_frame_get_side_data(decframe, AV_FRAME_DATA_MOTION_VECTORS);
AVMotionVector* mvs = (AVMotionVector*)sd->data;
int mvcount = sd->size / sizeof(AVMotionVector);
```

# Detecting Motion
The extracted motion vector is passed to `process_frame()` in `MVDetector`.
Inside the function, the occupancy map is deteremined basd on the motion vectors.
The occupancy_map is 8x8 grid and deemed to be occupied when the motion vector 
originating the grid cell is found. 

After an occupancy map for each frame is found, the average occupancy for last N 
frames is calculated. Which aims to reduce temporarily noise. The default value 
is set to 3 (`MVDetector::DEFAULT_WINDOW_SIZE`). 

At each frame the total number of grid in the occupancy map whose values is above a
a threshold, currently defaulted to 0.6 (`MVDetector::DEFAULT_LOCAL_OCCUPANCY_AVG_THRESHOLD),
is counted. When this number is above a thredhold (DEFAULT_OCCUPANCY_THRESHOLD),
the frame is finally decided to contain a `movement`.

In order to reduce flickering effect where a continus sequence of frame 









