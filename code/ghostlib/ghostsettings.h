#pragma once
#define INPUT_KINECT	1
#define INPUT_OPENNI	2
#define INPUT_VI		3

#define GHOST_INPUT INPUT_OPENNI

#define CAPTURE_KINECT 1
#define CAPTURE_OPENNI 2
#define CAPTURE_KINECT2 3

#define GHOST_CAPTURE CAPTURE_KINECT2

#define TABLET_GUI 0
#define FULL_VIDEO_CAPTURE 1
#define VIDEO_DUMP_IMMEDIATELY 1
#define DUMP_DEPTH 0

//if this is 0, will not try to initialize the kinect manager
//to do: replace kinect functions with "offline" functions
#define INIT_KINECT 1

#define GH_DEBUG_CYLPROJ 0