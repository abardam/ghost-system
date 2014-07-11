#pragma once

#include <TooN\TooN.h>
#include <opencv2\opencv.hpp>
#include <array>
#include "ghostsettings.h"

#define GH_MF_OLD 0
#define GH_MF_CYLPROJ 1
#define GH_MODELFITTING GH_MF_CYLPROJ

#define GH_CMAPPING

#define IMAGE_CHANNELS 3
typedef cv::Vec<unsigned char, IMAGE_CHANNELS> IMGPIXEL;

struct CroppedCvMat{
	cv::Mat mat;
	cv::Point2i offset;
	CroppedCvMat():offset(0,0){}

	void clear();
};

cv::Mat uncrop(CroppedCvMat);


#if GHOST_CAPTURE == CAPTURE_OPENNI

#define NUMLIMBS 11
#define NUMJOINTS 16

#define JOINT_CENTER_HIP 15

#endif


#define WHITE (IMGPIXEL(0xff, 0xff, 0xff))
#define BLUE (IMGPIXEL(0xff, 0, 0))
#define NUMBINS 10.0
#define MAXUSERS 10

const int HEAD = 0;
const int UPPERARM_LEFT = 1;
const int UPPERARM_RIGHT = 2;
const int LOWERARM_LEFT = 3;
const int LOWERARM_RIGHT = 4;
const int CHEST = 5;
const int ABS = 6;
const int UPPERLEG_LEFT = 7;
const int UPPERLEG_RIGHT = 8;
const int LOWERLEG_LEFT = 9;
const int LOWERLEG_RIGHT = 10;

//capture type
const int CT_KINECT = 0;
const int CT_FILE = 1;

typedef std::pair<int,int> lmap;

#define CLAMP_SIZE(x,y,w,h) (x>=0 && x < w && y>=0 && y < h)


struct FrameSkeletonDepth{
	cv::Mat frame;
	cv::Vec4f skeletonPositions4[NUMJOINTS];
	cv::Vec2f skeletonPositions2[NUMJOINTS];
};

//used in cylindermapping, cylinderbody
struct VecPart{
	cv::Vec4f vec;
	int part;
};

//used here!
struct Mapping{
	lmap limbmap[NUMLIMBS];
	std::vector<int> jointmap[NUMJOINTS];
	std::vector<std::vector<int>> combineParts;
	int combinePartsMap[NUMLIMBS];
	float partWeights[NUMLIMBS][NUMJOINTS];
};

void initDefinitions();

lmap * getLimbmap();
std::vector<int> * getJointmap();
std::vector<std::vector<int>> getCombineParts();
int * getCombinePartsMap();
float * getPartWeights(int i);
unsigned int getLimbWeight(unsigned int jt, unsigned int jt2);

//for debugging
cv::Scalar getLimbColor(int limb, int numChannels=3);