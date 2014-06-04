#pragma once

#include <TooN\TooN.h>
#include <opencv2\opencv.hpp>
#include <array>
#include "ghostsettings.h"

#define GH_MF_OLD 0
#define GH_MF_CYLPROJ 1
#define GH_MODELFITTING GH_MF_CYLPROJ

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

struct Skeleton{
	cv::Mat points;
	std::array<float, NUMJOINTS> states;
	int initSkeleton;

	Skeleton():points(4,NUMJOINTS,cv::DataType<float>::type){
		initSkeleton = 0;
	}

	Skeleton(const Skeleton& s){
		points = s.points.clone();
		states = s.states;
		initSkeleton = s.initSkeleton;
	}
};


struct SkeleVideoFrame{
	//Skeleton skeleton; //skeleton should be separate from SkeleVideoFrame; SkeleVideoFrame is for UNPROCESSED capture
	cv::Mat cam2World; //multiply this with kinectPoints in order to get the skeleton
	Skeleton kinectPoints; //4xNUMJOINTS matrix
	cv::Mat kinectPoints2P; //should be normalized na
	cv::Vec3f facing;
	CroppedCvMat videoFrame; //image of the actor only; cropped to save space, contains offset
	cv::Mat depthFrame; //depth frame, usually emptied to save space
	cv::Mat fullVideoFrame;
	int cluster;
	bool allPartsIn;

	SkeleVideoFrame(){}
	SkeleVideoFrame(const SkeleVideoFrame& svf){
		cam2World = svf.cam2World.clone();
		kinectPoints = svf.kinectPoints;
		kinectPoints2P = svf.kinectPoints2P.clone();
		facing = svf.facing;
		videoFrame = svf.videoFrame;
		depthFrame = svf.depthFrame;
		fullVideoFrame = svf.fullVideoFrame;
		cluster = svf.cluster;
		allPartsIn = svf.allPartsIn;
	}
};

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
	cv::Mat partWeights[NUMLIMBS];
};

void initDefinitions();

lmap * getLimbmap();
std::vector<int> * getJointmap();
std::vector<std::vector<int>> getCombineParts();
int * getCombinePartsMap();
cv::Mat getPartWeights(int i);
unsigned int getLimbWeight(unsigned int jt, unsigned int jt2);

//for debugging
cv::Scalar getLimbColor(int limb, int numChannels=3);