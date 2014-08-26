#pragma once

#include "definitions.h"

class CylinderBody;

struct Skeleton{
	//joint points; 4xNUMJOINTS
	cv::Mat points;

	//joint points offset per limb; 4x(2*NUMJOINTS)
	//access using 2*limbid + 0 or 1 (for first or second)
	cv::Mat offsetPoints;

	bool offsetPointsCalculated;

	std::array<float, NUMJOINTS> states;
	int initSkeleton;

	Skeleton():points(4,NUMJOINTS,cv::DataType<float>::type), 
		offsetPoints(4,2*NUMJOINTS,CV_32F),
		initSkeleton(0),
		offsetPointsCalculated(false)
	{}

	Skeleton(const Skeleton& s){
		points = s.points.clone();
		offsetPoints = s.offsetPoints.clone();
		states = s.states;
		initSkeleton = s.initSkeleton;
	}

	void calculateOffsetPoints(const CylinderBody& cb);

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