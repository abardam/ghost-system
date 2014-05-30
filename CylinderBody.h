#pragma once

#include <vector>
#include <TooN\TooN.h>
#include <TooN\se3.h>
#include <array>

#include "definitions.h"

#define MAXFRAMES 10000

class CylinderBody{
private:
	std::vector<SkeleVideoFrame> * vidRecord;
	std::vector<std::vector<SkeleVideoFrame *>> vidRecordBins;
public:

	void setVidRecord(std::vector<SkeleVideoFrame> * vidRecord);

	//to do: friend function
	//dont use these if you are not loader::initAndBuild
	float partRadii[NUMLIMBS];
	float leftOffset[NUMLIMBS];
	float rightOffset[NUMLIMBS];
	cv::Mat limbTransforms[NUMLIMBS][MAXFRAMES];

	//new skool
	//fixed cylinder radii
	//based on ratio of radius to body part length in the input img
	float fixedPartRadii[NUMLIMBS];
	
	//ellipse fitting
	float varianceX[NUMLIMBS];
	float varianceY[NUMLIMBS];

	//newskool
	float newPartRadii[NUMLIMBS];
	float newLeftOffset[NUMLIMBS];
	float newRightOffset[NUMLIMBS];
	
	//newskool_cyl
	float newPartRadii_cyl[NUMLIMBS];
	float newLeftOffset_cyl[NUMLIMBS];
	float newRightOffset_cyl[NUMLIMBS];

	//convenience
	int bestFrame;

	//debug
	cv::Mat blobs;


	void calcFacings();
	int getBestFrame(cv::Vec3f facing, int frameno, bool writeim = false);
	int getBestFrame(cv::Mat s, bool writeim = false);
	int getBestFrameBin(cv::Mat s, int bin, bool writeim = false);
	void getBestFrames(cv::Mat points, int arr[NUMLIMBS]);

	IMGPIXEL getColorAtPartAndPixel(int frame, int part, cv::Vec4f pixel, cv::Vec2f * pixelLoc);
	cv::Mat getCylinderTransform(int part, int frame);
	void calcLimbTransforms();

	void regularizeLimbs();
	void validateLimbs();

	void Reset();
	void colorsAtPixels(std::vector<cv::Vec4f> * pt3d, cv::Mat unrotmat2, int bestFrame, int part, std::vector<std::pair<IMGPIXEL, cv::Vec3f>> * colorPointVector, cv::Mat * toColorPic = NULL, bool whitePixels = false);
	void colorsAtPixels(std::vector<VecPart> * pt3d, cv::Mat unrotmat2[NUMLIMBS], int bestFrame, cv::Mat * out, cv::Mat * toColorPic = NULL, bool whitePixels = false);
	void colorsAtPixels(cv::Mat pt3d[NUMLIMBS], cv::Mat unrotmat2[NUMLIMBS], int bestFrame, cv::Mat * out, int * order, cv::Mat * toColorPic = NULL, bool whitePixels = false, bool shittyOption = false);
	void colorsAtPixelsMultiFrame(cv::Mat pt3d[NUMLIMBS], cv::Mat unrotmat2[NUMLIMBS], int bestFrames[NUMLIMBS], cv::Mat * out);

	void calcVidRecordBins();

	void Save(std::string path);
	void Load(std::string path);

};

cv::Mat normalizeSkeleton(cv::Mat skel);