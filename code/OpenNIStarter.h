#pragma once
#include <OpenNI.h>
#include <NiTE.h>
#include <string>

namespace KINECT{
	bool initNI();
	void initFromFile(std::string);
	void shutdownNI();

	void getColorData(unsigned char * );
	void getDepthData(openni::DepthPixel * );
	void getDepthDataForDisplay(unsigned char * );
	void getRandomData(unsigned char * );
	bool getSkeletonData(nite::Skeleton *);
	void getPlayerColorData(unsigned char *, int * offsetX, int * offsetY, int * maxX, int * maxY);
	void setRefresh(bool r);

	void mapDepthToSkeleton(float *dX, float *dY, float *dZ, float *sX, float *sY, float *sZ);
	void mapSkeletonToDepth(float *sX, float *sY, float *sZ, float *dX, float *dY, float *dZ);

	openni::VideoStream * getDepthStream();

	void cycleTrackedSkeleton();
}