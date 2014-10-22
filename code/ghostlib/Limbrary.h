#pragma once

#include "definitions.h"
#include "CylinderBody.h"

#define MIN_DELTA 10
#define FL_GOOD_RATIO 0.65
#define FL_WHITE_RATIO 0.1

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480


typedef std::vector<CroppedCvMat> FrameLimbs;

class Limbrary{
public:
	Limbrary();
	void build(std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cb, bool verbose = false, int startFrame = -1, int endFrame = -1);
	void Save(std::string path);
	void Load(std::string path);

	//index = frame. FrameLimbs is a vector with index = limb id
	std::vector<FrameLimbs> frames;

	//after clustering, we will delete every non-cluster limb image;
	//use this function to get a list of all the valid frame indices per limb id
	std::vector<int>& getAvailableFramesForLimb(int limbid) ;

	//should be the same vidRecord used to build
	//maybe its safer to put this as private
	void cluster(std::vector<SkeleVideoFrame> * vidRecord, unsigned int K=16, unsigned int iterations=20000);

	//use after clustering to get rid of non-main-cluster images
	void clean();

	//removes bad images (ex: mostly occluded pixels)
	void removeBadFrames();


private:

	//index = limb id. contains list of all valid frames. if empty, assume all vidRecord frames are valid
	std::vector<std::vector<int>> framesForLimb;


};