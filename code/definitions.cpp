#include "definitions.h"
#include "KinectManager.h"

cv::Scalar white(int channels){
	if(channels == 1)
		return cv::Scalar(255);
	else if(channels == 3)
		return cv::Scalar(255,255,255);
	else if(channels == 4)
		return cv::Scalar(255,255,255,255);
	return cv::Scalar();
}

cv::Mat uncrop(CroppedCvMat ccm){
	cv::Mat u(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, ccm.mat.type(), white(ccm.mat.channels()));
	cv::Mat tmp = u(cv::Rect(ccm.offset.x, ccm.offset.y, ccm.mat.cols, ccm.mat.rows));
	ccm.mat.copyTo(tmp);
	return u;
}


Mapping mapping;
cv::Scalar colors[NUMLIMBS+1];
bool defInit = false;
unsigned int limbWeights[NUMLIMBS][NUMLIMBS];

lmap * getLimbmap(){
	return mapping.limbmap;
}

std::vector<int> * getJointmap(){
	return mapping.jointmap;
}

std::vector<std::vector<int>> getCombineParts(){
	return mapping.combineParts;
}

int * getCombinePartsMap(){
	return mapping.combinePartsMap;
}

void initDefinitions(){
	if(defInit) return;

	KINECT::initMapping(&mapping);

	colors[0] = cv::Scalar(125, 0, 0);
	colors[1] = cv::Scalar(75, 180, 0);
	colors[2] = cv::Scalar(0, 125, 0);
	colors[3] = cv::Scalar(75, 0, 75);
	colors[4] = cv::Scalar(0, 75, 75);
	colors[5] = cv::Scalar(125, 75, 0);
	colors[6] = cv::Scalar(125, 0, 75);
	colors[7] = cv::Scalar(0, 200, 75);
	colors[8] = cv::Scalar(50, 75, 250);
	colors[9] = cv::Scalar(50, 250, 50);
	colors[10] = cv::Scalar(50, 0, 160);
	colors[11] = cv::Scalar(255,255,255);

	limbWeights[HEAD][HEAD]=1;
	limbWeights[HEAD][UPPERARM_LEFT]=1;
	limbWeights[HEAD][UPPERARM_RIGHT]=1;
	limbWeights[HEAD][LOWERARM_LEFT]=0;
	limbWeights[HEAD][LOWERARM_RIGHT]=0;
	limbWeights[HEAD][CHEST]=0;
	limbWeights[HEAD][ABS]=0;
	limbWeights[HEAD][UPPERLEG_LEFT]=0;
	limbWeights[HEAD][UPPERLEG_RIGHT]=0;
	limbWeights[HEAD][LOWERLEG_LEFT]=0;
	limbWeights[HEAD][LOWERLEG_RIGHT]=0;

	limbWeights[UPPERARM_LEFT][HEAD]=0;
	limbWeights[UPPERARM_LEFT][UPPERARM_LEFT]=1;
	limbWeights[UPPERARM_LEFT][UPPERARM_RIGHT]=0;
	limbWeights[UPPERARM_LEFT][LOWERARM_LEFT]=1;
	limbWeights[UPPERARM_LEFT][LOWERARM_RIGHT]=0;
	limbWeights[UPPERARM_LEFT][CHEST]=0;
	limbWeights[UPPERARM_LEFT][ABS]=0;
	limbWeights[UPPERARM_LEFT][UPPERLEG_LEFT]=0;
	limbWeights[UPPERARM_LEFT][UPPERLEG_RIGHT]=0;
	limbWeights[UPPERARM_LEFT][LOWERLEG_LEFT]=0;
	limbWeights[UPPERARM_LEFT][LOWERLEG_RIGHT]=0;

	limbWeights[UPPERARM_RIGHT][HEAD]=0;
	limbWeights[UPPERARM_RIGHT][UPPERARM_LEFT]=0;
	limbWeights[UPPERARM_RIGHT][UPPERARM_RIGHT]=1;
	limbWeights[UPPERARM_RIGHT][LOWERARM_LEFT]=0;
	limbWeights[UPPERARM_RIGHT][LOWERARM_RIGHT]=1;
	limbWeights[UPPERARM_RIGHT][CHEST]=0;
	limbWeights[UPPERARM_RIGHT][ABS]=0;
	limbWeights[UPPERARM_RIGHT][UPPERLEG_LEFT]=0;
	limbWeights[UPPERARM_RIGHT][UPPERLEG_RIGHT]=0;
	limbWeights[UPPERARM_RIGHT][LOWERLEG_LEFT]=0;
	limbWeights[UPPERARM_RIGHT][LOWERLEG_RIGHT]=0;

	limbWeights[LOWERARM_LEFT][HEAD]=0;
	limbWeights[LOWERARM_LEFT][UPPERARM_LEFT]=1;
	limbWeights[LOWERARM_LEFT][UPPERARM_RIGHT]=0;
	limbWeights[LOWERARM_LEFT][LOWERARM_LEFT]=1;
	limbWeights[LOWERARM_LEFT][LOWERARM_RIGHT]=0;
	limbWeights[LOWERARM_LEFT][CHEST]=0;
	limbWeights[LOWERARM_LEFT][ABS]=0;
	limbWeights[LOWERARM_LEFT][UPPERLEG_LEFT]=0;
	limbWeights[LOWERARM_LEFT][UPPERLEG_RIGHT]=0;
	limbWeights[LOWERARM_LEFT][LOWERLEG_LEFT]=0;
	limbWeights[LOWERARM_LEFT][LOWERLEG_RIGHT]=0;

	limbWeights[LOWERARM_RIGHT][HEAD]=0;
	limbWeights[LOWERARM_RIGHT][UPPERARM_LEFT]=0;
	limbWeights[LOWERARM_RIGHT][UPPERARM_RIGHT]=1;
	limbWeights[LOWERARM_RIGHT][LOWERARM_LEFT]=0;
	limbWeights[LOWERARM_RIGHT][LOWERARM_RIGHT]=1;
	limbWeights[LOWERARM_RIGHT][CHEST]=0;
	limbWeights[LOWERARM_RIGHT][ABS]=0;
	limbWeights[LOWERARM_RIGHT][UPPERLEG_LEFT]=0;
	limbWeights[LOWERARM_RIGHT][UPPERLEG_RIGHT]=0;
	limbWeights[LOWERARM_RIGHT][LOWERLEG_LEFT]=0;
	limbWeights[LOWERARM_RIGHT][LOWERLEG_RIGHT]=0;

	limbWeights[CHEST][HEAD]=0;
	limbWeights[CHEST][UPPERARM_LEFT]=1;
	limbWeights[CHEST][UPPERARM_RIGHT]=1;
	limbWeights[CHEST][LOWERARM_LEFT]=0;
	limbWeights[CHEST][LOWERARM_RIGHT]=0;
	limbWeights[CHEST][CHEST]=1;
	limbWeights[CHEST][ABS]=1;
	limbWeights[CHEST][UPPERLEG_LEFT]=1;
	limbWeights[CHEST][UPPERLEG_RIGHT]=1;
	limbWeights[CHEST][LOWERLEG_LEFT]=0;
	limbWeights[CHEST][LOWERLEG_RIGHT]=0;

	limbWeights[ABS][HEAD]=0;
	limbWeights[ABS][UPPERARM_LEFT]=1;
	limbWeights[ABS][UPPERARM_RIGHT]=1;
	limbWeights[ABS][LOWERARM_LEFT]=0;
	limbWeights[ABS][LOWERARM_RIGHT]=0;
	limbWeights[ABS][CHEST]=1;
	limbWeights[ABS][ABS]=1;
	limbWeights[ABS][UPPERLEG_LEFT]=1;
	limbWeights[ABS][UPPERLEG_RIGHT]=1;
	limbWeights[ABS][LOWERLEG_LEFT]=0;
	limbWeights[ABS][LOWERLEG_RIGHT]=0;

	limbWeights[UPPERLEG_LEFT][HEAD]=0;
	limbWeights[UPPERLEG_LEFT][UPPERARM_LEFT]=0;
	limbWeights[UPPERLEG_LEFT][UPPERARM_RIGHT]=0;
	limbWeights[UPPERLEG_LEFT][LOWERARM_LEFT]=0;
	limbWeights[UPPERLEG_LEFT][LOWERARM_RIGHT]=0;
	limbWeights[UPPERLEG_LEFT][CHEST]=0;
	limbWeights[UPPERLEG_LEFT][ABS]=1;
	limbWeights[UPPERLEG_LEFT][UPPERLEG_LEFT]=1;
	limbWeights[UPPERLEG_LEFT][UPPERLEG_RIGHT]=0;
	limbWeights[UPPERLEG_LEFT][LOWERLEG_LEFT]=1;
	limbWeights[UPPERLEG_LEFT][LOWERLEG_RIGHT]=0;

	limbWeights[UPPERLEG_RIGHT][HEAD]=0;
	limbWeights[UPPERLEG_RIGHT][UPPERARM_LEFT]=0;
	limbWeights[UPPERLEG_RIGHT][UPPERARM_RIGHT]=0;
	limbWeights[UPPERLEG_RIGHT][LOWERARM_LEFT]=0;
	limbWeights[UPPERLEG_RIGHT][LOWERARM_RIGHT]=0;
	limbWeights[UPPERLEG_RIGHT][CHEST]=0;
	limbWeights[UPPERLEG_RIGHT][ABS]=1;
	limbWeights[UPPERLEG_RIGHT][UPPERLEG_LEFT]=0;
	limbWeights[UPPERLEG_RIGHT][UPPERLEG_RIGHT]=1;
	limbWeights[UPPERLEG_RIGHT][LOWERLEG_LEFT]=0;
	limbWeights[UPPERLEG_RIGHT][LOWERLEG_RIGHT]=1;

	limbWeights[LOWERLEG_LEFT][HEAD]=0;
	limbWeights[LOWERLEG_LEFT][UPPERARM_LEFT]=0;
	limbWeights[LOWERLEG_LEFT][UPPERARM_RIGHT]=0;
	limbWeights[LOWERLEG_LEFT][LOWERARM_LEFT]=0;
	limbWeights[LOWERLEG_LEFT][LOWERARM_RIGHT]=0;
	limbWeights[LOWERLEG_LEFT][CHEST]=0;
	limbWeights[LOWERLEG_LEFT][ABS]=0;
	limbWeights[LOWERLEG_LEFT][UPPERLEG_LEFT]=1;
	limbWeights[LOWERLEG_LEFT][UPPERLEG_RIGHT]=0;
	limbWeights[LOWERLEG_LEFT][LOWERLEG_LEFT]=1;
	limbWeights[LOWERLEG_LEFT][LOWERLEG_RIGHT]=0;

	limbWeights[LOWERLEG_RIGHT][HEAD]=0;
	limbWeights[LOWERLEG_RIGHT][UPPERARM_LEFT]=0;
	limbWeights[LOWERLEG_RIGHT][UPPERARM_RIGHT]=0;
	limbWeights[LOWERLEG_RIGHT][LOWERARM_LEFT]=0;
	limbWeights[LOWERLEG_RIGHT][LOWERARM_RIGHT]=0;
	limbWeights[LOWERLEG_RIGHT][CHEST]=0;
	limbWeights[LOWERLEG_RIGHT][ABS]=0;
	limbWeights[LOWERLEG_RIGHT][UPPERLEG_LEFT]=0;
	limbWeights[LOWERLEG_RIGHT][UPPERLEG_RIGHT]=1;
	limbWeights[LOWERLEG_RIGHT][LOWERLEG_LEFT]=0;
	limbWeights[LOWERLEG_RIGHT][LOWERLEG_RIGHT]=1;

	defInit = true;
}

float * getPartWeights(int i){
	if(i >=0 && i<NUMLIMBS)
		return mapping.partWeights[i];
	else
		return 0;
}

cv::Scalar getLimbColor(int limb, int numChannels){
	if(numChannels == 3)
		return colors[limb];
	else{
		return cv::Scalar(colors[limb](0), 
			colors[limb](1),
			colors[limb](2),
			255);
	}
		
}

unsigned int getLimbWeight(unsigned int jt, unsigned int jt2){
	return limbWeights[jt][jt2];
}

void CroppedCvMat::clear(){
	mat = cv::Mat();
	offset = cv::Point();
}