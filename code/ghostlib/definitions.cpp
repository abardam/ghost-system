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
	cv::Mat u(ccm.origHeight, ccm.origWidth, ccm.mat.type(), white(ccm.mat.channels()));
	cv::Mat tmp = u(cv::Rect(ccm.offset.x, ccm.offset.y, ccm.mat.cols, ccm.mat.rows));
	ccm.mat.copyTo(tmp);
	return u;
}


Mapping mapping;
cv::Scalar colors[NUMLIMBS+1];
bool defInit = false;
unsigned int limbWeights[NUMLIMBS][NUMLIMBS];

// limbOccludes[limb 1][limb 2] = whether limb 1 occludes limb 2 (in building the limbrary)
bool limbOccludes[NUMLIMBS][NUMLIMBS];

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


bool getLimbOccludes(unsigned int jt, unsigned int jt2){
	return limbOccludes[jt][jt2];
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
	limbWeights[CHEST][UPPERLEG_LEFT]=0;
	limbWeights[CHEST][UPPERLEG_RIGHT]=0;
	limbWeights[CHEST][LOWERLEG_LEFT]=0;
	limbWeights[CHEST][LOWERLEG_RIGHT]=0;

	limbWeights[ABS][HEAD]=0;
	limbWeights[ABS][UPPERARM_LEFT]=0;
	limbWeights[ABS][UPPERARM_RIGHT]=0;
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


	
	limbOccludes[HEAD][HEAD]=true;
	limbOccludes[HEAD][UPPERARM_LEFT]=true;
	limbOccludes[HEAD][UPPERARM_RIGHT]=true;
	limbOccludes[HEAD][LOWERARM_LEFT]=true;
	limbOccludes[HEAD][LOWERARM_RIGHT]=true;
	limbOccludes[HEAD][CHEST]=true;
	limbOccludes[HEAD][ABS]=true;
	limbOccludes[HEAD][UPPERLEG_LEFT]=true;
	limbOccludes[HEAD][UPPERLEG_RIGHT]=true;
	limbOccludes[HEAD][LOWERLEG_LEFT]=true;
	limbOccludes[HEAD][LOWERLEG_RIGHT]=true;

	limbOccludes[UPPERARM_LEFT][HEAD]=true;
	limbOccludes[UPPERARM_LEFT][UPPERARM_LEFT]=true;
	limbOccludes[UPPERARM_LEFT][UPPERARM_RIGHT]=true;
	limbOccludes[UPPERARM_LEFT][LOWERARM_LEFT]=false;
	limbOccludes[UPPERARM_LEFT][LOWERARM_RIGHT]=true;
	limbOccludes[UPPERARM_LEFT][CHEST]=true;
	limbOccludes[UPPERARM_LEFT][ABS]=true;
	limbOccludes[UPPERARM_LEFT][UPPERLEG_LEFT]=true;
	limbOccludes[UPPERARM_LEFT][UPPERLEG_RIGHT]=true;
	limbOccludes[UPPERARM_LEFT][LOWERLEG_LEFT]=true;
	limbOccludes[UPPERARM_LEFT][LOWERLEG_RIGHT]=true;

	limbOccludes[UPPERARM_RIGHT][HEAD]=true;
	limbOccludes[UPPERARM_RIGHT][UPPERARM_LEFT]=true;
	limbOccludes[UPPERARM_RIGHT][UPPERARM_RIGHT]=true;
	limbOccludes[UPPERARM_RIGHT][LOWERARM_LEFT]=true;
	limbOccludes[UPPERARM_RIGHT][LOWERARM_RIGHT]=false;
	limbOccludes[UPPERARM_RIGHT][CHEST]=true;
	limbOccludes[UPPERARM_RIGHT][ABS]=true;
	limbOccludes[UPPERARM_RIGHT][UPPERLEG_LEFT]=true;
	limbOccludes[UPPERARM_RIGHT][UPPERLEG_RIGHT]=true;
	limbOccludes[UPPERARM_RIGHT][LOWERLEG_LEFT]=true;
	limbOccludes[UPPERARM_RIGHT][LOWERLEG_RIGHT]=true;

	limbOccludes[LOWERARM_LEFT][HEAD]=true;
	limbOccludes[LOWERARM_LEFT][UPPERARM_LEFT]=false;
	limbOccludes[LOWERARM_LEFT][UPPERARM_RIGHT]=true;
	limbOccludes[LOWERARM_LEFT][LOWERARM_LEFT]=true;
	limbOccludes[LOWERARM_LEFT][LOWERARM_RIGHT]=true;
	limbOccludes[LOWERARM_LEFT][CHEST]=true;
	limbOccludes[LOWERARM_LEFT][ABS]=true;
	limbOccludes[LOWERARM_LEFT][UPPERLEG_LEFT]=true;
	limbOccludes[LOWERARM_LEFT][UPPERLEG_RIGHT]=true;
	limbOccludes[LOWERARM_LEFT][LOWERLEG_LEFT]=true;
	limbOccludes[LOWERARM_LEFT][LOWERLEG_RIGHT]=true;

	limbOccludes[LOWERARM_RIGHT][HEAD]=true;
	limbOccludes[LOWERARM_RIGHT][UPPERARM_LEFT]=true;
	limbOccludes[LOWERARM_RIGHT][UPPERARM_RIGHT]=false;
	limbOccludes[LOWERARM_RIGHT][LOWERARM_LEFT]=true;
	limbOccludes[LOWERARM_RIGHT][LOWERARM_RIGHT]=true;
	limbOccludes[LOWERARM_RIGHT][CHEST]=true;
	limbOccludes[LOWERARM_RIGHT][ABS]=true;
	limbOccludes[LOWERARM_RIGHT][UPPERLEG_LEFT]=true;
	limbOccludes[LOWERARM_RIGHT][UPPERLEG_RIGHT]=true;
	limbOccludes[LOWERARM_RIGHT][LOWERLEG_LEFT]=true;
	limbOccludes[LOWERARM_RIGHT][LOWERLEG_RIGHT]=true;

	limbOccludes[CHEST][HEAD]=false;
	limbOccludes[CHEST][UPPERARM_LEFT]=false;
	limbOccludes[CHEST][UPPERARM_RIGHT]=false;
	limbOccludes[CHEST][LOWERARM_LEFT]=true;
	limbOccludes[CHEST][LOWERARM_RIGHT]=true;
	limbOccludes[CHEST][CHEST]=true;
	limbOccludes[CHEST][ABS]=false;
	limbOccludes[CHEST][UPPERLEG_LEFT]=true;
	limbOccludes[CHEST][UPPERLEG_RIGHT]=true;
	limbOccludes[CHEST][LOWERLEG_LEFT]=true;
	limbOccludes[CHEST][LOWERLEG_RIGHT]=true;

	limbOccludes[ABS][HEAD]=true;
	limbOccludes[ABS][UPPERARM_LEFT]=true;
	limbOccludes[ABS][UPPERARM_RIGHT]=true;
	limbOccludes[ABS][LOWERARM_LEFT]=true;
	limbOccludes[ABS][LOWERARM_RIGHT]=true;
	limbOccludes[ABS][CHEST]=false;
	limbOccludes[ABS][ABS]=true;
	limbOccludes[ABS][UPPERLEG_LEFT]=false;
	limbOccludes[ABS][UPPERLEG_RIGHT]=false;
	limbOccludes[ABS][LOWERLEG_LEFT]=true;
	limbOccludes[ABS][LOWERLEG_RIGHT]=true;

	limbOccludes[UPPERLEG_LEFT][HEAD]=true;
	limbOccludes[UPPERLEG_LEFT][UPPERARM_LEFT]=true;
	limbOccludes[UPPERLEG_LEFT][UPPERARM_RIGHT]=true;
	limbOccludes[UPPERLEG_LEFT][LOWERARM_LEFT]=true;
	limbOccludes[UPPERLEG_LEFT][LOWERARM_RIGHT]=true;
	limbOccludes[UPPERLEG_LEFT][CHEST]=true;
	limbOccludes[UPPERLEG_LEFT][ABS]=true;
	limbOccludes[UPPERLEG_LEFT][UPPERLEG_LEFT]=true;
	limbOccludes[UPPERLEG_LEFT][UPPERLEG_RIGHT]=true;
	limbOccludes[UPPERLEG_LEFT][LOWERLEG_LEFT]=false;
	limbOccludes[UPPERLEG_LEFT][LOWERLEG_RIGHT]=true;

	limbOccludes[UPPERLEG_RIGHT][HEAD]=true;
	limbOccludes[UPPERLEG_RIGHT][UPPERARM_LEFT]=true;
	limbOccludes[UPPERLEG_RIGHT][UPPERARM_RIGHT]=true;
	limbOccludes[UPPERLEG_RIGHT][LOWERARM_LEFT]=true;
	limbOccludes[UPPERLEG_RIGHT][LOWERARM_RIGHT]=true;
	limbOccludes[UPPERLEG_RIGHT][CHEST]=true;
	limbOccludes[UPPERLEG_RIGHT][ABS]=true;
	limbOccludes[UPPERLEG_RIGHT][UPPERLEG_LEFT]=true;
	limbOccludes[UPPERLEG_RIGHT][UPPERLEG_RIGHT]=true;
	limbOccludes[UPPERLEG_RIGHT][LOWERLEG_LEFT]=true;
	limbOccludes[UPPERLEG_RIGHT][LOWERLEG_RIGHT]=false;

	limbOccludes[LOWERLEG_LEFT][HEAD]=true;
	limbOccludes[LOWERLEG_LEFT][UPPERARM_LEFT]=true;
	limbOccludes[LOWERLEG_LEFT][UPPERARM_RIGHT]=true;
	limbOccludes[LOWERLEG_LEFT][LOWERARM_LEFT]=true;
	limbOccludes[LOWERLEG_LEFT][LOWERARM_RIGHT]=true;
	limbOccludes[LOWERLEG_LEFT][CHEST]=true;
	limbOccludes[LOWERLEG_LEFT][ABS]=true;
	limbOccludes[LOWERLEG_LEFT][UPPERLEG_LEFT]=false;
	limbOccludes[LOWERLEG_LEFT][UPPERLEG_RIGHT]=true;
	limbOccludes[LOWERLEG_LEFT][LOWERLEG_LEFT]=true;
	limbOccludes[LOWERLEG_LEFT][LOWERLEG_RIGHT]=true;

	limbOccludes[LOWERLEG_RIGHT][HEAD]=true;
	limbOccludes[LOWERLEG_RIGHT][UPPERARM_LEFT]=true;
	limbOccludes[LOWERLEG_RIGHT][UPPERARM_RIGHT]=true;
	limbOccludes[LOWERLEG_RIGHT][LOWERARM_LEFT]=true;
	limbOccludes[LOWERLEG_RIGHT][LOWERARM_RIGHT]=true;
	limbOccludes[LOWERLEG_RIGHT][CHEST]=true;
	limbOccludes[LOWERLEG_RIGHT][ABS]=true;
	limbOccludes[LOWERLEG_RIGHT][UPPERLEG_LEFT]=true;
	limbOccludes[LOWERLEG_RIGHT][UPPERLEG_RIGHT]=false;
	limbOccludes[LOWERLEG_RIGHT][LOWERLEG_LEFT]=true;
	limbOccludes[LOWERLEG_RIGHT][LOWERLEG_RIGHT]=true;


	defInit = true;
}

void CroppedCvMat::clear(){
	mat = cv::Mat();
	offset = cv::Point();
}