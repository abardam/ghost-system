#include "process.h"
#include "loader.h"
#include "bodybuild.h"
#include "KinectManager.h"
#include "ghostcam.h"

void initAndLoad(cv::Mat mse3CfW, cv::Mat K2P, std::vector<SkeleVideoFrame> * vidRecord, std::vector<Skeleton> * wcSkeletons, std::string path, bool loadRGB){
	initLoader();
	initDefinitions();
	LoadVideo(mse3CfW, K2P, vidRecord, wcSkeletons, path, loadRGB);
	if(vidRecord->empty()) {
		std::cout << "no frames loaded... check file path\n";
		return;
	}
	
	int offset = 0;
	/*for(auto it = vidRecord.begin(); it+offset != vidRecord.end(); ++ it){
		buildDepth((it)->kinectPoints, (it)->skeleton.states, (it+offset)->depthFrame, (it+offset)->videoFrame);
	}*/
	
	//TODO: store camera matrix in the SVF.xml
	//for now use the hardcoded camera matrix
	setCameraMatrix(expGetCameraMatrix());
}


void buildCylinderBody(std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cb){

	cb->setVidRecord(vidRecord);
	cb->validateLimbs();

	int best = -10000;
	int bestIndex;
	SkeleVideoFrame * bestF = NULL;

	for(auto it = vidRecord->begin(); it != vidRecord->end(); ++it){
		if(it->depthFrame.empty() || it->videoFrame.mat.empty()) continue;
		if(it->kinectPoints.initSkeleton == 0){
			//float mult = KINECT::getSkeletonGoodness(&it->skeleton);
			int score = KINECT::initSkeletonScore(it->kinectPoints);
			it->kinectPoints.initSkeleton =  score ;
			std::cout << "score: " << score << std::endl;
		}

		if(best < it->kinectPoints.initSkeleton){

			if(bestF != NULL && !bestF->depthFrame.empty()){
				bestF->depthFrame = cv::Mat();
			}

			best = it->kinectPoints.initSkeleton;
			bestF = &*it;
			bestIndex = it - vidRecord->begin();
		}else{
			it->depthFrame = cv::Mat();
		}
	}

	cv::imwrite("best.png", bestF->videoFrame.mat);
	buildDepth(bestF->kinectPoints, bestF->depthFrame, bestF->videoFrame, cb);

	cb->bestFrame = bestIndex;


	cb->regularizeLimbs();
	cb->calcLimbTransforms();

	//temp debug
	/*
	cv::Mat cylinderPts[NUMLIMBS];
	cv::Mat ident[NUMLIMBS];

	for(int i=0;i<NUMLIMBS;++i){
		cylinderPts[i] = cv::Mat(4,100*100,cv::DataType<float>::type);
		ident[i] = cv::Mat::eye(4,4,cv::DataType<float>::type);
		for(int r=0;r<100;++r){
			for(int c=0;c<100;++c){
				float angle = CV_PI/100*c;
				float ht = 0.01 * r;
				cylinderPts[i].at<float>(0, r*100+c) = cosf(angle);
				cylinderPts[i].at<float>(1, r*100+c) = sinf(angle);
				cylinderPts[i].at<float>(2, r*100+c) = ht;
				cylinderPts[i].at<float>(3, r*100+c) = 1;
			}
		}
	}

	CreateDirectory("shit", NULL);
	char buff[10];

	for(int i=0;i<vidRecord->size();++i){
		cv::Mat t = (*vidRecord)[i].videoFrame.mat.clone();
		cv::Mat t2 = (*vidRecord)[i].videoFrame.mat.clone();
		cb->colorsAtPixels(cylinderPts, ident, i, &t, &t2);

		cv::imwrite("shit/" + std::string(itoa(i, buff, 10)) + ".png", t2);
	}*/

	cb->calcFacings();
	//cb->calcVidRecordBins();

}

void calculateWorldCoordinateSkeletons(cv::Mat K2P, std::vector<SkeleVideoFrame> * vidRecord, std::vector<Skeleton> * outputSkeletons){
	outputSkeletons->resize(vidRecord->size());

	for(int i=0;i<vidRecord->size();++i){
		if(!(*vidRecord)[i].cam2World.empty()){

			(*outputSkeletons)[i] = (*vidRecord)[i].kinectPoints;
			(*outputSkeletons)[i].points = (*vidRecord)[i].cam2World * K2P * (*outputSkeletons)[i].points;
		}
	}
}

