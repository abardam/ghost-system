#include "KinectManager.h"
#include "KinectManager_OpenNI.h"
#include "KinectManager_Kinect2.h"
#include <opencv2\opencv.hpp>
#include "definitions.h"
#include "ghostutil.h"
#include "ghostcam.h"
#include "cvutil.h"

//#if !INIT_KINECT
//#endif

//#include "PTAM2Kinect.h"

#define JFS(s, j) mat_to_vec3(s.points.col(j))
#define JFM(m, j) mat_to_vec3(m.col(j))

namespace KINECT{
	IKinectManager * kinectManager;

	bool setKinectManager(int captureType){

		if(kinectManager){
			kinectManager->release();
			delete kinectManager;
		}

		switch(captureType){
		case CAPTURE_OPENNI:
			kinectManager = new KinectManagerOpenNI;
			break;
		case CAPTURE_KINECT:
			break;
		case CAPTURE_KINECT2:
			kinectManager = new KinectManagerKinect2;
			break;
		}

		return true;

	}

	bool init(){
		if(!kinectManager){
			std::cerr << "Set KinectManager type first!\n";
			throw;
		}
		return kinectManager->init();
	}

	bool doCalib(){
		return kinectManager->doCalib();
	}

	bool release(){
		return kinectManager->release();
	}

	void updateFrames(){
		kinectManager->updateFrames();
	}

	cv::Mat getColorFrame(){
		return kinectManager->getColorFrame();
	}

	CroppedCvMat getPlayerColorFrame(){
		return kinectManager->getPlayerColorFrame();
	}

	cv::Mat getDepthFrame(){
		return kinectManager->getDepthFrame();
	}

	Skeleton getSkeleton(){
		return kinectManager->getSkeleton();
	}

	bool skeletonIsGood(){
		return kinectManager->skeletonIsGood();
	}

	float getSkeletonGoodness(Skeleton * s){
		return kinectManager->getSkeletonGoodness(s);
	}

	int getCenterJoint(){
		return kinectManager->getCenterJoint();
	}

	int getHeadJoint(){
		return kinectManager->getHeadJoint();
	}

	void initMapping(Mapping * mapping){
		kinectManager->initMapping(mapping);
	}

	cv::Vec3f calculateFacing(Skeleton * s){
		return kinectManager->calculateFacing(s);
	}

	bool checkTracked(int state){
		return kinectManager->checkTracked(state);
	}

	int initSkeletonScore(Skeleton kinectPoints){
		return kinectManager->initSkeletonScore(kinectPoints);
	}

	//save/load device
	void saveParams(std::string s){
		kinectManager->saveParams(s);
	}

	void loadParams(std::string s){
		kinectManager->loadParams(s);
	}

	//adds new joint to OpenNI skeletons
	void augmentSkeleton(Skeleton * s){
		kinectManager->augmentSkeleton(s);
	}

	void getKinectData_depth_raw(DepthXY * depthPoints){
		kinectManager->getKinectData_depth_raw(depthPoints);
	}

	cv::Vec3f mapDepthToSkeletonPoint(DepthXY d){
		return kinectManager->mapDepthToSkeletonPoint(d);
	}

	//takes 4xN Mat of camera points and uses Kinect function to map it to 2xN Mat of color space points
	cv::Mat mapCameraPointsToColorPoints(cv::Mat cameraPoints){
		return kinectManager->mapCameraPointsToColorPoints(cameraPoints);
	}
	
	//temp function for approximating facing. s=1: shoulders, s=2:hips
	std::pair<int, int> facingHelper(int s){
		return kinectManager->facingHelper(s);
	}
};
