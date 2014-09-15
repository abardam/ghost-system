#include "KinectManager.h"

namespace KINECT{
	class KinectManagerKinect2
		: public IKinectManager{
		bool doCalib();
		bool init();
		bool release();
		void updateFrames();
		cv::Mat getColorFrame();
		CroppedCvMat getPlayerColorFrame();
		cv::Mat getDepthFrame();
		Skeleton getSkeleton();
		bool skeletonIsGood();
		float getSkeletonGoodness(Skeleton * s);
		int getCenterJoint();
		int getHeadJoint();
		void initMapping(Mapping * mapping);
		cv::Vec3f calculateFacing(Skeleton * s);
		bool checkTracked(int state);
		int initSkeletonScore(Skeleton kinectPoints);
		void saveParams(std::string);
		void loadParams(std::string);
		void augmentSkeleton(Skeleton * s);
		void getKinectData_depth_raw(DepthXY * depthPoints);
		cv::Vec3f mapDepthToSkeletonPoint(DepthXY d);
		cv::Mat mapCameraPointsToColorPoints(cv::Mat cameraPoints);
		std::pair<int, int> facingHelper(int s);
	};
};