#include "KinectManager.h"

class OpenNIManager : public KinectManager{
	
public:
	bool doCalib();
	bool init();
	cv::Mat getColorFrame();
	CroppedCvMat getPlayerColorFrame();
	cv::Mat getDepthFrame();
	Skeleton getSkeleton();
	cv::Mat getUserColorFrame();
	bool skeletonIsGood();
	float getSkeletonGoodness(Skeleton * s);

	int getCenterJoint();
	int getHeadJoint();
	void initMapping(Mapping * mapping);

	cv::Vec3f calculateFacing(Skeleton * s);
	bool checkTracked(int state);
	int initSkeletonScore(Skeleton kinectPoints);

	//save/load device
	void saveParams(std::string);
	void loadParams(std::string);

	//adds new joint to OpenNI skeletons
	void augmentSkeleton(Skeleton * s);
	void getKinectData_depth_raw(DepthXY * depthPoints);

	cv::Vec3f mapDepthToSkeletonPoint(DepthXY d);

	//temp function for approximating facing. s=1: shoulders, s=2:hips
	std::pair<int, int> facingHelper(int s);
};