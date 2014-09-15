#include "KinectManager.h"
#include "KinectManager_Kinect2.h"
#include <opencv2\opencv.hpp>
#include "definitions.h"
#include "ghostcam.h"
#include "cvutil.h"

#define JFS(s, j) mat_to_vec3(s.points.col(j))
#define JFM(m, j) mat_to_vec3(m.col(j))

#include "Kinect2Starter.h"

namespace KINECT{

	bool bInit = false;
	bool bSkeletonIsGood = false;
	bool bAutoUpdate = false;

	bool KinectManagerKinect2::doCalib(){
		return true;
	}

	bool KinectManagerKinect2::init(){
		if(bInit) return true;
		bInit = SUCCEEDED(InitializeDefaultSensor());
		if (bInit){
			InitKinect2Starter();
		}
		return bInit;
	}

	bool KinectManagerKinect2::release(){
		DestroyKinect2Starter();
		return true;
	}

	void KinectManagerKinect2::updateFrames(){
		UpdateColor();
		UpdateDepth();
		UpdateBody();
		UpdateBodyFrameIndex();
	}

	cv::Mat KinectManagerKinect2::getColorFrame(){
		if(bAutoUpdate) UpdateColor();

		if (getColorHeight() == 0 || getColorWidth() == 0) return cv::Mat();
		cv::Mat colorFrame_ = cv::Mat(getColorHeight(), getColorWidth(), CV_8UC4, GetColorRGBX()).clone();
		cv::Mat colorFrame;
		cv::resize(colorFrame_, colorFrame, cv::Size(CAPTURE_SIZE_X, CAPTURE_SIZE_Y));
		return colorFrame;
	}

	CroppedCvMat KinectManagerKinect2::getPlayerColorFrame(){
		if(bAutoUpdate){
			UpdateColor();
			UpdateDepth();
			UpdateBody();
			UpdateBodyFrameIndex();
		}

		if (getDepthHeight() == 0 || getDepthWidth() == 0 || getColorHeight() == 0 || getColorWidth() == 0 || !skeletonIsGood()) return CroppedCvMat();

		cv::Mat bodyFrame__ = cv::Mat(getColorHeight(), getColorWidth(), CV_8UC4, GetBodyColorRGBX()).clone();
		cv::Mat bodyFrame_;

		cv::resize(bodyFrame__, bodyFrame_, cv::Size(CAPTURE_SIZE_X, CAPTURE_SIZE_Y));

		int minY = bodyFrame_.rows, minX = bodyFrame_.cols, maxY = 0, maxX = 0;

		for (int y = 0; y < bodyFrame_.rows; ++y){
			for (int x = 0; x < bodyFrame_.cols; ++x){
				if (bodyFrame_.ptr<cv::Vec4b>(y)[x](3) != 0){
					if (minY>y)minY = y;
					if (minX>x)minX = x;
					if (maxY < y)maxY = y;
					if (maxX < x)maxX = x;
				}
			}
		}

		cv::Mat bodyFrame = bodyFrame_(cv::Rect(minX, minY, maxX - minX, maxY - minY));

		CroppedCvMat croppedCvMat;
		croppedCvMat.mat = bodyFrame.clone();
		croppedCvMat.offset.x = minX;
		croppedCvMat.offset.y = minY;
		croppedCvMat.origWidth = bodyFrame_.cols;
		croppedCvMat.origHeight = bodyFrame_.rows;

		return croppedCvMat;
	}

	cv::Mat KinectManagerKinect2::getDepthFrame(){
		if(bAutoUpdate) UpdateDepth();

		if (getDepthHeight() == 0 || getDepthWidth() == 0 || getColorHeight() == 0 || getColorWidth() == 0) return cv::Mat();
		cv::Mat depthFrame_ = cv::Mat(getColorHeight(), getColorWidth(), CV_16U, GetDepthMappedToColor()).clone();
		cv::Mat depthFrame;
		cv::resize(depthFrame_, depthFrame, cv::Size(CAPTURE_SIZE_X, CAPTURE_SIZE_Y));
		return depthFrame;
	}

	Skeleton KinectManagerKinect2::getSkeleton(){

		Skeleton skeleton;

		UpdateBody();

		Joint * joints = GetJoints();

		float trackingStateTable[3];
		trackingStateTable[TrackingState_Inferred] = 0.5;
		trackingStateTable[TrackingState_NotTracked] = 0;
		trackingStateTable[TrackingState_Tracked] = 1;

		for(int j=0;j<NUMJOINTS;++j){
			Joint joint = joints[j];
			skeleton.points.ptr<float>(0)[joint.JointType] = joint.Position.X;
			skeleton.points.ptr<float>(1)[joint.JointType] = joint.Position.Y;
			skeleton.points.ptr<float>(2)[joint.JointType] = joint.Position.Z;
			skeleton.points.ptr<float>(3)[joint.JointType] = 1;

			skeleton.states[joint.JointType] = trackingStateTable[joint.TrackingState];
		}

		return skeleton;
	}

	bool KinectManagerKinect2::skeletonIsGood(){
		return getSkeletonIsGood();
	}

	float KinectManagerKinect2::getSkeletonGoodness(Skeleton * s){
		return 0;
	}

	int KinectManagerKinect2::getCenterJoint(){
		return JointType_SpineMid;
	}

	int KinectManagerKinect2::getHeadJoint(){
		return JointType_Head;
	}

	void KinectManagerKinect2::initMapping(Mapping * mapping){
		mapping->limbmap[HEAD]				= lmap(JointType_Head,				JointType_SpineShoulder)			;
		mapping->limbmap[UPPERARM_LEFT]		= lmap(JointType_ShoulderLeft,		JointType_ElbowLeft)		;
		mapping->limbmap[UPPERARM_RIGHT]	= lmap(JointType_ShoulderRight,		JointType_ElbowRight)	;
		mapping->limbmap[LOWERARM_LEFT]		= lmap(JointType_ElbowLeft,			JointType_WristLeft)		;
		mapping->limbmap[LOWERARM_RIGHT]	= lmap(JointType_ElbowRight,			JointType_WristRight)		;
		mapping->limbmap[CHEST]				= lmap(JointType_SpineShoulder,				JointType_SpineMid)			;
		mapping->limbmap[ABS]				= lmap(JointType_SpineBase,				JointType_SpineMid)			;
		mapping->limbmap[UPPERLEG_LEFT]		= lmap(JointType_HipLeft,			JointType_KneeLeft)		;
		mapping->limbmap[UPPERLEG_RIGHT]	= lmap(JointType_HipRight,			JointType_KneeRight)		;
		mapping->limbmap[LOWERLEG_LEFT]		= lmap(JointType_KneeLeft,			JointType_AnkleLeft)		;
		mapping->limbmap[LOWERLEG_RIGHT]	= lmap(JointType_KneeRight,			JointType_AnkleRight)		;

		mapping->jointmap[JointType_Head].push_back(HEAD);
		mapping->jointmap[JointType_SpineShoulder].push_back(HEAD);
		mapping->jointmap[JointType_SpineShoulder].push_back(CHEST);
		mapping->jointmap[JointType_ShoulderLeft].push_back(UPPERARM_LEFT);
		mapping->jointmap[JointType_ShoulderRight].push_back(UPPERARM_RIGHT);
		mapping->jointmap[JointType_ElbowLeft].push_back(LOWERARM_LEFT);
		mapping->jointmap[JointType_ElbowLeft].push_back(UPPERARM_LEFT);
		mapping->jointmap[JointType_ElbowRight].push_back(LOWERARM_RIGHT);
		mapping->jointmap[JointType_ElbowRight].push_back(UPPERARM_RIGHT);
		mapping->jointmap[JointType_WristLeft].push_back(LOWERARM_LEFT);
		mapping->jointmap[JointType_WristRight].push_back(LOWERARM_RIGHT);
		mapping->jointmap[JointType_SpineMid].push_back(CHEST);
		mapping->jointmap[JointType_SpineMid].push_back(ABS);
		mapping->jointmap[JointType_SpineBase].push_back(ABS);
		mapping->jointmap[JointType_HipLeft].push_back(UPPERLEG_LEFT);
		mapping->jointmap[JointType_HipRight].push_back(UPPERLEG_RIGHT);
		mapping->jointmap[JointType_KneeLeft].push_back(LOWERLEG_LEFT);
		mapping->jointmap[JointType_KneeLeft].push_back(UPPERLEG_LEFT);
		mapping->jointmap[JointType_KneeRight].push_back(LOWERLEG_RIGHT);
		mapping->jointmap[JointType_KneeRight].push_back(UPPERLEG_RIGHT);
		mapping->jointmap[JointType_AnkleLeft].push_back(LOWERLEG_LEFT);
		mapping->jointmap[JointType_AnkleRight].push_back(LOWERLEG_RIGHT);

		
		for(int i=0; i<NUMLIMBS; ++i){
			mapping->partWeights[i][getLimbmap()[i].first] = 1;
			mapping->partWeights[i][getLimbmap()[i].second] = 1;
		}

		mapping->partWeights[HEAD][JointType_ShoulderLeft] = 1;
		mapping->partWeights[HEAD][JointType_ShoulderRight] = 1;

		mapping->partWeights[UPPERARM_LEFT]	[JointType_WristLeft] = 1;
		mapping->partWeights[UPPERARM_RIGHT][JointType_WristRight] = 1;
		mapping->partWeights[LOWERARM_LEFT]	[JointType_ShoulderLeft] = 1;
		mapping->partWeights[LOWERARM_RIGHT][JointType_ShoulderRight] = 1;
		

		mapping->partWeights[CHEST][JointType_ShoulderLeft] = 1;
		mapping->partWeights[CHEST][JointType_ShoulderRight] = 1;
		mapping->partWeights[CHEST][JointType_HipLeft] = 1;
		mapping->partWeights[CHEST][JointType_SpineBase] = 1;
		mapping->partWeights[CHEST][JointType_HipRight] = 1;

		mapping->partWeights[ABS][JointType_ShoulderLeft] = 1;
		mapping->partWeights[ABS][JointType_SpineShoulder] = 1;
		mapping->partWeights[ABS][JointType_ShoulderRight] = 1;
		mapping->partWeights[ABS][JointType_HipLeft] = 1;
		mapping->partWeights[ABS][JointType_HipRight] = 1;
		mapping->partWeights[ABS][JointType_SpineShoulder] = 1;

	}


	
	cv::Vec3f KinectManagerKinect2::calculateFacing(Skeleton * _s){
		
		Skeleton s = *_s;

		//calculate torso normalized vector
		cv::Vec3f tnorm = cv::normalize(JFS(s,JointType_SpineShoulder) - JFS(s,JointType_SpineMid));

		//vector from center shoulder to right shoulder
		cv::Vec3f v = JFS(s,JointType_ShoulderRight) - JFS(s,JointType_SpineShoulder);

		//dot product with torso norm vector
		cv::Vec3f dotp = tnorm * (v.dot(tnorm));

		//subtract from right shoulder
		cv::Vec3f projplane = JFS(s,JointType_ShoulderRight) - dotp;
		

		//repeat for left
		cv::Vec3f vl = JFS(s,JointType_ShoulderLeft) - JFS(s,JointType_SpineShoulder);
		cv::Vec3f dotpl = tnorm * (vl.dot(tnorm));
		cv::Vec3f projplanel = JFS(s,JointType_ShoulderLeft) - dotpl;
		

		//find angle bisectors
		cv::Vec3f angr = cv::normalize(projplane - JFS(s,JointType_SpineShoulder));
		cv::Vec3f angl = cv::normalize(projplanel - JFS(s,JointType_SpineShoulder));
		   
		cv::Vec3f bis = cv::normalize(angr + angl);

		return bis;
	}

	bool KinectManagerKinect2::checkTracked(int state){
		//TODO
		return true;
	}

	int KinectManagerKinect2::initSkeletonScore(Skeleton kinectPoints){
		//return a score of how good this skeleton is for building from

		cv::Vec3f torsoV = mat_to_vec3(kinectPoints.points.col(JointType_SpineBase) - kinectPoints.points.col(JointType_SpineShoulder) );
		cv::Vec3f larmV = mat_to_vec3(kinectPoints.points.col(JointType_WristLeft) - kinectPoints.points.col(JointType_ShoulderLeft));
		cv::Vec3f rarmV = mat_to_vec3(kinectPoints.points.col(JointType_WristRight) - kinectPoints.points.col(JointType_ShoulderRight));

		double torsoVA = atan2(torsoV[1], torsoV[0]);
		double larmVA = atan2(larmV[1], larmV[0]);
		double rarmVA = atan2(rarmV[1], rarmV[0]);

		//arms must be away from the body: the further the better
	
		//cv::Vec3f llegV = kinectPoints[nite::JOINT_LEFT_FOOT] - kinectPoints[nite::JOINT_LEFT_HIP];
		//cv::Vec3f rlegV = kinectPoints[nite::JOINT_RIGHT_FOOT] - kinectPoints[nite::JOINT_RIGHT_HIP];

		//double llegVA = atan2(llegV[1], llegV[0]);
		//double rlegVA = atan2(rlegV[1], rlegV[0]);
		
		//double cleg = ((llegVA > rlegVA)?-1000:1000);

		//convert to 2D
		cv::Vec2f a1 = toScreen(mat_to_vec3(kinectPoints.points.col(JointType_AnkleLeft)));
		cv::Vec2f b1 = toScreen(mat_to_vec3(kinectPoints.points.col(JointType_HipLeft)));
		cv::Vec2f a2 = toScreen(mat_to_vec3(kinectPoints.points.col(JointType_AnkleRight)));
		cv::Vec2f b2 = toScreen(mat_to_vec3(kinectPoints.points.col(JointType_HipRight)));

		float lambda1, lambda2;

		//find intersection
		bool int_exist = calculateIntersection(a1, b1, a2, b2, &lambda1, &lambda2);

		double cleg = int_exist?((lambda1>0&&lambda1<1&&lambda2>0&&lambda2<1)?-1000:1000):1000;

		//legs should not be crossed

		//better if facing screen
		//i.e. arms parallel to screen

		cv::Vec3f z_in(1,0,0);

		double multL = abs( ((cv::normalize(larmV)).dot( z_in)) );
		double multR = abs( ((cv::normalize(rarmV)).dot( z_in)));

		std::cout << "torso: " << torsoVA << " larm: " << larmVA << " rarm: " << rarmVA << " cleg: " << cleg << " multL: " << multL << " multR: " << multR << std::endl;

		//int score = multL * 1000 * (torsoVA - larmVA) + multR * 1000 * (rarmVA - torsoVA) + cleg;
		int score = multL * 1000 + multR * 1000 + cleg;

		int s2 = score > 200000? 200000: score;

		return getSkeletonGoodness(&kinectPoints) * s2;
	}

	void KinectManagerKinect2::augmentSkeleton(Skeleton * s){
		//nothing to do here
	}

	void KinectManagerKinect2::saveParams(std::string filename){
		//nothing to do here
	}

	void KinectManagerKinect2::loadParams(std::string filename){
		//nothing to do here
	}

	//delete later

	void KinectManagerKinect2::getKinectData_depth_raw(DepthXY * depthPoints){
		if(!init()) { 
			std::cerr << "Error! not initialized!\n";  
			return;
		}
		
		if(getDepthHeight() == 0 || getDepthWidth() == 0) return;

		cv::Mat depthFrame_ = cv::Mat(getDepthHeight(), getDepthWidth(), CV_16U, GetDepth());
		cv::Mat depthFrame;
		cv::resize(depthFrame_, depthFrame, cv::Size(CAPTURE_SIZE_X, CAPTURE_SIZE_Y));

		for(int y=0;y<CAPTURE_SIZE_Y;++y){
			for(int x=0;x<CAPTURE_SIZE_X;++x){
				depthPoints[x + CAPTURE_SIZE_X*y].depth = depthFrame.ptr<unsigned short>()[x + CAPTURE_SIZE_X*y];
				depthPoints[x + CAPTURE_SIZE_X*y].x = x;
				depthPoints[x + CAPTURE_SIZE_X*y].y = y;
			}
		}
	}

	cv::Vec3f KinectManagerKinect2::mapDepthToSkeletonPoint(DepthXY d){
		cv::Vec3f ret;

		float dx = d.x;
		float dy = d.y;
		long dz = d.depth;

		mapDepthToSkeleton(&dx, &dy, &dz, &ret(0), &ret(1), &ret(2));

		return ret;
	}

	
	//takes 4xN Mat of camera points and uses Kinect function to map it to 2xN Mat of color space points
	cv::Mat KinectManagerKinect2::mapCameraPointsToColorPoints(cv::Mat cameraPoints){
		ICoordinateMapper * coordinateMapper = getCoordinateMapper();
		int nCameraPoints = cameraPoints.cols;
		std::vector<CameraSpacePoint> vCameraPoints(nCameraPoints);
		std::vector<ColorSpacePoint> vColorPoints(nCameraPoints);

		for(int i=0;i<nCameraPoints;++i){
			vCameraPoints[i].X = cameraPoints.ptr<float>(0)[i];
			vCameraPoints[i].Y = cameraPoints.ptr<float>(1)[i];
			vCameraPoints[i].Z = cameraPoints.ptr<float>(2)[i];
		}

		HRESULT hr = coordinateMapper->MapCameraPointsToColorSpace(nCameraPoints, vCameraPoints.data(), nCameraPoints, vColorPoints.data());

		float ratioX = (CAPTURE_SIZE_X + 0.0) / CAPTURE_SIZE_X_COLOR;
		float ratioY = (CAPTURE_SIZE_Y + 0.0) / CAPTURE_SIZE_Y_COLOR;

		cv::Mat mColorPoints(2, nCameraPoints, CV_32F);
		for(int i=0;i<nCameraPoints;++i){
			mColorPoints.ptr<float>(0)[i] = ratioX * vColorPoints[i].X;
			mColorPoints.ptr<float>(1)[i] = ratioY * vColorPoints[i].Y;
		}

		return mColorPoints;
	}

	std::pair<int, int> KinectManagerKinect2::facingHelper(int s){
		if(s==1) //shoulders
		{
			return std::pair<int,int>(JointType_ShoulderLeft, JointType_ShoulderRight);
		}else if(s==2){
			return std::pair<int,int>(JointType_HipLeft, JointType_HipRight);
		}else{
			std::cerr << "wrong value passed into facingHelper\n";
			throw std::exception();
		}
	}
}