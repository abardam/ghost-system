#include "KinectManager.h"
#include <opencv2\opencv.hpp>
#include "definitions.h"
#include "ghostutil.h"
#include "ghostcam.h"

//#if !INIT_KINECT
//#endif

//#include "PTAM2Kinect.h"



#if GHOST_CAPTURE == CAPTURE_OPENNI
//openNI
#include <OpenNI.h>
#include "OpenNIStarter.h"
#include <NiTE.h>
#include "cvutil.h"

#define JFS(s, j) mat_to_vec3(s.points.col(j))
#define JFM(m, j) mat_to_vec3(m.col(j))


namespace KINECT{
#if GHOST_INPUT == INPUT_OPENNI
	bool docalib = true;
#endif
#if GHOST_INPUT == INPUT_VI
	bool docalib = false;
#endif
	bool doCalib(){
		return docalib;
	}

	bool init(){
		return initNI();
	}

	cv::Mat getColorFrame(){
		unsigned char buffer[CAPTURE_SIZE_X * CAPTURE_SIZE_Y * 3];
		getColorData(buffer);
		return cv::Mat(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, cv::DataType<cv::Vec3b>::type, buffer).clone();

		/*cv::Mat mat3 = cv::Mat(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, cv::DataType<cv::Vec3b>::type, buffer).clone();
		cv::Mat chan[4];
		cv::split(mat3, chan);

		cv::Mat mat4;
		chan[3] = cv::Mat(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, cv::DataType<unsigned char>::type, 255);
		cv::merge(chan, 4, mat4);

		return mat4;*/

	}

	CroppedCvMat getPlayerColorFrame(){
		unsigned char buffer[CAPTURE_SIZE_X * CAPTURE_SIZE_Y * 3];

		if(!skeletonIsGood()) return CroppedCvMat();
		
		int offsetX, offsetY, maxX, maxY;
		getPlayerColorData(buffer, &offsetX, &offsetY, &maxX, &maxY);

		CroppedCvMat ccm;
		
		ccm.offset = cv::Point2d(offsetX, offsetY);
		cv::Mat temp = cv::Mat(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, cv::DataType<cv::Vec3b>::type, buffer).clone();

		if(offsetX < maxX && offsetY < maxY)
			ccm.mat = temp(cv::Rect(offsetX, offsetY, maxX-offsetX, maxY-offsetY)).clone();

		return ccm;

		/*cv::Mat mat3 = cv::Mat(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, cv::DataType<cv::Vec3b>::type, buffer).clone();
		cv::Mat chan[4];
		cv::split(mat3, chan);

		cv::Mat mat4;
		chan[3] = cv::Mat(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, cv::DataType<unsigned char>::type, 255);
		cv::merge(chan, 4, mat4);

		return mat4;*/
	}

	cv::Mat getDepthFrame(){
		openni::DepthPixel buffer[CAPTURE_SIZE_X * CAPTURE_SIZE_Y];
		getDepthData(buffer);
		
		return cv::Mat(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, cv::DataType<openni::DepthPixel>::type, buffer).clone();
	}

	Skeleton getSkeleton(){
		nite::Skeleton nSkeleton;
		bool res = getSkeletonData(&nSkeleton);

		Skeleton skeleton;

		if(!res) return skeleton;

		for(int i=0;i<NUMJOINTS;++i){
			//skeleton.points.at<float>(0,i) = nSkeleton.getJoint((nite::JointType)i).getPosition().x/1000.;
			//skeleton.points.at<float>(1,i) = nSkeleton.getJoint((nite::JointType)i).getPosition().y/1000.;
			//skeleton.points.at<float>(2,i) = nSkeleton.getJoint((nite::JointType)i).getPosition().z/1000.;
			//skeleton.points.at<float>(3,i) = 1;

			skeleton.points.ptr<float>()[NUMJOINTS*0+i] = nSkeleton.getJoint((nite::JointType)i).getPosition().x/1000.;
			skeleton.points.ptr<float>()[NUMJOINTS*1+i] = nSkeleton.getJoint((nite::JointType)i).getPosition().y/1000.;
			skeleton.points.ptr<float>()[NUMJOINTS*2+i] = nSkeleton.getJoint((nite::JointType)i).getPosition().z/1000.;
			skeleton.points.ptr<float>()[NUMJOINTS*3+i] = 1;

			skeleton.states[i] = nSkeleton.getJoint((nite::JointType)i).getPositionConfidence();
		}

		return skeleton;
	}

	int getCenterJoint(){
		return nite::JOINT_TORSO;
	}

	int getHeadJoint(){
		return nite::JOINT_HEAD;
	}
	

#if INIT_KINECT
	cv::Vec2f toScreen(cv::Vec3f v){
		cv::Vec2f ret;
		float dpth;
		mapSkeletonToDepth(&v[0], &v[1], &v[2], &ret[0], &ret[1], &dpth);
		return ret;
	}
#endif

	void initMapping(Mapping * mapping){
		mapping->limbmap[HEAD]				= lmap(nite::JOINT_HEAD,				nite::JOINT_NECK)			;
		mapping->limbmap[UPPERARM_LEFT]		= lmap(nite::JOINT_LEFT_SHOULDER,		nite::JOINT_LEFT_ELBOW)		;
		mapping->limbmap[UPPERARM_RIGHT]	= lmap(nite::JOINT_RIGHT_SHOULDER,		nite::JOINT_RIGHT_ELBOW)	;
		mapping->limbmap[LOWERARM_LEFT]		= lmap(nite::JOINT_LEFT_ELBOW,			nite::JOINT_LEFT_HAND)		;
		mapping->limbmap[LOWERARM_RIGHT]	= lmap(nite::JOINT_RIGHT_ELBOW,			nite::JOINT_RIGHT_HAND)		;
		mapping->limbmap[CHEST]				= lmap(nite::JOINT_NECK,				nite::JOINT_TORSO)			;
		mapping->limbmap[ABS]				= lmap(JOINT_CENTER_HIP,				nite::JOINT_TORSO)			;
		mapping->limbmap[UPPERLEG_LEFT]		= lmap(nite::JOINT_LEFT_HIP,			nite::JOINT_LEFT_KNEE)		;
		mapping->limbmap[UPPERLEG_RIGHT]	= lmap(nite::JOINT_RIGHT_HIP,			nite::JOINT_RIGHT_KNEE)		;
		mapping->limbmap[LOWERLEG_LEFT]		= lmap(nite::JOINT_LEFT_KNEE,			nite::JOINT_LEFT_FOOT)		;
		mapping->limbmap[LOWERLEG_RIGHT]	= lmap(nite::JOINT_RIGHT_KNEE,			nite::JOINT_RIGHT_FOOT)		;

		mapping->jointmap[nite::JOINT_HEAD			].push_back(HEAD);
		mapping->jointmap[nite::JOINT_NECK			].push_back(HEAD);
		mapping->jointmap[nite::JOINT_NECK			].push_back(CHEST);
		mapping->jointmap[nite::JOINT_LEFT_SHOULDER	].push_back(UPPERARM_LEFT);
		mapping->jointmap[nite::JOINT_RIGHT_SHOULDER	].push_back(UPPERARM_RIGHT);
		mapping->jointmap[nite::JOINT_LEFT_ELBOW		].push_back(LOWERARM_LEFT);
		mapping->jointmap[nite::JOINT_LEFT_ELBOW		].push_back(UPPERARM_LEFT);
		mapping->jointmap[nite::JOINT_RIGHT_ELBOW	].push_back(LOWERARM_RIGHT);
		mapping->jointmap[nite::JOINT_RIGHT_ELBOW	].push_back(UPPERARM_RIGHT);
		mapping->jointmap[nite::JOINT_LEFT_HAND		].push_back(LOWERARM_LEFT);
		mapping->jointmap[nite::JOINT_RIGHT_HAND		].push_back(LOWERARM_RIGHT);
		mapping->jointmap[nite::JOINT_TORSO			].push_back(CHEST);
		mapping->jointmap[nite::JOINT_TORSO			].push_back(ABS);
		mapping->jointmap[JOINT_CENTER_HIP		].push_back(ABS);
		mapping->jointmap[nite::JOINT_LEFT_HIP		].push_back(UPPERLEG_LEFT);
		mapping->jointmap[nite::JOINT_RIGHT_HIP		].push_back(UPPERLEG_RIGHT);
		mapping->jointmap[nite::JOINT_LEFT_KNEE		].push_back(LOWERLEG_LEFT);
		mapping->jointmap[nite::JOINT_LEFT_KNEE		].push_back(UPPERLEG_LEFT);
		mapping->jointmap[nite::JOINT_RIGHT_KNEE		].push_back(LOWERLEG_RIGHT);
		mapping->jointmap[nite::JOINT_RIGHT_KNEE		].push_back(UPPERLEG_RIGHT);
		mapping->jointmap[nite::JOINT_LEFT_FOOT		].push_back(LOWERLEG_LEFT);
		mapping->jointmap[nite::JOINT_RIGHT_FOOT		].push_back(LOWERLEG_RIGHT);

		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(HEAD);

		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(UPPERARM_LEFT);
		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(UPPERARM_RIGHT);
		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(LOWERARM_LEFT);
		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(LOWERARM_RIGHT);

		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(CHEST);
		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(ABS);

		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(UPPERLEG_LEFT);
		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(UPPERLEG_RIGHT);
		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(LOWERLEG_LEFT);
		mapping->combineParts.push_back(std::vector<int>());
		mapping->combineParts.back().push_back(LOWERLEG_RIGHT);

		std::fill_n(mapping->combinePartsMap, NUMLIMBS, -1);

		for(int i=0; i<mapping->combineParts.size(); ++i){
			for(int j=0; j<mapping->combineParts[i].size(); ++j){
				mapping->combinePartsMap[mapping->combineParts[i][j]] = mapping->combineParts[i][0];
			}
		}

		
		for(int i=0; i<NUMLIMBS; ++i){
			mapping->partWeights[i][getLimbmap()[i].first] = 1;
			mapping->partWeights[i][getLimbmap()[i].second] = 1;
		}

		mapping->partWeights[HEAD][nite::JOINT_LEFT_SHOULDER] = 1;
		mapping->partWeights[HEAD][nite::JOINT_RIGHT_SHOULDER] = 1;

		mapping->partWeights[UPPERARM_LEFT]	[nite::JOINT_LEFT_HAND] = 1;
		mapping->partWeights[UPPERARM_RIGHT][nite::JOINT_RIGHT_HAND] = 1;
		mapping->partWeights[LOWERARM_LEFT]	[nite::JOINT_LEFT_SHOULDER] = 1;
		mapping->partWeights[LOWERARM_RIGHT][nite::JOINT_RIGHT_SHOULDER] = 1;
		

		mapping->partWeights[CHEST][nite::JOINT_LEFT_SHOULDER] = 1;
		mapping->partWeights[CHEST][nite::JOINT_RIGHT_SHOULDER] = 1;
		mapping->partWeights[CHEST][nite::JOINT_LEFT_HIP] = 1;
		mapping->partWeights[CHEST][JOINT_CENTER_HIP] = 1;
		mapping->partWeights[CHEST][nite::JOINT_RIGHT_HIP] = 1;

		mapping->partWeights[ABS][nite::JOINT_LEFT_SHOULDER] = 1;
		mapping->partWeights[ABS][nite::JOINT_NECK] = 1;
		mapping->partWeights[ABS][nite::JOINT_RIGHT_SHOULDER] = 1;
		mapping->partWeights[ABS][nite::JOINT_LEFT_HIP] = 1;
		mapping->partWeights[ABS][nite::JOINT_RIGHT_HIP] = 1;
	}

	cv::Vec3f calculateFacing(Skeleton * _s){
		
		Skeleton s = *_s;

		//calculate torso normalized vector
		cv::Vec3f tnorm = cv::normalize(JFS(s,nite::JOINT_NECK) - JFS(s,nite::JOINT_TORSO));

		//vector from center shoulder to right shoulder
		cv::Vec3f v = JFS(s,nite::JOINT_RIGHT_SHOULDER) - JFS(s,nite::JOINT_NECK);

		//dot product with torso norm vector
		cv::Vec3f dotp = tnorm * (v.dot(tnorm));

		//subtract from right shoulder
		cv::Vec3f projplane = JFS(s,nite::JOINT_RIGHT_SHOULDER) - dotp;
		

		//repeat for left
		cv::Vec3f vl = JFS(s,nite::JOINT_LEFT_SHOULDER) - JFS(s,nite::JOINT_NECK);
		cv::Vec3f dotpl = tnorm * (vl.dot(tnorm));
		cv::Vec3f projplanel = JFS(s,nite::JOINT_LEFT_SHOULDER) - dotpl;
		

		//find angle bisectors
		cv::Vec3f angr = cv::normalize(projplane - JFS(s,nite::JOINT_NECK));
		cv::Vec3f angl = cv::normalize(projplanel - JFS(s,nite::JOINT_NECK));
		   
		cv::Vec3f bis = cv::normalize(angr + angl);

		return bis;
	}

	bool checkTracked(int state){
		//TODO
		return true;
	}

	bool skeletonIsGood(){
		nite::Skeleton s;
		return getSkeletonData(&s);
	}

	float getSkeletonGoodness(Skeleton * s){
		float mult = s->states[KINECT::getCenterJoint()];
		if(mult <0.3) mult = 0.3;
		return mult;
	}

	int initSkeletonScore(Skeleton kinectPoints){
		//return a score of how good this skeleton is for building from

		cv::Vec3f torsoV = mat_to_vec3(kinectPoints.points.col(JOINT_CENTER_HIP) - kinectPoints.points.col(nite::JOINT_NECK) );
		cv::Vec3f larmV = mat_to_vec3(kinectPoints.points.col(nite::JOINT_LEFT_HAND) - kinectPoints.points.col(nite::JOINT_LEFT_SHOULDER));
		cv::Vec3f rarmV = mat_to_vec3(kinectPoints.points.col(nite::JOINT_RIGHT_HAND) - kinectPoints.points.col(nite::JOINT_RIGHT_SHOULDER));

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
		cv::Vec2f a1 = toScreen(mat_to_vec3(kinectPoints.points.col(nite::JOINT_LEFT_FOOT)));
		cv::Vec2f b1 = toScreen(mat_to_vec3(kinectPoints.points.col(nite::JOINT_LEFT_HIP)));
		cv::Vec2f a2 = toScreen(mat_to_vec3(kinectPoints.points.col(nite::JOINT_RIGHT_FOOT)));
		cv::Vec2f b2 = toScreen(mat_to_vec3(kinectPoints.points.col(nite::JOINT_RIGHT_HIP)));

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

	void augmentSkeleton(Skeleton * s){
		//center hip joint to be added
		cv::Mat extraJoint = (s->points.col(nite::JOINT_LEFT_HIP) + s->points.col(nite::JOINT_RIGHT_HIP))/2;
		for(int i=0;i<4;++i){
			s->points.at<float>(i, JOINT_CENTER_HIP) = extraJoint.at<float>(i);
			
		}

		s->states[JOINT_CENTER_HIP] = (s->states[nite::JOINT_LEFT_HIP]  + s->states[nite::JOINT_RIGHT_HIP])/2;
	}

	void saveParams(std::string filename){
		openni::Recorder recorder;
		recorder.create(filename.c_str());
		recorder.attach(*KINECT::getDepthStream());
		recorder.start();
		recorder.stop();

	}

	void loadParams(std::string filename){

		initFromFile(filename);
	}

	//delete later

	void getKinectData_depth_raw(DepthXY * depthPoints){
		if(!init()) { 
			std::cerr << "Error! not initialized!\n";  
			return;
		}

		openni::DepthPixel buffer[CAPTURE_SIZE_X * CAPTURE_SIZE_Y];
		getDepthData(buffer);

		for(int y=0;y<CAPTURE_SIZE_Y;++y){
			for(int x=0;x<CAPTURE_SIZE_X;++x){
				depthPoints[x + CAPTURE_SIZE_X*y].depth = buffer[x + CAPTURE_SIZE_X*y];
				depthPoints[x + CAPTURE_SIZE_X*y].x = x;
				depthPoints[x + CAPTURE_SIZE_X*y].y = y;
			}
		}
	}

	cv::Vec3f mapDepthToSkeletonPoint(DepthXY d){
		cv::Vec3f ret;

		float dx = d.x;
		float dy = d.y;
		float dz = d.depth;

		mapDepthToSkeleton(&dx, &dy, &dz, &ret(0), &ret(1), &ret(2));

		return ret/1000.;
	}

	std::pair<int, int> facingHelper(int s){
		if(s==1) //shoulders
		{
			return std::pair<int,int>(nite::JOINT_LEFT_SHOULDER, nite::JOINT_RIGHT_SHOULDER);
		}else if(s==2){
			return std::pair<int,int>(nite::JOINT_LEFT_HIP, nite::JOINT_RIGHT_HIP);
		}else{
			std::cerr << "wrong value passed into facingHelper\n";
			throw std::exception();
		}
	}
}


#elif GHOST_CAPTURE == CAPTURE_KINECT2
#include "Kinect2Starter.h"

namespace KINECT{

	bool bInit = false;

	bool doCalib(){
		return false;
	}

	bool init(){
		if(bInit) return true;
		bInit = SUCCEEDED(InitializeDefaultSensor());
		if (bInit){
			InitKinect2Starter();
		}
		return bInit;
	}

	bool release(){
		DestroyKinect2Starter();
		return true;
	}

	cv::Mat getColorFrame(){
		UpdateColor();
		if (getColorHeight() == 0 || getColorWidth() == 0) return cv::Mat();
		cv::Mat colorFrame_ = cv::Mat(getColorHeight(), getColorWidth(), CV_8UC4, GetColorRGBX()).clone();
		//cv::Mat colorFrame;
		//cv::resize(colorFrame_, colorFrame, cv::Size(getColorWidth(), getColorHeight()));
		//return colorFrame;

		return colorFrame_;
	}

	CroppedCvMat getPlayerColorFrame(){
		return CroppedCvMat();
	}

	cv::Mat getDepthFrame(){
		UpdateDepth();
		if (getDepthHeight() == 0 || getDepthWidth() == 0 || getColorHeight() == 0 || getColorWidth() == 0) return cv::Mat();
		cv::Mat depthFrame_ = cv::Mat(getColorHeight(), getColorWidth(), CV_16U, GetDepthMappedToColor()).clone();
		//cv::Mat depthFrame;
		//cv::resize(depthFrame_, depthFrame, cv::Size(getColorWidth(), getColorHeight()));
		//return depthFrame;

		return depthFrame_;
	}

	Skeleton getSkeleton(){

		Skeleton skeleton;
		IBody * body;
		//get body from kinect;


		Joint joints[NUMJOINTS];
		HRESULT hr = body->GetJoints(NUMJOINTS, joints);

		if(!SUCCEEDED(hr)) return skeleton;

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

		return Skeleton();
	}

	cv::Mat getUserColorFrame(){
		return cv::Mat();
	}

	bool skeletonIsGood(){
		return false;
	}

	float getSkeletonGoodness(Skeleton * s){
		return 0;
	}

	int getCenterJoint(){
		return JointType_SpineMid;
	}

	int getHeadJoint(){
		return JointType_Head;
	}

	void initMapping(Mapping * mapping){
		mapping->limbmap[HEAD]				= lmap(JointType_Head,				JointType_Neck)			;
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
		mapping->jointmap[JointType_Neck].push_back(HEAD);
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
		mapping->partWeights[ABS][JointType_Neck] = 1;
		mapping->partWeights[ABS][JointType_ShoulderRight] = 1;
		mapping->partWeights[ABS][JointType_HipLeft] = 1;
		mapping->partWeights[ABS][JointType_HipRight] = 1;
		mapping->partWeights[ABS][JointType_SpineShoulder] = 1;

	}

}

#endif