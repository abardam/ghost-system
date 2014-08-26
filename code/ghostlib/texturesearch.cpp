#include "texturesearch.h"

#include "definitions.h"
#include "KinectManager.h"
#include "cvutil.h"
#include "Limbrary.h"


cv::Mat normalizeSkeleton(cv::Mat skel){
	cv::Mat retval(3,NUMJOINTS,cv::DataType<float>::type);
	float * retptr = retval.ptr<float>();

	for(int i=0; i<NUMJOINTS; ++i){
		cv::Mat tempcol;
		//cv::normalize(skel.col(i) - skel.col(KINECT::getCenterJoint()), tempcol);
		tempcol = skel.col(i) - skel.col(KINECT::getCenterJoint());
		float * colptr = tempcol.ptr<float>();
		for(int j=0;j<3;++j){
			//retval.at<float>(j, i) = tempcol.at<float>(j);
			*(retptr+j*NUMJOINTS+i) = *colptr;
			++colptr;
		}
	}

	////rotate s.t. head joint is directly above neck joint
	//
	////first convert head joint into 2D by projecting onto viewplane
	//cv::Vec2f hj2(retval.at<float>(0,KINECT::getHeadJoint()) / retval.at<float>(2,KINECT::getHeadJoint()),
	//	retval.at<float>(1,KINECT::getHeadJoint()) / retval.at<float>(2,KINECT::getHeadJoint()));
	//
	////then get rotation to that vector...
	//cv::Mat rot = getRotationMatrix(hj2);
	//
	////invert
	//cv::Mat rot_i = rot.inv();
	//
	////convert to 3D transform
	//cv::Mat rot3 = cv::Mat::eye(3,3,cv::DataType<float>::type);
	//
	//rot_i.copyTo(rot3.rowRange(0,2).colRange(0,2));
	//
	////apply
	//for(int i=0;i<NUMJOINTS;++i){
	//	retval.col(i) = rot3 * retval.col(i);
	//}

	return retval;
}

bool ScoreSort(const std::pair<int, float>& lhs, std::pair<int, float>& rhs) 
{ 
  return lhs.second < rhs.second; 
} 

ScoreList sortFrames(Skeleton s, const std::vector<SkeleVideoFrame>& vidRecord, const Limbrary& limbrary, unsigned int limbid, int limit, bool sort, int weightType){
	
	float bestScore = -1;
	cv::Mat b = normalizeSkeleton(s.points);

	ScoreList scoreArray;

	bool useCluster = true;
	int frames = limbrary.getAvailableFramesForLimb(limbid).size();
	if(frames == 0){
		frames = vidRecord.size();
		useCluster = false;
	}

	for(int _i=0; _i<frames; ++_i){

		int i;
		if(useCluster)
			i=limbrary.getAvailableFramesForLimb(limbid)[_i];
		else
			i = _i;

		//if(!(*vidRecord)[i].allPartsIn) continue;
		//if((*vidRecord)[i].videoFrame.mat.empty()) continue;

		float bst=0;
		cv::Mat a = ((vidRecord)[i].kinectPoints2P);
		//for(int j=0; j<NUMJOINTS; ++j){
		//	cv::Mat tmp = b.col(j)-a.col(j);
		//	for(int k=0;k<3;++k){
		//		float tmp2 = tmp.at<float>(k)*tmp.at<float>(k);
		//		bst += tmp2 * weights.at<float>(j);
		//	}
		//}

		//for(int j=0; j<NUMLIMBS; ++j){
		//	if(getLimbWeight(limbid, j) == 1){
		//		int f = getLimbmap()[j].first;
		//		int s = getLimbmap()[j].second;
		//
		//		cv::Mat tmp = (b.col(s)-b.col(f)) - (a.col(s)-a.col(f));
		//		for(int k=0;k<3;++k){
		//			float tmp2 = tmp.at<float>(k)*tmp.at<float>(k);
		//			bst += tmp2;
		//		}
		//	}
		//}
		cv::Mat skelmat_a;
		cv::Mat skelmat_b;

		switch(weightType){
		case GH_WT_NONE:
			
			skelmat_a = jointbasedSkeletonMatrix(a, NUMLIMBS);
			skelmat_b = jointbasedSkeletonMatrix(b, NUMLIMBS);
			break;
		case GH_WT_JOINT:

			skelmat_a = jointbasedSkeletonMatrix(a, limbid);
			skelmat_b = jointbasedSkeletonMatrix(b, limbid);
			break;
		case GH_WT_LIMB:
			
			skelmat_a = limbbasedSkeletonMatrix(a, limbid);
			skelmat_b = limbbasedSkeletonMatrix(b, limbid);
			break;
		}

		bst += sqrSum(skelmat_a - skelmat_b);

		scoreArray.push_back(std::pair<int,float>(i, bst));
	}
	
	if(sort) scoreArray.sort(ScoreSort);

	if(limit != -1 && scoreArray.size() > limit){
		auto it = scoreArray.begin();
		std::advance(it, limit);
		scoreArray.erase(it, scoreArray.end());
	}
	return scoreArray;
}

cv::Mat limbbasedSkeletonMatrix(cv::Mat skelMat, int limbid){
	cv::Mat ret(3,NUMLIMBS,CV_32F,cv::Scalar(0));
	for(int i=0;i<NUMLIMBS;++i){
		if(getLimbWeight(limbid, i) == 1){
			int f = getLimbmap()[i].first;
			int s = getLimbmap()[i].second;

			cv::Mat tmp = (skelMat.col(s)-skelMat.col(f));
			for(int j=0;j<3;++j){
				ret.at<float>(j,i) = tmp.at<float>(j);
			}
		}
	}
	return ret;
}

cv::Mat jointbasedSkeletonMatrix(cv::Mat skelMat, int limbid){
	float * weights = getPartWeights(limbid);
	cv::Mat ret(3,NUMJOINTS,CV_32F);
	for(int j=0;j<3;++j){
		float * retptr = ret.ptr<float>(j);
		float * skelptr = skelMat.ptr<float>(j);
		for(int i=0;i<NUMJOINTS;++i){
			float wt = weights!=0?weights[i]:1;
			
			*(retptr+i) = *(skelptr+i) * wt;
		}
	}
	return ret;
}