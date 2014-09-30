#if 0

#include <iostream>
#include <vector>
#include <set>

#include "pthread.h"
#include "TooN\TooN.h"
#include "TooN\se3.h"
#include "tinyxml.h"

#include "SkeletonTaker.h"
#include "KinectManager.h"
#include "ghostutil.h"
#include "PTAM2Kinect.h"

#include "loader.h"
#include "util.h"

#include "definitions.h"

using namespace PTAMM;
namespace KINECT{

	SkeletonTaker::SkeletonTaker(){
	}

	void SkeletonTaker::Reset(){
		ptamcalib = false;
		//proc_vidRecord.clear();
		//getVidRecord().clear();
		gridpts.clear();
		gridpts2.clear();
	}

	void SkeletonTaker::calibPTAM(std::vector<PTAMM::MapPoint*> points, TooN::SE3<> mse3CfW){
		
		if(!ptamcalib && doCalib())
		{
			NUI_DEPTH_IMAGE_POINT * _depthdata = new NUI_DEPTH_IMAGE_POINT[640 * 480];
			KINECT::getKinectData_depth_raw(_depthdata);

			std::vector<TooN::Vector<4>> ptampoints;
			for(auto it=points.begin();it!=points.end();++it){
				Vector<3> worldPos = (*it)->v3WorldPos;
				Vector<4> worldPosHom = TooN::makeVector(worldPos[0], worldPos[1], worldPos[2], 1);
				ptampoints.push_back(mse3CfW*worldPosHom);
			}

			KINECT::calcPTAMfromKinect(ptampoints,_depthdata);

			ptamcalib = true;
			delete [] _depthdata;
		}
		else{
			ptamcalib = true;
		}
	}

	void SkeletonTaker::recalibPTAM(bool calib){
		ptamcalib = calib;
	}

	void SkeletonTaker::ProjectGrid(TooN::SE3<> mse3CfW){
		/*
		if(!ptamcalib) return;

		gridpts.clear();
		gridpts2.clear();

		NUI_DEPTH_IMAGE_POINT * _depthdata = new NUI_DEPTH_IMAGE_POINT[640 * 480];

		KINECT::getKinectData_depth_raw(_depthdata);

		TooN::SE3<> cam2world = mse3CfW.inverse();
		TooN::Matrix<4,4> K2P = KINECT::getPTAMfromKinect();

		INuiCoordinateMapper* mapper;
		sensor->NuiGetCoordinateMapper(&mapper);

		for(int x=64;x<WIDTH;x+=64){
			for(int y=64;y<HEIGHT;y+=64){
				if(_depthdata[WIDTH-1-x+y*WIDTH].depth == 0) continue;
				Vector4 skeletonPt;
				mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &_depthdata[WIDTH-1-x+y*WIDTH], &skeletonPt);
				//skeletonPt = KinectFlip(skeletonPt);
				gridpts.push_back(cam2world * K2P * Vector2TooN(skeletonPt));
				gridpts2.push_back(cam2world * Vector2TooN(skeletonPt));
				std::cout << _depthdata[WIDTH-1-x+y*WIDTH].depth << ", (" << WIDTH-1-x << ", " << y << ") -> " << gridpts2.back() << std::endl;
			}
		}
		std::cout << "Grid size: " << gridpts.size() << std::endl;

		delete [] _depthdata;
		*/
	};
}

#else

#include <iostream>
#include <vector>
#include <set>

#include "pthread.h"
#include "TooN\TooN.h"
#include "TooN\se3.h"
#include "tinyxml.h"

#include "SkeletonTaker.h"
#include "KinectManager.h"
#include "ghostutil.h"
#include "PTAM2Kinect.h"

#include "loader.h"
#include "util.h"

#include "definitions.h"

using namespace PTAMM;
namespace KINECT{

	SkeletonTaker::SkeletonTaker(){
		Reset();
	}

	void SkeletonTaker::Reset(){
		ptamcalib = false;
		//proc_vidRecord.clear();
		//getVidRecord().clear();
		gridpts.clear();
		gridpts2.clear();
	}

	void SkeletonTaker::calibPTAM(std::vector<PTAMM::MapPoint*> points, TooN::SE3<> mse3CfW){
		
		if(!ptamcalib && doCalib())
		{
			std::cout << "calibrating PTAM to Kinect!\n";

			DepthXY * _depthdata = new DepthXY[CAPTURE_SIZE_X*CAPTURE_SIZE_Y];
			KINECT::getKinectData_depth_raw(_depthdata);

			std::vector<TooN::Vector<4>> ptampoints;
			for(auto it=points.begin();it!=points.end();++it){
				Vector<3> worldPos = (*it)->v3WorldPos;
				Vector<4> worldPosHom = TooN::makeVector(worldPos[0], worldPos[1], worldPos[2], 1);
				ptampoints.push_back(mse3CfW*worldPosHom);
			}

			KINECT::calcPTAMfromKinect(ptampoints, _depthdata);

			//cv::Mat dmap = KINECT::getDepthFrame();

			//KINECT::calcPTAMfromKinect(ptampoints, dmap);

			ptamcalib = true;
			delete [] _depthdata;
		}
		else{
			ptamcalib = true;
		}
	}

	void SkeletonTaker::recalibPTAM(bool calib){
		ptamcalib = calib;
	}

	void SkeletonTaker::ProjectGrid(TooN::SE3<> mse3CfW){
		
		if(!ptamcalib) return;
		//KINECT::GridProjection(mse3CfW

		/*
		gridpts.clear();
		gridpts2.clear();

		NUI_DEPTH_IMAGE_POINT * _depthdata = new NUI_DEPTH_IMAGE_POINT[640 * 480];

		KINECT::getKinectData_depth_raw(_depthdata);

		TooN::SE3<> cam2world = mse3CfW.inverse();
		TooN::Matrix<4,4> K2P = KINECT::getPTAMfromKinect();

		INuiCoordinateMapper* mapper;
		sensor->NuiGetCoordinateMapper(&mapper);

		const int WIDTH = CAPTURE_SIZE_X;
		const int HEIGHT = CAPTURE_SIZE_Y;

		for(int x=64;x<WIDTH;x+=64){
			for(int y=64;y<HEIGHT;y+=64){
				if(_depthdata[WIDTH-1-x+y*WIDTH].depth == 0) continue;
				Vector4 skeletonPt;
				mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &_depthdata[WIDTH-1-x+y*WIDTH], &skeletonPt);
				//skeletonPt = KinectFlip(skeletonPt);
				gridpts.push_back(cam2world * K2P * Vector2TooN(skeletonPt));
				gridpts2.push_back(cam2world * Vector2TooN(skeletonPt));
				std::cout << _depthdata[WIDTH-1-x+y*WIDTH].depth << ", (" << WIDTH-1-x << ", " << y << ") -> " << gridpts2.back() << std::endl;
			}
		}
		std::cout << "Grid size: " << gridpts.size() << std::endl;

		delete [] _depthdata;
		*/
	};
}

#endif