#include "tinyxml.h"
#include "OpenNIStarter.h"
//#include "GCoptimization.h"
#include "cylinder.h"
#include "definitions.h"
#include "bodybuild.h"

#include <array>
#include <vector>
#include <fstream>

#include "loader.h"
#include "cvutil.h"
#include "ghostutil.h"
#include "KinectManager.h"

#include "opencv2\opencv.hpp"
#include <TooN\TooN.h>
#include "TooN\se3.h"


bool printout;



void initLoader(){
	KINECT::init();
}


std::vector<bool> LoadVideo(cv::Mat matCfW, cv::Mat K2P, std::vector<SkeleVideoFrame> * vidRecord, std::vector<Skeleton> * wcSkeletons, std::string path, bool loadRGB){
	cv::Mat cam2World = matCfW.inv();
		
	char buffer[100];

	TiXmlDocument xmlDoc;

	if(!xmlDoc.LoadFile(path + "SVF.xml")){
		std::cerr << "Failed to load video. Aborting.\n";
		return std::vector<bool>();
	}

	TiXmlHandle doc(&xmlDoc);
	TiXmlElement * elem;
	TiXmlHandle root(0);

	elem = doc.FirstChildElement().Element();
	if(!elem){
		std::cerr << "No root handle in video XML file\n";
		return std::vector<bool>();
	}

	
	double fileVersion = 0.0;
	elem->QueryDoubleAttribute("version", &fileVersion);

	root = TiXmlHandle(elem);

	TiXmlHandle framesNode = root.FirstChild("frames");
	
	std::vector<bool> ret;

	for(TiXmlElement * elem = framesNode.FirstChild().Element(); 
		elem != NULL; elem = elem->NextSiblingElement()){
			
			ret.push_back(false);

			SkeleVideoFrame temp;

			if(loadRGB && elem->Attribute("frame") != NULL)
			{
				std::string ffilename = path + elem->Attribute("frame");
				
				//old ver
				if(fileVersion == 1.1){
					ffilename = elem->Attribute("frame");
				}

#if IMAGE_CHANNELS == 3
				temp.videoFrame.mat = cv::imread(ffilename, CV_LOAD_IMAGE_COLOR);
#endif
#if IMAGE_CHANNELS == 4
				temp.videoFrame.mat = cv::imread(ffilename, CV_LOAD_IMAGE_UNCHANGED);
#endif
				if(temp.videoFrame.mat.empty()) {
					std::cout << "unable to read " << ffilename << "; skipping\n";
				}
				double ox=0, oy=0;
				if(elem->Attribute("offsetX") != NULL) elem->QueryDoubleAttribute("offsetX", &ox);
				if(elem->Attribute("offsetY") != NULL) elem->QueryDoubleAttribute("offsetY", &oy);

				temp.videoFrame.offset.x = ox;
				temp.videoFrame.offset.y = oy;
			}

			if(elem->Attribute("framedepth") != NULL)
			{
				std::string dfilename = path + elem->Attribute("framedepth");

				//old ver
				if(fileVersion == 1.1){
					//intense workaround
					dfilename = elem->Attribute("framedepth");
					if(dfilename.substr(0,16)=="map000000/video/"){
						dfilename = path + dfilename.substr(16);
					}
				}

				cv::FileStorage fs(dfilename.c_str(), cv::FileStorage::READ);
				fs["depth"] >> temp.depthFrame;
				fs.release();
				if(temp.depthFrame.empty()) {
					std::cout << "unable to read " << dfilename << "; skipping\n";
				}
			}

			TiXmlElement * kinPtsElem = elem->FirstChildElement();

			int i=0;
			
			float skpoints[4*NUMJOINTS];

			
			for(TiXmlElement * kinPointElem = kinPtsElem->FirstChildElement();
				kinPointElem != NULL; kinPointElem = kinPointElem->NextSiblingElement()){
					kinPointElem->QueryFloatAttribute("X", &(skpoints[i+0*NUMJOINTS]));
					kinPointElem->QueryFloatAttribute("Y", &(skpoints[i+1*NUMJOINTS]));
					kinPointElem->QueryFloatAttribute("Z", &(skpoints[i+2*NUMJOINTS]));
					skpoints[i+3*NUMJOINTS] = 1;

					float tempstate;
					kinPointElem->QueryFloatAttribute("state", &tempstate);
					temp.kinectPoints.states[i] = tempstate;
					++i;
			}

			temp.kinectPoints.points = cv::Mat(4,NUMJOINTS,CV_32F,skpoints).clone();
			temp.kinectPoints2P = normalizeSkeleton(K2P * temp.kinectPoints.points);

			//old ver
			if (kinPtsElem->Attribute("Cam2World") == NULL){
				Skeleton wcSkeleton = temp.kinectPoints;
				wcSkeleton.points = cam2World * K2P * wcSkeleton.points;
				(*wcSkeletons).push_back(wcSkeleton);
			}
			else {
				std::string cfilename = path + kinPtsElem->Attribute("Cam2World");

				cv::FileStorage fs(cfilename.c_str(), cv::FileStorage::READ);
				fs["cam2world"] >> temp.cam2World;
				fs.release();
				if(temp.cam2World.empty()) {
					std::cout << "unable to read " << cfilename << "; skipping\n";
				}
			}

#if 0 //reworked: PROCESS
			for(TiXmlElement * kinPointElem = kinPtsElem->FirstChildElement();
				kinPointElem != NULL; kinPointElem = kinPointElem->NextSiblingElement()){
					kinPointElem->QueryFloatAttribute("X", &(temp.kinectPoints[i][0]));
					kinPointElem->QueryFloatAttribute("Y", &(temp.kinectPoints[i][1]));
					kinPointElem->QueryFloatAttribute("Z", &(temp.kinectPoints[i][2]));
					//kinPointElem->QueryFloatAttribute("W", &(temp.kinectPoints[i][3]));

					int tempstate;

					kinPointElem->QueryIntAttribute("state", &tempstate);
					
					skpoints[i+0*NUMJOINTS] = (temp.kinectPoints[i])[0];
					skpoints[i+1*NUMJOINTS] = (temp.kinectPoints[i])[1];
					skpoints[i+2*NUMJOINTS] = (temp.kinectPoints[i])[2];
					skpoints[i+3*NUMJOINTS] = 1;
					//skpoints[i+3*NUMJOINTS] = (temp.kinectPoints[i])[3];

					//Vector4 temv = temp.kinectPoints[i];
					//temp.skeleton.points[i] = cam2World * K2P * KinectFlip(Vector2TooN(temv));
					temp.skeleton.states[i] = tempstate;

					++i;
				}

			
			temp.skeleton.points = cv::Mat(4,NUMJOINTS,cv::DataType<float>::type,skpoints).clone();

			temp.kinectPoints2P = normalizeSkeleton( K2P * temp.skeleton.points );

			temp.skeleton.points = cam2World * K2P /* getScaleMatrix(-1,-1,1) */ * temp.skeleton.points;
#endif
			//calculate temp.skeleton.points


			vidRecord->push_back(temp);

			ret.back() = true;
	}

	return ret;
}

void SaveVideo(std::vector<SkeleVideoFrame> * vidRecord, std::string path){
	CreateDirectoryA(path.c_str(), NULL);
		
	char buffer[100];
	char buffer2[100];

	TiXmlDocument xmlDoc;

	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	xmlDoc.LinkEndChild(decl);

	TiXmlElement * rootNode = new TiXmlElement("SkeletonTaker");
	xmlDoc.LinkEndChild(rootNode);
	rootNode->SetDoubleAttribute("version", 1.3);


	TiXmlElement * framesNode = new TiXmlElement("frames");
	rootNode->LinkEndChild(framesNode);
	framesNode->SetAttribute("size", vidRecord->size());

	for(int i=0;i<vidRecord->size();++i){

		TiXmlElement * svfNode = new TiXmlElement("SkeleVideoFrame");
		framesNode->LinkEndChild(svfNode);

		if(!(*vidRecord)[i].videoFrame.mat.empty())
		{
			sprintf(buffer, "%sframe%d.png", path.c_str(), i);
			sprintf(buffer2, "frame%d.png", i);
			cvSaveImage(buffer, &(IplImage((*vidRecord)[i].videoFrame.mat)));
			svfNode->SetAttribute("frame", std::string(buffer2));
			svfNode->SetAttribute("offsetX", (*vidRecord)[i].videoFrame.offset.x);
			svfNode->SetAttribute("offsetY", (*vidRecord)[i].videoFrame.offset.y);
		}

		if(!(*vidRecord)[i].depthFrame.empty())
		{
			sprintf(buffer, "%sframe%d_depth.yml", path.c_str(), i);
			sprintf(buffer2, "frame%d_depth.yml", i);
			cv::FileStorage fs(buffer, cv::FileStorage::WRITE);
			fs << "depth" << (*vidRecord)[i].depthFrame;
			fs.release();
			svfNode->SetAttribute("framedepth", std::string(buffer2));
		}
			
		TiXmlElement * kinectPointsNode = new TiXmlElement("KinectPoints");
		svfNode->LinkEndChild(kinectPointsNode);

		//kinect points
		for(int j=0;j<NUMJOINTS;++j){
			TiXmlElement * kinectPointNode = new TiXmlElement("Point");
			kinectPointsNode->LinkEndChild(kinectPointNode);

			kinectPointNode->SetDoubleAttribute("X", (*vidRecord)[i].kinectPoints.points.at<float>(0,j));
			kinectPointNode->SetDoubleAttribute("Y", (*vidRecord)[i].kinectPoints.points.at<float>(1,j));
			kinectPointNode->SetDoubleAttribute("Z", (*vidRecord)[i].kinectPoints.points.at<float>(2,j));
			kinectPointNode->SetDoubleAttribute("W", 1);
			kinectPointNode->SetDoubleAttribute("state",   (*vidRecord)[i].kinectPoints.states[j]);
		}

		if(!(*vidRecord)[i].cam2World.empty())
		{
			sprintf(buffer, "%sframe%d_cam2world.yml", path.c_str(), i);
			sprintf(buffer2, "frame%d_cam2world.yml", i);
			cv::FileStorage fs(buffer, cv::FileStorage::WRITE);
			fs << "cam2world" << (*vidRecord)[i].cam2World;
			fs.release();
			svfNode->SetAttribute("Cam2World", std::string(buffer2));
		}

		//Skeleton
		//actually don't need skeleton
		//skeleton will be recalculated upon loading, using new K2P and cam position
	}

	xmlDoc.SaveFile(path + "SVF.xml");
}


