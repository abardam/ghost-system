#include "ghostsettings.h"

#if GHOST_INPUT == INPUT_OPENNI || GHOST_INPUT == INPUT_KINECT2

// Copyright 2008 Isis Innovation Limited
// This VideoSource for Win32 uses CMU's 1394 driver
// available at 
// http://www.cs.cmu.edu/~iwan/1394/

#define WIN32_LEAN_AND_MEAN
#include "VideoSource.h"
#include <Windows.h>
#include <cvd\utility.h>

#include "KinectManager.h"

#include <opencv2\opencv.hpp>

#include "gvars3\instances.h"
#include "definitions.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

//videosource should also be able to load from file
//can we control this thru gvars?

void GUICommandCallback(void *ptr, string sCommand, string sParams){
	VideoSource * vs = static_cast<VideoSource*>(ptr);
	if(sCommand == "GH_CaptureReset"){
		vs->setFrame(1);
	}
}

VideoSource::VideoSource()//: cam(1)
{
	KINECT::init();
	mirSize.x = CAPTURE_SIZE_X;
	mirSize.y = CAPTURE_SIZE_Y;

	GV2.Register(captureType, "GH_CaptureType", CT_KINECT, SILENT);
	GV2.Register(vidPath, "GH_VidPath", "videoin/", SILENT);
	GV2.Register(dumpFeed, "GH_DumpFeed", 0, SILENT);

	GUI.RegisterCommand("GH_CaptureReset", GUICommandCallback, this);

	m_buffer = new unsigned char[CAPTURE_SIZE_X*CAPTURE_SIZE_Y * 3];
	
	currFrame = 1;
	currOut = 1;
	
	GetLocalTime(&prevtime);
	elapsed = 0;

	refreshVidDirectory();
};

VideoSource::~VideoSource()
{
	delete[] m_buffer;
	KINECT::release();
};

void VideoSource::setFrame(int f){
	currFrame = f;
};

//copied from GhostGame.cpp
void VideoSource::refreshVidDirectory(){
	time_t t = time(0);
    struct tm now;
	localtime_s(&now, &t);
	char buf[100];

	sprintf_s(buf, "kinectdump-%d-%d-%d-%02d%02d", now.tm_year+1900, now.tm_mon+1, now.tm_mday, now.tm_hour, now.tm_sec);

	dumpPath = std::string(buf);
};

void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{
	
	SYSTEMTIME time;
	GetLocalTime(&time);

	int temp = time.wMilliseconds - prevtime.wMilliseconds + 1000*(time.wSecond - prevtime.wSecond);

	if(temp < 0) temp = 0;

	elapsed += temp;
	prevtime = time;

	KINECT::updateFrames();

	while(elapsed >= (1000/FRAMERATE))
	{
		++currFrame;
		elapsed -= (1000/FRAMERATE);
	}

	imRGB.resize(mirSize);
	imBW.resize(mirSize);

	
	if(*captureType == CT_KINECT){

		//max stuff
		//cv::Mat input;
		//cam >> input;

		//mirSize.x = input.cols;
		//mirSize.y = input.rows;
		

		//imRGB.resize(mirSize);
		//imBW.resize(mirSize);

		//m_buffer = input.ptr();

		cv::Mat video_ = KINECT::getColorFrame();

		if (video_.empty()) return;
		cv::Mat video;

		cv::resize(video_, video, cv::Size(mirSize.x, mirSize.y));
		//getDepthData(m_buffer);
		//getRandomData(m_buffer);

		unsigned char* pImage = video.ptr<unsigned char>();

		for (int y = 0; y<mirSize.y; y++) {
			for (int x = 0; x<mirSize.x; x++) {
				int x_ = mirSize.x - x - 1;

				imRGB[y][x_].blue = *pImage;
				pImage++;

				imRGB[y][x_].green = *pImage;
				imBW[y][x_] = *pImage;
				pImage++;

				imRGB[y][x_].red = *pImage;
				pImage++;

				if (video.channels() == 4)
					++pImage;

			}
		}

		if(*dumpFeed){
			
			CreateDirectoryA(dumpPath.c_str(), NULL);

			std::stringstream ssPath;
			ssPath << dumpPath << "/" << currOut << ".png";

			++currOut;

			cv::imwrite(ssPath.str(), video);
		}

	}else if(*captureType == CT_FILE){

		std::stringstream ssPath;
		ssPath << *vidPath << currFrame << ".png";

		cv::Mat im_ = cv::imread(ssPath.str(), CV_LOAD_IMAGE_COLOR);

		if(im_.empty()){
			currFrame=1;
			return;
		}

		cv::Mat im(mirSize.y, mirSize.x, CV_8UC3);

		cv::resize(im_, im, im.size());

		unsigned char* pImage = im.ptr();

		for (int y = 0; y<mirSize.y; y++) {
			for (int x = 0; x<mirSize.x; x++) {
				int x_ = mirSize.x - x - 1;

				imRGB[y][x_].blue = *pImage;
				pImage++;

				imRGB[y][x_].green = *pImage;
				imBW[y][x_] = *pImage;
				pImage++;

				imRGB[y][x_].red = *pImage;
				pImage++;

			}
		}
	}
}

ImageRef VideoSource::Size()
{
	return mirSize;
}
#endif
#if GHOST_INPUT == INPUT_VI


// This VideoSource for Win32 uses EWCLIB
//
// EWCLIB ver.1.2
// http://www.geocities.jp/in_subaru/ewclib/index.html

#define WIN32_LEAN_AND_MEAN
#include <Ole2.h>
#include <Windows.h>

#include "VideoSource.h"
#include <cvd/utility.h>

#include "KinectStarter.h"
#include "NuiApi.h"

#include "videoInput.h"

using namespace CVD;
using namespace std;

#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480
#define FPS				30

// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
INuiSensor* sensor;            // The kinect sensor

bool useKinect = false;
bool VIinit = false;
videoInput *VI;
int deviceID;
/*
bool initKinect() {
	/
	bool a = KINECT::initKinect();

	rgbStream = KINECT::getRGB();
	sensor = KINECT::getSensor();
	


	return a;
}*/

void StopEvent(int deviceID, void *userData)
{
	videoInput *VI = &videoInput::getInstance();

	VI->closeDevice(deviceID);
}

bool initVideoInput(){
	if (!VIinit){
		VI = &videoInput::getInstance();
		int i = VI->listDevices();

		if (i < 1) return false;
		deviceID = i - 1;
		if (!VI->setupDevice(deviceID, CAPTURE_SIZE_X, CAPTURE_SIZE_Y, 60)) return false;
		VI->setEmergencyStopEvent(deviceID, NULL, StopEvent);

		VIinit = true;
			
	}

	return VIinit;
}



VideoSource::VideoSource()
{
	m_buffer = new unsigned char[CAPTURE_SIZE_X*CAPTURE_SIZE_Y*4];
	m_buffer2 = new unsigned char[CAPTURE_SIZE_X*CAPTURE_SIZE_Y*4];

	mirSize.x = CAPTURE_SIZE_X;
	mirSize.y = CAPTURE_SIZE_Y;
};

VideoSource::~VideoSource()
{
    delete[] m_buffer;
    delete[] m_buffer2;
}

void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{
	if (false && useKinect){
		/*
		KINECT::getKinectData(m_buffer);

		unsigned char* pImage = m_buffer;
		unsigned char* pImage2;

		BasicImage<CVD::byte> imCaptured(pImage, mirSize);
		imRGB.resize(mirSize);
		imBW.resize(mirSize);

		if (depth){
			//KINECT::getKinectData_color_player(m_buffer2, 1);
			KINECT::getKinectData_depth(m_buffer2);
			pImage2 = m_buffer2;
		}

		for (int y = 0; y < mirSize.y; y++) {
			for (int x = 0; x < mirSize.x; x++) {
				int x_ = mirSize.x - x - 1;

				if (depth)
				{
					imRGB[y][x_].blue = *pImage2++;
					pImage++;

					imRGB[y][x_].green = *pImage2++;
					imBW[y][x_] = *pImage;
					pImage++;

					imRGB[y][x_].red = *pImage2++;
					pImage++;

					pImage++;
					++pImage2;
				}
				else{
					imRGB[y][x_].blue = *pImage;
					pImage++;

					imRGB[y][x_].green = *pImage;
					imBW[y][x_] = *pImage;
					pImage++;

					imRGB[y][x_].red = *pImage;
					pImage++;

					pImage++;
				}
			}
		}*/
	}
	else{
		if (!initVideoInput()){
			std::cerr << "VideoInput not initialized!\n";
			return;
		}


		imRGB.resize(mirSize);
		imBW.resize(mirSize);

		if (!VI->isFrameNew(deviceID)) return;

		VI->getPixels(deviceID, m_buffer);

		unsigned char* pImage = m_buffer;
		for (int y = 0; y < mirSize.y; y++) {
			for (int x = 0; x < mirSize.x; x++) {
				int x_ = x;//mirSize.x - x - 1;

				
				imRGB[y][x_].blue = *pImage;
				pImage++;

				imRGB[y][x_].green = *pImage;
				imBW[y][x_] = *pImage;
				pImage++;

				imRGB[y][x_].red = *pImage;
				pImage++;

			}
		}

	}

}

ImageRef VideoSource::Size()
{
	return mirSize;
}


#endif