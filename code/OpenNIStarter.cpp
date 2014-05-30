#include <OpenNI.h>
#include "OpenNIStarter.h"
#include <time.h>
#include <NiTE.h>
#include "KinectManager.h"

#define REFRESH_RATE (1./FPS)
#define MAX_DEPTH 10000

namespace KINECT
{

	openni::Device device;
	openni::VideoStream depth, color;
	openni::VideoFrameRef m_depthFrame;
	openni::VideoFrameRef m_colorFrame;
	//openni::RGB888Pixel* m_pTexMap;

	bool _init = false;
	bool _color = false;
	bool _depth = false;
	bool _nite = false;
	bool _userTracker = false;

	//nite
	nite::UserTracker * userTracker;
	nite::UserTrackerFrameRef userTrackerFrame;
	nite::UserMap userMaps;

	//NiTE sample utilities
	void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame)
	{
		const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
		int width = depthFrame.getWidth();
		int height = depthFrame.getHeight();
		// Calculate the accumulative histogram (the yellow display...)
		memset(pHistogram, 0, histogramSize*sizeof(float));
		int restOfRow = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel) - width;

		unsigned int nNumberOfPoints = 0;
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x, ++pDepth)
			{
				if (*pDepth != 0)
				{
					pHistogram[*pDepth]++;
					nNumberOfPoints++;
				}
			}
			pDepth += restOfRow;
		}
		for (int nIndex=1; nIndex<histogramSize; nIndex++)
		{
			pHistogram[nIndex] += pHistogram[nIndex-1];
		}
		if (nNumberOfPoints)
		{
			for (int nIndex=1; nIndex<histogramSize; nIndex++)
			{
				pHistogram[nIndex] = (256 * (1.0f - (pHistogram[nIndex] / nNumberOfPoints)));
			}
		}
	}

	
	void initFromFile(std::string filename){
#if !INIT_KINECT
		_init = false;
		return;
#endif

		openni::Status rc = openni::OpenNI::initialize();

		printf("After initialization from file!:\n%s\n", openni::OpenNI::getExtendedError());

		rc = device.open(filename.c_str());

		if (rc != openni::STATUS_OK)
		{
			printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			_init = false;
			return;
		}

		rc = depth.create(device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			rc = depth.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				depth.destroy();
			}else{
				_depth = true;
			}
		}
		else
		{
			printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		}

		_init = true;
	}

	bool initNI(){
#if !INIT_KINECT
		_init = false;
		return _init;
#endif

		if(_init) return _init;

		openni::Status rc = openni::STATUS_OK;

		const char* deviceURI = openni::ANY_DEVICE;

		rc = openni::OpenNI::initialize();

		printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

		rc = device.open(deviceURI);
		if (rc != openni::STATUS_OK)
		{
			printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			_init = false;
			return _init;
		}

		rc = depth.create(device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			rc = depth.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				depth.destroy();
			}else{
				_depth = true;
			}
		}
		else
		{
			printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		}

		rc = color.create(device, openni::SENSOR_COLOR);
		if (rc == openni::STATUS_OK)
		{
			rc = color.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
				color.destroy();
			}else{
				_color = true;
			}
		}
		else
		{
			printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
		}

	
		auto a = device.getSensorInfo(openni::SENSOR_COLOR);
		openni::VideoMode bestVM;
		for(int i=0; i < a->getSupportedVideoModes().getSize(); ++i){
			if(a->getSupportedVideoModes()[i].getResolutionX() == CAPTURE_SIZE_X && a->getSupportedVideoModes()[i].getResolutionY() == CAPTURE_SIZE_Y && a->getSupportedVideoModes()[i].getPixelFormat() == openni::PIXEL_FORMAT_RGB888){
				bestVM = a->getSupportedVideoModes()[i];
				color.setVideoMode(bestVM);

				break;
			}
		}

		if (!depth.isValid() || !color.isValid())
		{
			printf("No valid streams. Exiting\n");
			openni::OpenNI::shutdown();
			_init = false;
			_depth = false;
			_color = false;
			return _init;
		}

		device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

		userTracker = new nite::UserTracker;

		nite::NiTE::initialize();

		if(userTracker->create(&device) != nite::STATUS_OK){
		
			printf("NiTe failed. Exiting\n");
			openni::OpenNI::shutdown();
			nite::NiTE::shutdown();
			delete userTracker;
			_init = false;
			return _init;
		}else{
			_nite = true;
			_userTracker = true;
		}


		_init = true;
		return _init;
	}

	void shutdownNI(){
#if !INIT_KINECT
		return;
#endif
		if(_userTracker)
			delete userTracker;
		if(_nite){
			nite::NiTE::shutdown();
		}
		if(_color)
			color.destroy();
		if(_depth)
			depth.destroy();
		if(_init)
			openni::OpenNI::shutdown();
	}

	bool good;
	int chosenUserID = -1;
	const nite::Skeleton * skeleton;
	void refreshSkeletonData(){
#if !INIT_KINECT
		return;
#endif
		const nite::Array<nite::UserData>& a = userTrackerFrame.getUsers();

		if(chosenUserID > 0 && chosenUserID < a.getSize())
		{
			const nite::UserData& user = a[chosenUserID];

			if(!user.isLost() && user.getSkeleton().getState() == nite::SKELETON_TRACKED){

				skeleton = &user.getSkeleton();

				good = true;
				chosenUserID = user.getId();

				return;
			}else{

				userTracker->stopSkeletonTracking(chosenUserID);
			}
		}else{
			chosenUserID = -1;
		}

		for(int i=0;i<a.getSize();++i){

			const nite::UserData& user = a[i];

			if(user.isNew()){
				userTracker->startSkeletonTracking(user.getId());
			}
			else if(!user.isLost() && user.getSkeleton().getState() == nite::SKELETON_TRACKED){

				skeleton = &user.getSkeleton();

				good = true;
				chosenUserID = user.getId();

				return;
			}
		}

		skeleton = NULL;

		good = false;
	}

	bool doRefresh = true;
	clock_t t = clock();

	void refresh(){
#if !INIT_KINECT
		return;
#endif
		if(!doRefresh) return;

		clock_t t2 = clock();
		if(t2 - t > REFRESH_RATE * CLOCKS_PER_SEC){

			userTracker->readFrame(&userTrackerFrame);
			color.readFrame(&m_colorFrame);
			depth.readFrame(&m_depthFrame);

			refreshSkeletonData();

			userMaps = userTrackerFrame.getUserMap();

			t = clock();
		}
	}

	void setRefresh(bool r){
		doRefresh = r;
	}

	bool getSkeletonData(nite::Skeleton * s){
	
		refresh();

		if(skeleton == NULL) return false;
		int numT = 0;
		for(int i=0;i<15;++i){
			if(skeleton->getJoint(nite::JointType(i)).getPositionConfidence() > 0.5f)
				++numT;
		}

		if(numT > 1){

			*s = *skeleton;
	
			return good;
		}

		return false;
	}

	void cycleTrackedSkeleton(){
#if !INIT_KINECT
		return;
#endif
		userTracker->stopSkeletonTracking(chosenUserID);
		++chosenUserID;
	}


	void getColorData(unsigned char * dest){								
		if(!initNI()){														
			return;															
		}																	
										
		refresh();
																		
		int m_nTexMapX = CAPTURE_SIZE_X;									
		int m_nTexMapY = CAPTURE_SIZE_Y;									

		memset(dest, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));	

		// check if we need to draw image frame to texture
		if ( m_colorFrame.isValid())
		{
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
			openni::RGB888Pixel* pTexRow = /*m_pTexMap */ (openni::RGB888Pixel*)dest + m_colorFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);

			for (int y = 0; y < m_colorFrame.getHeight(); ++y)
			{
				const openni::RGB888Pixel* pImage = pImageRow;
				openni::RGB888Pixel* pTex = pTexRow + m_colorFrame.getCropOriginX();

				for (int x = 0; x < m_colorFrame.getWidth(); ++x, ++pImage, ++pTex)
				{
					pTex->b = pImage->r;
					pTex->g = pImage->g;
					pTex->r = pImage->b;
				}

				pImageRow += rowSize;
				pTexRow += m_nTexMapX;
			}
		}
	}

	void getPlayerColorData(unsigned char * dest, int * offsetX, int * offsetY, int * maxX, int * maxY){								
		if(!initNI()){														
			return;															
		}																	
										
		refresh();
																		
		int m_nTexMapX = CAPTURE_SIZE_X;									
		int m_nTexMapY = CAPTURE_SIZE_Y;									

		*offsetX = m_colorFrame.getWidth();
		*offsetY = m_colorFrame.getHeight();
		*maxX = 0;
		*maxY = 0;

		memset(dest, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));	

		// check if we need to draw image frame to texture
		if ( m_colorFrame.isValid())
		{
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
			openni::RGB888Pixel* pTexRow = /*m_pTexMap */ (openni::RGB888Pixel*)dest + m_colorFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);
			const nite::UserId* pLabels = userMaps.getPixels();

			for (int y = 0; y < m_colorFrame.getHeight(); ++y)
			{
				const openni::RGB888Pixel* pImage = pImageRow;
				openni::RGB888Pixel* pTex = pTexRow + m_colorFrame.getCropOriginX();

				for (int x = 0; x < m_colorFrame.getWidth(); ++x, ++pImage, ++pTex, ++pLabels)
				{
					if(*pLabels == chosenUserID){

						if(*offsetX > x) *offsetX = x;
						if(*offsetY > y) *offsetY = y;
						if(*maxX < x) *maxX = x;
						if(*maxY < y) *maxY = y;

						pTex->b = pImage->r;
						pTex->g = pImage->g;
						pTex->r = pImage->b;
					}
					else{
						pTex->r = 255;
						pTex->g = 255;
						pTex->b = 255;
					}
				}

				pImageRow += rowSize;
				pTexRow += m_nTexMapX;
			}
		}
	}

	void getDepthData(openni::DepthPixel * dest){
		if(!initNI()){
			return;
		}

		refresh();

		int m_nTexMapX = CAPTURE_SIZE_X;									
		int m_nTexMapY = CAPTURE_SIZE_Y;									
																		
		memset(dest, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::DepthPixel));	

		float depthHist[MAX_DEPTH];

		if(m_depthFrame.isValid()){

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
			openni::DepthPixel* outRow = (openni::DepthPixel*)dest + m_depthFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

			for(int y=0;y<m_depthFrame.getHeight();++y){
				const openni::DepthPixel* pDepth = pDepthRow;
				openni::DepthPixel* pTex = outRow + m_depthFrame.getCropOriginX();

				for(int x=0;x<m_depthFrame.getWidth();++x,++pDepth,++pTex){

					*pTex = *pDepth;
				}

				pDepthRow += rowSize;
				outRow += m_nTexMapX;
			}
		}
	}

	void getDepthDataForDisplay(unsigned char * dest){
		if(!initNI()){
			return;
		}

		refresh();

		int m_nTexMapX = CAPTURE_SIZE_X;									
		int m_nTexMapY = CAPTURE_SIZE_Y;									
																		
		memset(dest, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));	

		float depthHist[MAX_DEPTH];

		if(m_depthFrame.isValid()){
			calculateHistogram(depthHist, MAX_DEPTH, m_depthFrame);

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
			openni::RGB888Pixel* pTexRow = (openni::RGB888Pixel*)dest + m_depthFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

			for(int y=0;y<m_depthFrame.getHeight();++y){
				const openni::DepthPixel* pDepth = pDepthRow;
				openni::RGB888Pixel* pTex = pTexRow + m_depthFrame.getCropOriginX();

				for(int x=0;x<m_depthFrame.getWidth();++x,++pDepth,++pTex){
					if(*pDepth != 0){
						int histValue = depthHist[*pDepth];
						pTex->r = histValue;
						pTex->g = histValue;
						pTex->b = histValue;
					}
				}

				pDepthRow += rowSize;
				pTexRow += m_nTexMapX;
			}
		}
	}

	void getRandomData(unsigned char * dest){
		if(t%2==0){
			getColorData(dest);
		}else{
			getDepthDataForDisplay(dest);
		}
	}

	void mapDepthToSkeleton(float *dX, float *dY, float *dZ, float *sX, float *sY, float *sZ){
		openni::CoordinateConverter::convertDepthToWorld(depth, *dX, *dY, *dZ, sX, sY, sZ);
	}

	void mapSkeletonToDepth(float *sX, float *sY, float *sZ, float *dX, float *dY, float *dZ){
		openni::CoordinateConverter::convertWorldToDepth(depth, *sX, *sY, *sZ, dX, dY, dZ);
	}

	openni::VideoStream * getDepthStream(){
		return &depth;
	}
}