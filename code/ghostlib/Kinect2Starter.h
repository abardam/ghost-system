#pragma once

#include <Windows.h>

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

#define CAPTURE_SIZE_X_COLOR 1920
#define CAPTURE_SIZE_Y_COLOR 1080
#define CAPTURE_SIZE_X_DEPTH 512
#define CAPTURE_SIZE_Y_DEPTH 424

namespace KINECT{
	void InitKinect2Starter();

	void DestroyKinect2Starter();

	HRESULT InitializeDefaultSensor();
	
	//after calling this, get the depth frame with GetDepth or GetDepthRGBX or the color frame with GetColorRGBX
	void UpdateMulti();

	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void ProcessDepthNoRGBX(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);
	
	RGBQUAD * GetDepthRGBX();

	USHORT * GetDepth();

	RGBQUAD * GetColorRGBX();

	unsigned int getDepthWidth();
	unsigned int getDepthHeight();
	unsigned int getColorWidth();
	unsigned int getColorHeight();
}