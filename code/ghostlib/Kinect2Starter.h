#pragma once

#include <Windows.h>
#include <Kinect.h>

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
	
	//after calling this, get the depth fram with GetDepth or GetDepthRGBX
	void UpdateDepth();

	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void ProcessDepthNoRGBX(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void ProcessColorToDepth(const RGBQUAD * pColorBuffer, int nColorWidth, int nColorHeight, const DepthSpacePoint * pColorDepthMap, int nDepthWidth, int nDepthHeight);

	void UpdateColor();

	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);
	
	RGBQUAD * GetDepthRGBX();

	USHORT * GetDepth();

	RGBQUAD * GetColorRGBX();

	RGBQUAD * GetColorMappedToDepth();

	unsigned int getDepthWidth();
	unsigned int getDepthHeight();
	unsigned int getColorWidth();
	unsigned int getColorHeight();
}