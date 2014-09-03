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

namespace KINECT{
	void InitKinect2Starter();

	void DestroyKinect2Starter();

	HRESULT InitializeDefaultSensor();
	
	//after calling this, get the depth fram with GetDepth or GetDepthRGBX
	void UpdateDepth();

	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void ProcessDepthNoRGBX(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void UpdateColor();

	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);
	
	RGBQUAD * GetDepthRGBX();

	USHORT * GetDepth();

	RGBQUAD * GetColorRGBX();

	UINT8 getDepthWidth();

	UINT8 getDepthHeight();

	UINT8 getColorWidth();

	UINT8 getColorHeight();
}