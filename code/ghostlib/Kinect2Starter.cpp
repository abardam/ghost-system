#include "Kinect2Starter.h"
#include "KinectManager.h"
#include <Kinect.h>
#include <iostream>

void DumpHR(HRESULT hr)
{
	//
	//if (hr < 0)
	//	hr += 0x100000000;
	//if (hr & 0x80000000)
	//	std::cout << ("Error code") << std::endl;
	//else
	//	std::cout << ("Success code") << std::endl;
	//
	//auto facility = (hr & 0x7FFF0000) >> 16;
	//
	//std::cout << "Facility " << facility << std::endl;
	//
	//auto scode = hr & 0x0000FFFF;
	//
	//std::cout << "SCode " << scode << std::endl;

}


namespace KINECT{

	IKinectSensor * m_pKinectSensor;
	ICoordinateMapper * m_pCoordinateMapper;
	IBodyFrameReader * m_pBodyFrameReader;
	IColorFrameReader * m_pColorFrameReader;
	IDepthFrameReader * m_pDepthFrameReader;

	INT64 m_nStartTime;

    RGBQUAD * m_pDepthRGBX;
	USHORT * m_pDepth;
    RGBQUAD * m_pColorRGBX;

	unsigned int m_nDepthWidth;
	unsigned int m_nDepthHeight;
	unsigned int m_nColorWidth;
	unsigned int m_nColorHeight;

	bool m_bCalculateDepthRGBX;

	void InitKinect2Starter(){
		
		m_pDepthRGBX = new RGBQUAD[CAPTURE_SIZE_X_DEPTH * CAPTURE_SIZE_Y_DEPTH];
		m_pDepth = new USHORT[CAPTURE_SIZE_X_DEPTH * CAPTURE_SIZE_Y_DEPTH];
		m_pColorRGBX = new RGBQUAD[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];

		m_nStartTime = 0;
		m_bCalculateDepthRGBX = false;
	}

	void DestroyKinect2Starter(){
		if(m_pDepthRGBX){
			delete [] m_pDepthRGBX;
			m_pDepthRGBX = NULL;
		}
		if(m_pDepth){
			delete [] m_pDepth;
			m_pDepth = NULL;
		}
		if(m_pColorRGBX){
			delete [] m_pColorRGBX;
			m_pColorRGBX = NULL;
		}
	}

	HRESULT InitializeDefaultSensor()
	{
		HRESULT hr;

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
		{
			return hr;
		}

		if (m_pKinectSensor)
		{
			// Initialize the Kinect and get coordinate mapper and the body reader
			IBodyFrameSource* pBodyFrameSource = NULL;

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
			}

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
			}

			SafeRelease(pBodyFrameSource);

			
			// get the color reader
			IColorFrameSource* pColorFrameSource = NULL;

			if (SUCCEEDED(hr))
			{

				hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
			} 

			SafeRelease(pColorFrameSource);

			
			// get the depth reader

			IDepthFrameSource* pDepthFrameSource = NULL;

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
			}

			SafeRelease(pDepthFrameSource);

		}

		if (!m_pKinectSensor || FAILED(hr))
		{
			std::cout << "no ready Kinect found!";
			return E_FAIL;
		}

		return hr;
	}

	//after calling this, get the depth fram with GetDepth or GetDepthRGBX
	void UpdateDepth(){

		if (!m_pDepthFrameReader)
		{
			return;
		}

		IDepthFrame* pDepthFrame = NULL;

		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxReliableDistance = 0;
			UINT nBufferSize = 0;
			UINT16 *pBuffer = NULL;

			hr = pDepthFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				m_nDepthWidth = nWidth;
				hr = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr))
			{
				m_nDepthHeight = nHeight;
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
			}

			if (SUCCEEDED(hr))
			{
				if(m_bCalculateDepthRGBX)
					ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxReliableDistance);
				else
					ProcessDepthNoRGBX(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxReliableDistance);
			}

			SafeRelease(pFrameDescription);
		}
		else{
			DumpHR(hr);
		}

		SafeRelease(pDepthFrame);
	}

	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		// Make sure we've received valid data
		if (m_pDepthRGBX && m_pDepth && pBuffer)
		{
			RGBQUAD* pRGBX = m_pDepthRGBX;
			USHORT * pDepth = m_pDepth;

			// end pixel is start + width*height - 1
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

			while (pBuffer < pBufferEnd)
			{
				USHORT depth = *pBuffer;
				*pDepth = depth;

				// To convert to a byte, we're discarding the most-significant
				// rather than least-significant bits.
				// We're preserving detail, although the intensity will "wrap."
				// Values outside the reliable depth range are mapped to 0 (black).

				// Note: Using conditionals in this loop could degrade performance.
				// Consider using a lookup table instead when writing production code.
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

				pRGBX->rgbRed   = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue  = intensity;

				++pRGBX;
				++pDepth;
				++pBuffer;
			}

		}
	}

	void ProcessDepthNoRGBX(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		// Make sure we've received valid data
		if (m_pDepth && pBuffer)
		{
			USHORT * pDepth = m_pDepth;

			// end pixel is start + width*height - 1
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

			while (pBuffer < pBufferEnd)
			{
				USHORT depth = *pBuffer;
				*pDepth = depth;

				++pDepth;
				++pBuffer;
			}

		}
	}

	void UpdateColor()
	{
		if (!m_pColorFrameReader)
		{
			return;
		}

		IColorFrame* pColorFrame = NULL;

		HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nBufferSize = 0;
			RGBQUAD *pBuffer = NULL;

			hr = pColorFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_FrameDescription(&pFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				m_nColorWidth = nWidth;
				hr = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr))
			{
				m_nColorHeight = nHeight;
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
				}
				else if (m_pColorRGBX)
				{
					pBuffer = m_pColorRGBX;
					nBufferSize = nWidth * nHeight * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);            
				}
				else
				{
					hr = E_FAIL;
				}
			}

			if (SUCCEEDED(hr))
			{
				ProcessColor(nTime, pBuffer, nWidth, nHeight);
			}

			SafeRelease(pFrameDescription);
		}
		else{
			DumpHR(hr);
		}

		SafeRelease(pColorFrame);
	}

	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight) 
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		// Make sure we've received valid data
		if (pBuffer)
		{
			RGBQUAD * pColor = m_pColorRGBX;

			// end pixel is start + width*height - 1
			const RGBQUAD * pBufferEnd = pBuffer + (nWidth * nHeight);

			while (pBuffer < pBufferEnd)
			{
				RGBQUAD color = *pBuffer;
				*pColor = color;

				++pColor;
				++pBuffer;
			}
		}
	}
	

	RGBQUAD * GetDepthRGBX(){
		return m_pDepthRGBX;
	}

	USHORT * GetDepth(){
		return m_pDepth;
	}

	RGBQUAD * GetColorRGBX(){
		return m_pColorRGBX;
	}

	unsigned int getDepthWidth(){
		return m_nDepthWidth;
	}

	unsigned int getDepthHeight(){
		return m_nDepthHeight;
	}

	unsigned int getColorWidth(){
		return m_nColorWidth;
	}

	unsigned int getColorHeight(){
		return m_nColorHeight;
	}
}