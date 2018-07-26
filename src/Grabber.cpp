//============================================================================
// Name        : Grabber.cpp
// Author      : CosmaC
// Date        : March, 2016
// Copyright   : GWU Research
// Description : Kinect V2 + Flir Lepton grabber
//============================================================================


#pragma warning(push)
#define _AFXDLL
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998

// SensorFusion
#include "Grabber.h"
#include "stdafx.h"
#include "Clustering.h"

// C/C++
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <limits>
#include <string>
#include<ppl.h>

// Windows
#include <Kinect.h>
#include <Windows.h>
#include <Strsafe.h>


/// ???? Remove
/// OTHER UTILS
inline vec3 normalize(vec3 v1) {

    float sum = sqrtf(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    if (sum > 0) {
        return vec3(v1.x / sum, v1.y / sum, v1.z / sum);
    }
    return vec3(0,0,0);
}

inline vec3 cross(vec3 v1, vec3 v2) {

    return vec3((v1.y*v2.z - v1.z*v2.y),
        (v1.z*v2.x - v1.x*v2.z),
        (v1.x*v2.y - v1.y*v2.x));
}


/**
 * @brief Grabber constructor
 */
Grabber::Grabber() :
m_pKinectSensor(NULL),
m_pKinectReader(NULL),
m_pKinectMapper(NULL),
raw_color_RGBX(NULL),
raw_infrared_u16(NULL),
raw_depth_u16(NULL),
m_pDrawColor(NULL),
m_pDrawInfrared(NULL),
m_pDrawDepth(NULL),
m_pDrawResult(NULL),
m_hWnd(NULL),
screenshot_color(false),
screenshot_depth(false),
screenshot_infrared(false) {

	// create heap storage for color pixel data in RGBX format
    aux_color_RGBX = new RGBQUAD[cColorWidth * cColorHeight];
    
    infrared_RGBX = new RGBQUAD[cInfraredWidth * cInfraredHeight];
    aux_infrared_u16 = new UINT16[cInfraredWidth * cInfraredHeight];

    depth_XYZ = new CameraSpacePoint[cColorWidth * cColorHeight];
    depth_RGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
    aux_depth_u16 = new UINT16[cDepthWidth * cDepthHeight];
    
    result_RGBX = new RGBQUAD[cColorWidth * cColorHeight];
}


/**
 * @brief Grabber destructor
 */
Grabber::~Grabber() {

    // Clean frame data buffers
    // Color frame buffers
	if (aux_color_RGBX) {
		delete[] aux_color_RGBX;
        aux_color_RGBX = NULL;
	}
    // Infrared frame buffers
    if (infrared_RGBX) {
        delete[] infrared_RGBX;
        infrared_RGBX = NULL;
    }
    if (aux_infrared_u16) {
        delete[] aux_infrared_u16;
        aux_infrared_u16 = NULL;
    }
    // Depth frame buffers
	if (depth_RGBX) {
		delete[] depth_RGBX;
        depth_RGBX = NULL;
	}
    if (depth_XYZ) {
        delete[] depth_XYZ;
        depth_XYZ = NULL;
    }
    if (aux_depth_u16) {
        delete[] aux_depth_u16;
        aux_depth_u16 = NULL;
    }
    // Results frame buffers
    if (result_RGBX) {
        delete[] result_RGBX;
        result_RGBX = NULL;
    }
	// close the Kinect Sensor
	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
    SafeRelease(m_pKinectReader);
    SafeRelease(m_pKinectMapper);
	// Cluster
	if (cluster) {
		delete cluster;
		cluster = NULL;
	}
}


/**
 * @brief  Initializes the default Kinect sensor
 *
 * @returns  Indicates success or failure
 */
HRESULT Grabber::init() {
    
    // Connect to the default kinect sensor
    HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr) || !m_pKinectSensor) {
        std::cerr << "[Error][Grabber::init] Unable to conect to the Kinect device." 
                  << std::endl;
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }
    // If Kinect open correctly
    else {

        // Open comunication
        hr = m_pKinectSensor->Open();
        if (FAILED(hr)) {
            std::cerr << "[Error][Grabber::init] Unable to conect to the Kinect device."
                << std::endl;
        }
        else {
            hr = m_pKinectSensor->OpenMultiSourceFrameReader(
                 FrameSourceTypes::FrameSourceTypes_Depth |
                 FrameSourceTypes::FrameSourceTypes_Color |
                 FrameSourceTypes::FrameSourceTypes_Infrared,
                 &m_pKinectReader);

            if (FAILED(hr)) {
                std::cerr << "[Error][Grabber::init] Unable to create the multi-frame reader."
                    << std::endl;
            }

            hr = m_pKinectSensor->get_CoordinateMapper(&m_pKinectMapper);
            if (FAILED(hr)) {
                std::cerr << "[Error][Grabber::init] Unable to create the coordinate space mapper."
                    << std::endl;
            }

        }
    }
	// Create cluster 
	cluster = new Clustering();

    return hr;
} /* Grabber::init() */


/**
  * @brief  Updates current frame
  *
  * @returns  Indicates success or failure
  */
HRESULT Grabber::update() {

    // Checks if Kinect connection is open
    if (!m_pKinectReader) {
        std::cerr << "[Error][Grabber::update] Invalid frame reader."
                  << std::endl;
        return E_FAIL;
    }

    // Get current frame
    IMultiSourceFrame* frame = NULL;
    HRESULT hr = m_pKinectReader->AcquireLatestFrame(&frame);

    // Get RGBD data
    if (SUCCEEDED(hr)) {
        
        hr = getDepthData(frame);
        if (FAILED(hr)) {
            std::cerr << "[Error][Grabber::update] Unable to acquire RGB frame."
                      << std::endl;
        }
        else {
            hr = getColorData(frame);
            if (FAILED(hr)) {
                std::cerr << "[Error][Grabber::update] Unable to acquire depth frame."
                          << std::endl;
            }
            else {
                hr = getInfraredData(frame);
                if (FAILED(hr)) {
                    std::cerr << "[Error][Grabber::update] Unable to acquire infrared frame."
                              << std::endl;
                }
            }
        }
    }
    else {
        SetStatusMessage(L"No ready Kinect found!", 10000, false);
        std::cerr << "[Error][Grabber::update] Unable to acquire last frame."
                  << std::endl;
    }

    // Release frame
    SafeRelease(frame);

    return hr;
} /* Grabber::Update() */


/**
 * @brief  Grabs and stores the RGB (color) frame
 *
 * @param frame  Multi-source pointer to the current frame
 *
 * @returns  Indicates success or failure
 */
HRESULT Grabber::getDepthData(IMultiSourceFrame* frame) {

    // Get color frame reference
    IDepthFrameReference* depth_frame_ref = NULL;
    HRESULT hr = frame->get_DepthFrameReference(&depth_frame_ref);
    if (FAILED(hr)) {
        std::cerr << "[Error][Grabber::getDepthData] Unable to get depth frame reference."
            << std::endl;
        return E_FAIL;
    }

    // Get depth frame
    IDepthFrame* depth_frame = NULL;
    hr = depth_frame_ref->AcquireFrame(&depth_frame);

    if (SUCCEEDED(hr)) {

        // Frame properties
        int nWidth = 0;
        int nHeight = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        UINT nBufferSize = 0;
        
        // Get frame descriptor
        IFrameDescription* pFrameDescription = NULL;
        hr = depth_frame->get_FrameDescription(&pFrameDescription);

        if (SUCCEEDED(hr)) {
            hr = pFrameDescription->get_Width(&nWidth);
            hr = pFrameDescription->get_Height(&nHeight);
        }

        hr = depth_frame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        // In order to see the full range of depth (including the less reliable far field depth)
        // we are setting nDepthMaxDistance to the extreme potential depth threshold
        nDepthMaxDistance = USHRT_MAX;
        // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
        //// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);

        // Get depth data
        if (SUCCEEDED(hr)) {
            hr = depth_frame->AccessUnderlyingBuffer(&nBufferSize, &raw_depth_u16);
            memcpy(aux_depth_u16, raw_depth_u16, nBufferSize*sizeof(UINT16));
            raw_depth_u16 = aux_depth_u16;
        }

        // Post-Process depth data
        if (SUCCEEDED(hr)) {
            ProcessDepth(raw_depth_u16, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
        }

        SafeRelease(pFrameDescription);
    }

    // Release frame resources
    SafeRelease(depth_frame);
    SafeRelease(depth_frame_ref);

    return hr;
}


/**
 * @brief  Grabs and stores the depth frame
 *
 * @param frame  Multi-source pointer to the current frame
 *
 * @returns  Indicates success or failure
 */
HRESULT Grabber::getColorData(IMultiSourceFrame* frame) {

    // Get color frame reference
    IColorFrameReference* color_frame_ref = NULL;
    HRESULT hr = frame->get_ColorFrameReference(&color_frame_ref);
    if (FAILED(hr)) {
        std::cerr << "[Error][Grabber::getColorData] Unable to get color frame reference."
            << std::endl;
        return E_FAIL;
    }

    // Get color frame
    IColorFrame* color_frame;
    hr = color_frame_ref->AcquireFrame(&color_frame);
    if (SUCCEEDED(hr)) {
        
        // Frame properties
        int nWidth_c = 0;
        int nHeight_c = 0;
        UINT nBufferSize = 0;

        // Get frame descriptor
        IFrameDescription* pFrameDescription_c = NULL;
        hr = color_frame->get_FrameDescription(&pFrameDescription_c);

        if (SUCCEEDED(hr)) {
            pFrameDescription_c->get_Width(&nWidth_c);
            pFrameDescription_c->get_Height(&nHeight_c);
        }

        // Get color fromat
        ColorImageFormat imageFormat_c = ColorImageFormat_None;
        hr = color_frame->get_RawColorImageFormat(&imageFormat_c);

        // Get color data
        if (SUCCEEDED(hr)) {

            if (imageFormat_c == ColorImageFormat_Bgra) {
                hr = color_frame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&raw_color_RGBX));
                memcpy(aux_color_RGBX, raw_color_RGBX, cColorWidth * cColorHeight * sizeof(RGBQUAD));
                raw_color_RGBX = aux_color_RGBX;
            }
            else if (aux_color_RGBX) {
                raw_color_RGBX = aux_color_RGBX;
                nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
                hr = color_frame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(raw_color_RGBX), ColorImageFormat_Bgra);
            }
            else {
                hr = E_FAIL;
            }
        }

        // Post-process color data
        if (SUCCEEDED(hr)) {
            ProcessColor(raw_color_RGBX, nWidth_c, nHeight_c);
        }

        SafeRelease(pFrameDescription_c);
     }
    else {
        std::cerr << "[Error][Grabber::getColorData] Unable to get color frame."
                  << std::endl;
    }

    
    // Release frame resources
    SafeRelease(color_frame);
    SafeRelease(color_frame_ref);

    return hr;
} /* Grabber::getColorData() */


/**
 * @brief  Grabs and stores the IR (infrared) frame
 *
 * @param frame  Multi-source pointer to the current frame
 *
 * @returns  Indicates success or failure
 */
HRESULT Grabber::getInfraredData(IMultiSourceFrame* frame) {
    
    // Get color frame reference
    IInfraredFrameReference* ir_frame_ref = NULL;
    HRESULT hr = frame->get_InfraredFrameReference(&ir_frame_ref);
    if (FAILED(hr)) {
        std::cerr << "[Error][Grabber::getColorData] Unable to get infrared frame reference."
            << std::endl;
        return E_FAIL;
    }

    // Get color frame
    IInfraredFrame* ir_frame;
    hr = ir_frame_ref->AcquireFrame(&ir_frame);
    if (SUCCEEDED(hr)) {

        // Frame properties
        int nWidth = 0;
        int nHeight = 0;
        UINT nBufferSize = 0;

        // Get frame descriptor
        IFrameDescription* pFrameDescription = NULL;
        hr = ir_frame->get_FrameDescription(&pFrameDescription);

        if (SUCCEEDED(hr)) {
            hr = pFrameDescription->get_Width(&nWidth);
            hr = pFrameDescription->get_Height(&nHeight);
        }

        // Get infrared data
        if (SUCCEEDED(hr)) {
            hr = ir_frame->AccessUnderlyingBuffer(&nBufferSize, &raw_infrared_u16);
            memcpy(aux_infrared_u16, raw_infrared_u16, nBufferSize * sizeof(UINT16));
            raw_infrared_u16 = aux_infrared_u16;
        }


        // Post-process the infrared data
        if (SUCCEEDED(hr)) {
            ProcessInfrared(raw_infrared_u16, nWidth, nHeight);
        }

        SafeRelease(pFrameDescription);
    }
    
    // Release frame resources
    SafeRelease(ir_frame);
    SafeRelease(ir_frame_ref);

    return hr;
}


/**
 * @brief Post-process color data
 *
 * @param pBuffer  Color frame data buffer pointer
 * @param nWidth   Color frame width
 * @param nHeight  Color frame height
 */
void Grabber::ProcessColor(RGBQUAD* pBuffer, int nWidth, int nHeight) {

    // Color frame total size
    int nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);

    // Make sure we've received valid data
    if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
    {
        // Draw the data with Direct2D
        if (m_pDrawColor) {
            m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBuffer), cColorWidth * cColorHeight * sizeof(RGBQUAD));
        }

        // Store frame
        if (screenshot_color) {

            WCHAR szScreenshotPath[MAX_PATH];

            // Retrieve the path to My Photos
            GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath), L"color");

            // Write out the bitmap to disk
            HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(pBuffer), nWidth, nHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);

            WCHAR szStatusMessage[64 + MAX_PATH];
            if (SUCCEEDED(hr)) {
                // Set the status bar to show where the screenshot was saved
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Screenshot saved to %s", szScreenshotPath);
            }
            else {
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Failed to write screenshot to %s", szScreenshotPath);
            }
            SetStatusMessage(szStatusMessage, 5000, true);

            // toggle off so we don't save a screenshot again next frame
            screenshot_color = false;
        }
    }
} /* Grabber::ProcessColor */


/**
 * @brief Post-process detph data
 *
 * @param pBuffer    Detph frame data buffer pointer
 * @param nWidth     Detph frame width
 * @param nHeight    Detph frame height
 * @param nMinDepth  Minimum reliable distance
 * @param nMaxDepth  Maximum reliable distance
 */
void Grabber::ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth) {

	// Make sure we've received valid data
    if (depth_RGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight)) {

        // Map depth to higher resolution (1080p) color image
        /*if (m_pKinectMapper) {
            HRESULT hr = m_pKinectMapper->MapColorFrameToCameraSpace(cDepthWidth * cDepthHeight,
                                                                     (UINT16*)raw_depth_u16,
                                                                     cColorWidth * cColorHeight,
                                                                     depth_XYZ);
        }*/
        // Draw depth data
        RGBQUAD* depth_data = depth_RGBX;
        if (m_pDrawDepth) {
            // end pixel is start + width*height - 1
            const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

            size_t pBuffer_index = 0;
            while (pBuffer_index < (nWidth * nHeight)) {

                USHORT depth = pBuffer[pBuffer_index];

                // To convert to a byte, we're discarding the most-significant
                // rather than least-significant bits.
                // We're preserving detail, although the intensity will "wrap."
                // Values outside the reliable depth range are mapped to 0 (black).

                // Note: Using conditionals in this loop could degrade performance.
                // Consider using a lookup table instead when writing production code.
                BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

                depth_data->rgbRed = intensity;
                depth_data->rgbGreen = intensity;
                depth_data->rgbBlue = intensity;

                ++depth_data;
                ++pBuffer_index;
            }

            // Draw the data with Direct2D
            if (m_pDrawDepth) {
                m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(depth_RGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
            }
        }

        // Store depth data
		if (screenshot_depth) {

			WCHAR szScreenshotPath[MAX_PATH];

			// Retrieve the path to My Photos
			GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath), L"depth");

			// Write out the bitmap to disk
			HRESULT hr_D = SaveBitmapToFile(reinterpret_cast<BYTE*>(depth_RGBX), nWidth, nHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);

			WCHAR szStatusMessage[64 + MAX_PATH];
			if (SUCCEEDED(hr_D)) {
				// Set the status bar to show where the screenshot was saved
				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Screenshot saved to %s", szScreenshotPath);
			}
			else {
				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Failed to write screenshot to %s", szScreenshotPath);
			}
			SetStatusMessage(szStatusMessage, 5000, true);

			// toggle off so we don't save a screenshot again next frame
            screenshot_depth = false;
		}
	}
}


/**
* @brief Post-process infrared data
*
* @param pBuffer    Infrared frame (data buffer pointer)
* @param nWidth     Infrared frame width
* @param nHeight    Infrared frame height
*/
void Grabber::ProcessInfrared(const UINT16* pBuffer, int nWidth, int nHeight) {

    if (infrared_RGBX && pBuffer && (nWidth == cInfraredWidth) && (nHeight == cInfraredHeight)) {
        RGBQUAD* pDest = infrared_RGBX;

        // end pixel is start + width*height - 1
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

        while (pBuffer < pBufferEnd)
        {
            // normalize the incoming infrared data (ushort) to a float ranging from 
            // [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
            // 1. dividing the incoming value by the source maximum value
            float intensityRatio = static_cast<float>(*pBuffer) / InfraredSourceValueMaximum;

            // 2. dividing by the (average scene value * standard deviations)
            intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

            // 3. limiting the value to InfraredOutputValueMaximum
            intensityRatio = min(InfraredOutputValueMaximum, intensityRatio);

            // 4. limiting the lower value InfraredOutputValueMinimym
            intensityRatio = max(InfraredOutputValueMinimum, intensityRatio);

            // 5. converting the normalized value to a byte and using the result
            // as the RGB components required by the image
            byte intensity = static_cast<byte>(intensityRatio * 255.0f);
            pDest->rgbRed = intensity;
            pDest->rgbGreen = intensity;
            pDest->rgbBlue = intensity;

            ++pDest;
            ++pBuffer;
        }

        // Draw the data with Direct2D
        if (m_pDrawInfrared) {
            m_pDrawInfrared->Draw(reinterpret_cast<BYTE*>(infrared_RGBX), cInfraredWidth * cInfraredHeight * sizeof(RGBQUAD));
        }

        // Store infrared data
        if (screenshot_infrared) {

            WCHAR szScreenshotPath[MAX_PATH];

            // Retrieve the path to My Photos
            GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath), L"infrared");

            // Write out the bitmap to disk
            HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(infrared_RGBX), nWidth, nHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);

            WCHAR szStatusMessage[64 + MAX_PATH];
            if (SUCCEEDED(hr)) {
                // Set the status bar to show where the screenshot was saved
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Screenshot saved to %s", szScreenshotPath);
            }
            else {
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Failed to write screenshot to %s", szScreenshotPath);
            }

            SetStatusMessage(szStatusMessage, 5000, true);

            // toggle off so we don't save a screenshot again next frame
            screenshot_infrared = false;
        }
    }
}


/**
 * @brief Projects a point from image plane to the 3D world space
 *
 * @param (xp,yp)     Point in 2D fimage plane coordinates
 * @param zp          Depth of the 2D point
 * @param (xw,yw,zw)  Point in 3D world coordinates
 */
void Grabber::ConvertProjectiveToRealWorld(float xp, float yp, float zp,
                                           float &xw, float &yw, float &zw) {

    static const float fovWidth = (70.0 * PI / 180.0);// 1.0144686707507438;
    static const float fovHeight = (60.0 * PI / 180.0); // 0.78980943449644714;
    static const float fScaleX = tanf(fovWidth * 0.5f) * 2.0;
    static const float fScaleY = tanf(fovHeight * 0.5f) * 2.0;
    
    xw = (float)zp * ((float)xp / 1920.0f - 0.5f) * fScaleX;
    yw = (float)zp * ((float)yp / 1080.0f - 0.5f) * fScaleY;
    zw = (float)zp;
}


/**
* @brief Registers the 3 image planes(color, depth and temperature) at the
*        pixel level, using highest resolution.
*
* @returns True if registration succeed; false otherwise
*/
HRESULT Grabber::registerFrame() {

    // checks if valid data available
    if (!raw_depth_u16 || !raw_color_RGBX) {
        std::cerr << "[Error][Grabber::registerFrame] Empty depth or color frame."
                  << std::endl;
        return E_FAIL;
    }

    // maps depth to higher resolution RGB image
    if (!m_pKinectMapper) {
        std::cerr << "[Error][Grabber::registerFrame] Empty depth mapper."
            << std::endl;
        return E_FAIL;
    }
    HRESULT hr = m_pKinectMapper->MapColorFrameToCameraSpace(cDepthWidth * cDepthHeight,
                                                             (UINT16*)raw_depth_u16,
                                                             cColorWidth * cColorHeight,
                                                             depth_XYZ);
    if (SUCCEEDED(hr)) {

        // loop over output pixels
        float x, y, z;

        #pragma omp parallel for num_threads(4)
        for (size_t color_index = 0; color_index < kFRAME_SIZE; ++color_index) {

            // default setting source to copy from the background pixel
            CameraSpacePoint p = depth_XYZ[color_index];
            if (p.Z != std::numeric_limits<float>::infinity() &&
                p.Z != -std::numeric_limits<float>::infinity() &&
                static_cast<float>(p.Z) >= kMIN_DEPTH && static_cast<float>(p.Z) < kMAX_DEPTH) {

                float xp = color_index % cColorWidth;
                float yp = color_index / cColorWidth;
                float zp = static_cast<float>(p.Z);
                //float depthX = static_cast<float>(p.X);//changed by kamran 2015-1-7
                //float depthY = static_cast<float>(p.Y);//changed by kamran 2015-1-7
                //float depthZ = static_cast<float>(p.Z);//changed by kamran 2015-1-7

                // position
                ConvertProjectiveToRealWorld(xp, yp, zp, x, y, z);
                frame_points[color_index].position.x = x;
                frame_points[color_index].position.y = y;
                frame_points[color_index].position.z = z;
                // color (check the alignment)
                const RGBQUAD* pSrc = raw_color_RGBX + color_index;
                frame_points[color_index].color.x = pSrc->rgbRed;
                frame_points[color_index].color.y = pSrc->rgbGreen;
                frame_points[color_index].color.z = pSrc->rgbBlue;

                // new valid point
                valid_frame_points[color_index] = true;
            }
            else {
                // invalid point
                valid_frame_points[color_index] = false;
            }
        }
        
        // Calculate number of faces
        hr = CalculateSurfaceNormal();
        if (FAILED(hr)) {
            std::cerr << "[Error][Grabber::registerFrame] Unable to compute the surface normals."
                      << std::endl;
        }
    }

    return hr;
}


/**
* @brief Computes the surface normal for each valid point
*
* @returns S_OK on success, otherwise failure code.
*/
HRESULT Grabber::CalculateSurfaceNormal() {

    // Locals
    vec3 v1, v2, v3;
    const int nSamplingRate = 2;
    const int nOffset = cColorWidth*nSamplingRate;
    // Calculate face normal
   // #pragma omp parallel for num_threads(8)

	concurrency::parallel_for(int (0),  kFRAME_SIZE - (nOffset + nSamplingRate),[&]( int i)
	{

        frame_points[i].normal.set(0, 0, 0);
        int index_v1 = i;
        int index_v2 = i + nOffset;
        int index_v3 = i + nSamplingRate;

        // is nIndex i ? no, id sampling rate is different
        
        // offsetted points are valid
        if (valid_frame_points[index_v1] &&
            valid_frame_points[index_v2] &&
            valid_frame_points[index_v3]) {

            // compute surface normal
            v1 = frame_points[index_v1].position;
            v2 = frame_points[index_v2].position;
            v3 = frame_points[index_v3].position;
            frame_points[index_v1].normal = normalize(cross(v2 - v1, v3 - v1));
            if (frame_points[index_v1].normal.x > 1 ||
                frame_points[index_v1].normal.y > 1 ||
                frame_points[index_v1].normal.z > 1) {
                printf("\n\n %f %f %f", frame_points[index_v1].normal.x, frame_points[index_v1].normal.y, frame_points[index_v1].normal.z);
            }
        }
});

    return S_OK;
}


/**
 * @brief Draw the results on screen.
 *
 * @param output_type  The type of information to be drawn on screen
 *
 * @returns S_OK on success, otherwise failure code.
 */
HRESULT Grabber::drawResults(OUTPUT_TYPE output_type) {

    // Check for valid render
    if (m_pDrawResult == NULL) {
        std::cerr << "[Error][Grabber::drawResult] Result render is not initialized."
                  << std::endl;
        return E_FAIL;
    }

    // Populate result buffer
    RGBQUAD* result_buff = result_RGBX;
    FrameDescriptor* frame_buff = frame_points;
    if (output_type == OUTPUT_COLOR) {
        #pragma omp parallel for num_threads(8)
        for (int i = 0; i < kFRAME_SIZE; i++) {
            if (valid_frame_points[i]) {
                result_buff->rgbRed = frame_buff->color.x;
                result_buff->rgbGreen = frame_buff->color.y;
                result_buff->rgbBlue = frame_buff->color.z;
            }
            else {
                result_buff->rgbRed = 0;
                result_buff->rgbGreen = 0;
                result_buff->rgbBlue = 0;
            }
            result_buff++;
            frame_buff++;
        }
    }
    else if (output_type == OUTPUT_DEPTH) {
        #pragma omp parallel for num_threads(8)
        for (int i = 0; i < kFRAME_SIZE; i++) {
            if (valid_frame_points[i]) {
                result_buff->rgbRed = fabsf(frame_buff->position.x * 130);
                result_buff->rgbGreen = fabsf(frame_buff->position.y * 130);
                result_buff->rgbBlue = fabsf(frame_buff->position.z * 130);
            }
            else {
                result_buff->rgbRed = 0;
                result_buff->rgbGreen = 0;
                result_buff->rgbBlue = 0;
            }
            result_buff++;
            frame_buff++;
        }
    }
    else if (output_type == OUTPUT_NORMAL) {
        #pragma omp parallel for num_threads(8)
        for (int i = 0; i < kFRAME_SIZE; i++) {
            if (valid_frame_points[i]) {
                result_buff->rgbRed = fabsf(frame_buff->normal.x*255.0);
                result_buff->rgbGreen = fabsf(frame_buff->normal.y*255.0);
                result_buff->rgbBlue = fabsf(frame_buff->normal.z*255.0);
            }
            else {
                result_buff->rgbRed = 0;
                result_buff->rgbGreen = 0;
                result_buff->rgbBlue = 0;
            }
            result_buff++;
            frame_buff++;
        }
    }
    m_pDrawResult->Draw(reinterpret_cast<BYTE*>(result_RGBX), cColorWidth * cColorHeight * sizeof(RGBQUAD));
    
    return S_OK;
}


/**
 * @brief Get the name of the file where screenshot will be stored.
 *
 * @param lpszFilePath   String buffer that will receive screenshot file name.
 * @param nFilePathSize  Number of characters in lpszFilePath string buffer.
 *
 * @returns S_OK on success, otherwise failure code.
 */
HRESULT Grabber::GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath,
                                       UINT nFilePathSize, WCHAR* imageTypeString) {

	WCHAR* pszKnownPath = NULL;
	HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);

	if (SUCCEEDED(hr)) {

		// Get the time
		WCHAR szTimeString[MAX_PATH];
		GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

		// File name will be KinectScreenshotColor-HH-MM-SS.bmp
		StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\KinectScreenshot-%s-%s.bmp", pszKnownPath, imageTypeString, szTimeString);
	}

	if (pszKnownPath) {
		CoTaskMemFree(pszKnownPath);
	}

	return hr;
}


/**
 * @brief Save passed in image data to disk as a bitmap.
 *
 * @param pBitmapBits    Image data to save.
 * @param lWidth         Width (in pixels) of input image data.
 * @param lHeight        Height (in pixels) of input image data.
 * @param wBitsPerPixel  Bits per pixel of image data.
 * @param lpszFilePath   Full file path to output bitmap.
 *
 * @returns indicates success or failure.
 */
HRESULT Grabber::SaveBitmapToFile(const BYTE* pBitmapBits,
                                  LONG lWidth, LONG lHeight,
                                  WORD wBitsPerPixel,
                                  LPCWSTR lpszFilePath)
{
	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = { 0 };

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes = 1;                         // Default
	bmpInfoHeader.biSizeImage = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = { 0 };

	bfh.bfType = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

	// Create the file on disk to write to
	HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	// Return if error opening file
	if (NULL == hFile)
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the RGB Data
	if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}


/**
* @brief Set the status bar message
*
* @param szMessage     Message to display
* @param showTimeMsec  Time in milliseconds to ignore future status messages
* @param bForce        Force status update
*
* @returns True if message was displayed; false otherwise
*/
bool Grabber::SetStatusMessage(_In_z_ WCHAR* szMessage,
                               DWORD nShowTimeMsec,
                               bool bForce) {

    // Next time available for message display 
    static INT64 m_nNextStatusTime = 0 ;
    
    // Current time 
    INT64 now = GetTickCount64();

    // Display message
    if (m_hWnd && (bForce || (m_nNextStatusTime <= now))) {

        SetDlgItemText(*m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

HRESULT Grabber::clustering() {

	cluster->set_frame(reinterpret_cast<Image_buffer*>(&frame_points[0]));
	cluster->set_mask(valid_frame_points);
	cluster->update();
	return NULL;
}