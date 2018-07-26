//============================================================================
// Name        : DatasetCollector.cpp
// Author      : CosmaC
// Date        : March, 2016
// Copyright   : GWU Research
// Description : Dataset grabbing core
//============================================================================


#pragma warning(push)
#define _AFXDLL


// SensorFusion
#include "DatasetCollector.h"
#include "Grabber.h"
#include "resource.h"
#include "vec3.h"
#include "stdafx.h"

// Windows
#include <Kinect.h>
#include <Wincodec.h>
#include <strsafe.h>
#include <Windows.h>

// C/C++
#include <vector>
#include <thread>
#include <iostream>
#include <ppl.h>
#include <omp.h>
#include <time.h>


/**
 * @brief Entry point for the application
 *
 * @param hInstance      Handle to the application instance
 * @param hPrevInstance  Always 0
 * @param lpCmdLine      Command line arguments
 * @param nCmdShow       Whether to display minimized, maximized, or normally
 *
 * @returns status
 */
int APIENTRY wWinMain(
	_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR lpCmdLine,
	_In_ int nShowCmd) {

	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

    DatasetCollector application(true);
	application.Run(hInstance, nShowCmd);
}


/**
 * @brief Class constructor
 */
DatasetCollector::DatasetCollector(bool viewer_enable_) :
m_hWnd(NULL),
m_nNextStatusTime(0LL),
m_pD2DFactory(NULL),
m_pDrawColor(NULL),
m_pDrawInfrared(NULL),
m_pDrawDepth(NULL),
m_pDrawResult(NULL) {

    viewer_enable = viewer_enable_;
}


/**
 * @brief Class destructor
 */
DatasetCollector::~DatasetCollector() {

	// clean up Direct2D renderer
	if (m_pDrawColor) {
		delete m_pDrawColor;
		m_pDrawColor = NULL;
	}

	if (m_pDrawDepth) {
		delete m_pDrawDepth;
		m_pDrawDepth = NULL;
	}

    if (m_pDrawInfrared) {
        delete m_pDrawInfrared;
        m_pDrawInfrared = NULL;
    }

    if (m_pDrawResult) {
        delete m_pDrawResult;
        m_pDrawResult = NULL;
    }

    // clean up Direct2D
	SafeRelease(m_pD2DFactory);
}


/**
 * @brief Creates the main window and begins processing
 *
 * @param hInstance  Handle to the application instance
 * @param nCmdShow   Whether to display minimized, maximized, or normally
 *
 * @returns Result fina message
 */
int DatasetCollector::Run(HINSTANCE hInstance, int nCmdShow) {

    // Create kinect grabber
    kinect_grabber = new Grabber();
    kinect_grabber->init();

	// Dialog custom window class
    MSG       msg = { 0 };
    WNDCLASS  wc;
    ZeroMemory(&wc, sizeof(wc));
	wc.style         = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra    = DLGWINDOWEXTRA;
	wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc   = DefDlgProcW;
	wc.lpszClassName = L"ColorBasicsAppDlgWndClass";

	if (!RegisterClassW(&wc)) {
		return 0;
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		                NULL,
		                MAKEINTRESOURCE(IDD_APP),
		                NULL,
		                (DLGPROC)DatasetCollector::MessageRouter,
		                reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, nCmdShow);

	// Main message loop
    clock_t tStart = clock();
    size_t frame_count = 0;
    #pragma loop(hint_parallel(32))
    while (WM_QUIT != msg.message) {

        // Get new frame
        HRESULT hr = kinect_grabber->update();
        if (SUCCEEDED(hr)) {
            // Processed new frame
            frame_count++;
            hr = kinect_grabber->registerFrame();
            hr = kinect_grabber->clustering();
            if (SUCCEEDED(hr)) {
                //kinect_grabber->clusterFrame();
                kinect_grabber->drawResults(OUTPUT_NORMAL);
            }
        }

        // Process messages
		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE)) {

			// If a dialog message will be taken care of by the dialog proc
			if (hWndApp && IsDialogMessageW(hWndApp, &msg)) {
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}

        clock_t tEnd = clock();
        double tDuration = (double)(tEnd - tStart) / CLOCKS_PER_SEC;
        if (tDuration > 1.0) {
            double fps = (double)frame_count / tDuration;
            tStart = tEnd;
            frame_count = 0;

            WCHAR szStatusMessage[50];
            StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"  FPS = %0.2f", fps);
            SetStatusMessage(szStatusMessage, 1000, false);
        }
	}

	return static_cast<int>(msg.wParam);
}


/**
 * @brief Handles window messages, passes most to the class instance to handle
 *
 * @param hWnd    Window message is for
 * @param uMsg    Message
 * @param wParam  Message data
 * @param lParam  Additional message data
 *
 * @returns Result of message processing
 */
LRESULT CALLBACK DatasetCollector::MessageRouter(HWND hWnd,
                                                 UINT uMsg,
                                                 WPARAM wParam,
                                                 LPARAM lParam)
{
    DatasetCollector* pThis = NULL;

	if (WM_INITDIALOG == uMsg) {
		pThis = reinterpret_cast<DatasetCollector*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else {
		pThis = reinterpret_cast<DatasetCollector*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis) {
		return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}


/**
 * @brief Handle windows messages for the class instance
 *
 * @param hWnd    Window message is for
 * @param uMsg    Message
 * @param wParam  Message data
 * @param lParam  Additional message data
 *
 * @returns Result of message processing
 */
LRESULT CALLBACK DatasetCollector::DlgProc(HWND hWnd,
                                           UINT message,
                                           WPARAM wParam,
                                           LPARAM lParam) {

	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);
	
	switch (message) {

    	case WM_INITDIALOG:
	    {
		    // Bind application window handle
		    m_hWnd = hWnd;

		    // Init Direct2D
		    D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

		    // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
		    // We'll use this to draw the color data we receive from the Kinect to the screen
		    m_pDrawColor = new ImageRenderer();
		    HRESULT hr = m_pDrawColor->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW_RGB),
                                                  m_pD2DFactory,
                                                  kinect_grabber->cColorWidth,
                                                  kinect_grabber->cColorHeight,
                                                  kinect_grabber->cColorWidth * sizeof(RGBQUAD));
		    if (FAILED(hr)) {
			    SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		    }

		    // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
		    // We'll use this to draw the depth data we receive from the Kinect to the screen
		    m_pDrawDepth = new ImageRenderer();
		    hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW_DEPTH),
                                          m_pD2DFactory,
                                          kinect_grabber->cDepthWidth,
                                          kinect_grabber->cDepthHeight,
                                          kinect_grabber->cDepthWidth * sizeof(RGBQUAD));
		    if (FAILED(hr)) {
			    SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		    }

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the infrared data we receive from the Kinect to the screen
            m_pDrawInfrared = new ImageRenderer();
            hr = m_pDrawInfrared->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW_IR),
                                             m_pD2DFactory,
                                             kinect_grabber->cInfraredWidth,
                                             kinect_grabber->cInfraredHeight,
                                             kinect_grabber->cInfraredWidth * sizeof(RGBQUAD));
            if (FAILED(hr)) {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the result data
            m_pDrawResult = new ImageRenderer();
            hr = m_pDrawResult->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW),
                m_pD2DFactory,
                kinect_grabber->cColorWidth,
                kinect_grabber->cColorHeight,
                kinect_grabber->cColorWidth * sizeof(RGBQUAD));
            if (FAILED(hr)) {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

		    // Connect the renders to the grabber class
            if (viewer_enable) {
                kinect_grabber->set_color_render(m_pDrawColor);
                kinect_grabber->set_depth_render(m_pDrawDepth);
                kinect_grabber->set_infrared_render(m_pDrawInfrared);
                kinect_grabber->set_result_render(m_pDrawResult);
                kinect_grabber->set_render_window(&m_hWnd);
            }
	    }
		break;

		// If the titlebar X is clicked, destroy app
	    case WM_CLOSE:
		    DestroyWindow(hWnd);
		    break;
        
        // Left button clicked 
	    case WM_LBUTTONUP:
		     posX = LOWORD(lParam);
		     posY = HIWORD(lParam);	
	         break;

	    case WM_DESTROY:
		    // Quit the main message pump
		    PostQuitMessage(0);
		    break;

		// Handle button press
	    case WM_COMMAND:
		    // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
		    if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)) {
                kinect_grabber->set_screenshot(true);
		    }

		    if (IDC_BUTTON_Grabbing == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)) {
            }

		    break;
	}

	return FALSE;
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
bool DatasetCollector::SetStatusMessage(_In_z_ WCHAR* szMessage,
                                        DWORD nShowTimeMsec, 
                                        bool bForce) {

    // Current time 
	INT64 now = GetTickCount64();

    // Display message
	if (m_hWnd && (bForce || (m_nNextStatusTime <= now))) {

		SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
		m_nNextStatusTime = now + nShowTimeMsec;

		return true;
	}

	return false;
}