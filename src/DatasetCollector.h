//============================================================================
// Name        : DatasetCollector.h
// Author      : CosmaC
// Date        : March, 2016
// Copyright   : GWU Research
// Description : Dataset grabbing core
//============================================================================

#pragma once

// SensorFusion
#include "resource.h"
#include "ImageRenderer.h"
#include "Grabber.h"

// C/C++
#include <time.h>

// Compute processing time macros
#ifndef SYSOUT_F
#define SYSOUT_F(f, ...)      _RPT1( 0, f, __VA_ARGS__ ) // For Visual studio
#endif
#ifndef speedtest__             
#define speedtest__(data)   for (long blockTime = NULL; (blockTime == NULL ? (blockTime = clock()) != NULL : false); SYSOUT_F(data "%.9fs", (double) (clock() - blockTime) / CLOCKS_PER_SEC))
#endif


class DatasetCollector {

public:

    /**
     * @brief Class constructor
     */
    DatasetCollector(bool viewer_enable_);


    /**
     * @brief Class destructor
     */
	~DatasetCollector();


    /**
     * @brief Creates the main window and begins processing
     *
     * @param hInstance  Handle to the application instance
     * @param nCmdShow   Whether to display minimized, maximized, or normally
     *
     * @returns Result fina message
     */
	int Run(HINSTANCE hInstance, int nCmdShow);


private:

    // Current Kinect device
    Grabber*        kinect_grabber;

    // Viewer
	HWND            m_hWnd;
    bool            viewer_enable;
    int             posX, posY;

    // Direct2D
    ImageRenderer*  m_pDrawColor;
    ImageRenderer*  m_pDrawInfrared;
    ImageRenderer*  m_pDrawDepth;
    ImageRenderer*  m_pDrawResult;
    ID2D1Factory*   m_pD2DFactory;

    // Timer
	INT64           m_nNextStatusTime;
	

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
    static LRESULT CALLBACK MessageRouter(HWND hWnd, 
                                          UINT uMsg,
                                          WPARAM wParam,
                                          LPARAM lParam);

    
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
    LRESULT CALLBACK DlgProc(HWND hWnd,
                             UINT uMsg,
                             WPARAM wParam,
                             LPARAM lParam);


    /**
     * @brief Set the status bar message
     *
     * @param szMessage     Message to display
     * @param showTimeMsec  Time in milliseconds to ignore future status messages
     * @param bForce        Force status update
     *
     * @returns True if message was displayed; false otherwise
     */
	bool SetStatusMessage(_In_z_ WCHAR* szMessage,
                          DWORD nShowTimeMsec,
                          bool bForce);

};

