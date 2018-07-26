//============================================================================
// Name        : Grabber.h
// Author      : CosmaC
// Date        : March, 2016
// Copyright   : GWU Research
// Description : Kinect V2 + Flir Lepton grabber
//============================================================================

#pragma once

#include "ImageRenderer.h"
#include "vec3.h"
#include "Clustering.h"

// Windows
#include <Kinect.h>

// C/C++
#include <string>
#include <vector>



// Frame descriptor
struct FrameDescriptor {
    vec3 position;                      // Point 3D world position
    vec3 color;                         // Point color
    vec3 normal;                        // Point normal (orientation)
    //std::vector<float> fuzzy_labels;    // Point fuzzy labels
	float fuzzy_labels[4];    // Point fuzzy labels
    /// ????
    int label;
    vec3 color_buffer;
};


// Output types
enum OUTPUT_TYPE {
    OUTPUT_COLOR,
    OUTPUT_DEPTH,
    OUTPUT_NORMAL,
    OUTPUT_CLUSTER
};


// Kinect custom grabber class
class Grabber {

public:

    /**
     * @brief Grabber constructor
     */
    Grabber();

    
    /**
     * @brief Grabber destructor
     */
	~Grabber();


    /**
     * @brief  Initializes the default Kinect sensor
     *
     * @returns  Indicates success or failure
     */
    HRESULT init();
    

    /**
     * @brief  Updates current frame
     *
     * @returns  Indicates success or failure
     */
    HRESULT update();
    
    
    /**
     * @brief Registers the 3 image planes(color, depth and temperature) at the
     *        pixel level, using highest resolution.
     *
     * @returns True if registration succeed; false otherwise
     */
    HRESULT registerFrame();
	HRESULT clustering();
    
    /**
     * @brief Draw the results on screen.
     *
     * @param output_type  The type of information to be drawn on screen
     *
     * @returns True if registration succeed; false otherwise
    */
    HRESULT drawResults(OUTPUT_TYPE output_type);


    /**
     * @brief  Sets the scrrenshot flag for the color image
     *
     * @param do_screenshot  Screenshot flag value
     */
    inline void set_screenshot_color(bool do_screenshot) {
        screenshot_color = do_screenshot;
    }
    

    /**
     * @brief  Sets the scrrenshot flag for the depthimage
     *
     * @param do_screenshot  Screenshot flag value
     */
    inline void set_screenshot_depth(bool do_screenshot) {
        screenshot_depth = do_screenshot;
    }
    

    /**
     * @brief  Sets the scrrenshot flag for all images
     *
     * @param do_screenshot  Screenshot flag value
     */
    inline void set_screenshot(bool do_screenshot) {
        screenshot_depth = do_screenshot;
        screenshot_color = do_screenshot;
        screenshot_infrared = do_screenshot;
    }


    /**
     * @brief  Sets the image render for the color image
     *
     * @param render  Image render pre-assigned to a window
     */
    inline void set_color_render(ImageRenderer* render) {
        m_pDrawColor = render;
    }


    /**
     * @brief  Sets the image render for the depth image
     *
     * @param render  Image render pre-assigned to a window
     */
    inline void set_depth_render(ImageRenderer* render) {
        m_pDrawDepth = render;
    }


    /**
     * @brief  Sets the image render for the infrared image
     *
     * @param render  Image render pre-assigned to a window
     */
    inline void set_infrared_render(ImageRenderer* render) {
        m_pDrawInfrared = render;
    }


    /**
     * @brief  Sets the image render for the result image
     *
     * @param render  Image render pre-assigned to a window
     */
    inline void set_result_render(ImageRenderer* render) {
        m_pDrawResult = render;
    }


    /**
     * @brief  Sets the rendering window
     *
     * @param m_hWnd_  Redering window
     */
    inline void set_render_window(HWND* m_hWnd_) {
        m_hWnd = m_hWnd_;
    }


    // Frame default properties
    static const int cColorWidth = 1920;
    static const int cColorHeight = 1080;
    static const int cDepthWidth = 512;
    static const int cDepthHeight = 424;
    static const int cInfraredWidth = 512;
    static const int cInfraredHeight = 424;
    static const int kFRAME_SIZE = cColorWidth*cColorHeight;
    const float kMIN_DEPTH = 0.5f;    // ???? input maybe ????
    const float kMAX_DEPTH = 2.69f;   // ???? input maybe ????

private:

	// Kinect driver and frame rider
	IKinectSensor*           m_pKinectSensor;   // Sensor driver
    IMultiSourceFrameReader* m_pKinectReader;   // Kinect frame grabber
    ICoordinateMapper*       m_pKinectMapper;   // Converts between depth, color, and 3d coordinates
    FrameDescriptor          frame_points[kFRAME_SIZE]; // Registered image planes of the current frame
    bool                     valid_frame_points[kFRAME_SIZE]; // Mask for valid points of the frame

	// Color buffers
    RGBQUAD*        aux_color_RGBX;     // Pre-allocated RGBX frame 
    RGBQUAD*        raw_color_RGBX;     // Raw RGB data buffer
    bool            screenshot_color;   // Trigger for color image save
                                        // Color buffers
    // IR buffers
    RGBQUAD*        infrared_RGBX;      // Post-processed(normalized) infrared buffer
    UINT16*         aux_infrared_u16;   // Pre-allocated UINT16 frame 
    UINT16*         raw_infrared_u16;   // Raw infrared data buffer
    bool            screenshot_infrared;// Trigger for color image save

	// Depth buffers
    RGBQUAD*          depth_RGBX;         // Post-processed(normalized) z-buffer
    UINT16*           aux_depth_u16;      // Pre-allocated UINT16 frame 
    UINT16*           raw_depth_u16;      // Raw depth(z) data buffer
    CameraSpacePoint* depth_XYZ;          // Depth frame projected into XYZ space (camera space)
    bool              screenshot_depth;   // Trigger for depth image save

    // Results buffers
    RGBQUAD*          result_RGBX;        // Result to be showed on screen
    
    // Render
    ImageRenderer*  m_pDrawColor;       // Color render pointer
    ImageRenderer*  m_pDrawInfrared;    // Infrared render pointer
    ImageRenderer*  m_pDrawDepth;       // Depth render pointer
    ImageRenderer*  m_pDrawResult;      // Result render pointer
    HWND*           m_hWnd;             // Rendering window

	// Clustering
	Clustering* cluster;
    /**
     * @brief  Grabs and stores the depth frame
     *
     * @param frame  Multi-source pointer to the current frame
     *
     * @returns  Indicates success or failure
     */
    HRESULT getDepthData(IMultiSourceFrame* frame);
    

    /**
     * @brief  Grabs and stores the RGB (color) frame
     *
     * @param frame  Multi-source pointer to the current frame
     *
     * @returns  Indicates success or failure
     */
    HRESULT getColorData(IMultiSourceFrame* frame);

    
    /**
    * @brief  Grabs and stores the IR (infrared) frame
    *
    * @param frame  Multi-source pointer to the current frame
    *
    * @returns  Indicates success or failure
    */
    HRESULT getInfraredData(IMultiSourceFrame* frame);


    /**
     * @brief Post-process color data
     *
     * @param pBuffer  Color frame data buffer pointer
     * @param nWidth   Color frame width
     * @param nHeight  Color frame height
     */
    void ProcessColor(RGBQUAD* pBuffer,
                      int nWidth, int nHeight);
    

    /**
     * @brief Post-process detph data
     *
     * @param pBuffer    Detph frame data buffer pointer
     * @param nWidth     Detph frame width
     * @param nHeight    Detph frame height
     * @param nMinDepth  Minimum reliable distance
     * @param nMaxDepth  Maximum reliable distance
     */
    void ProcessDepth(const UINT16* pBuffer,
                      int nHeight, int nWidth,
                      USHORT nMinDepth, USHORT nMaxDepth);


    /**
     * @brief Post-process infrared data
     *
     * @param pBuffer    Infrared frame (data buffer pointer)
     * @param nWidth     Infrared frame width
     * @param nHeight    Infrared frame height
     */
    void ProcessInfrared(const UINT16* pBuffer,
                         int nWidth, int nHeight);


    /**
     * @brief Projects a point from image plane to the 3D world space
     *
     * @param (xp,yp)     Point in 2D fimage plane coordinates
     * @param zp          Depth of the 2D point
     * @param (xw,yw,zw)  Point in 3D world coordinates
     */
    void ConvertProjectiveToRealWorld(float xp, float yp, float zp,
                                      float &xw, float &yw, float &zw);
    
    
    /**
     * @brief Computes the surface normal for each valid point
     *
     * @returns S_OK on success, otherwise failure code.
     */
    HRESULT CalculateSurfaceNormal();

    /**
     * @brief Get the name of the file where screenshot will be stored.
     *
     * @param lpszFilePath     String buffer that will receive screenshot file name.
     * @param nFilePathSize    Number of characters in lpszFilePath string buffer.
     * @param imageTypeString  Image type to be included in the name (depth, color or infrared)
     *
     * @returns S_OK on success, otherwise failure code.
     */
    HRESULT GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath,
                                  UINT nFilePathSize, WCHAR* imageTypeString);


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
    HRESULT SaveBitmapToFile(const BYTE* pBitmapBits,
                             LONG lWidth, LONG lHeight,
                             WORD wBitsPerPixel,
                             LPCWSTR lpszFilePath);


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


    // InfraredSourceValueMaximum is the highest value that can be returned in the InfraredFrame.
    // It is cast to a float for readability in the visualization code.
    const float InfraredSourceValueMaximum = static_cast<float>(USHRT_MAX);

    // The InfraredOutputValueMinimum value is used to set the lower limit, post processing, of the
    // infrared data that we will render.
    // Increasing or decreasing this value sets a brightness "wall" either closer or further away.
    const float InfraredOutputValueMinimum = 0.01f;

    // The InfraredOutputValueMaximum value is the upper limit, post processing, of the
    // infrared data that we will render.
    const float InfraredOutputValueMaximum = 1.0f;

    // The InfraredSceneValueAverage value specifies the average infrared value of the scene.
    // This value was selected by analyzing the average pixel intensity for a given scene.
    // Depending on the visualization requirements for a given application, this value can be
    // hard coded, as was done here, or calculated by averaging the intensity for each pixel prior
    // to rendering.
    const float InfraredSceneValueAverage = 0.08f;

    // The InfraredSceneStandardDeviations value specifies the number of standard deviations
    // to apply to InfraredSceneValueAverage. This value was selected by analyzing data
    // from a given scene.
    // Depending on the visualization requirements for a given application, this value can be
    // hard coded, as was done here, or calculated at runtime.
    const float InfraredSceneStandardDeviations = 3.0f;
};

