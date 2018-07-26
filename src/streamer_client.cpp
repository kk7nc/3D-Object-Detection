//============================================================================
// Name        : streamer_client.cpp
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Simple app for streaming video over the local network (TCP)
//============================================================================

#define WIN32_LEAN_AND_MEAN

/////////////////////////////////////////////////////////////////////////////
///                            Includes                                    //
/////////////////////////////////////////////////////////////////////////////

// Module interface
#include "streamer.h"

// C/C++
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <errno.h>
using namespace std;
#pragma comment(lib, "Ws2_32.lib")


int count(0);

// Close Connection
int IR_disconnect(SOCKET soc)
{
  // Shut down the transmision
  int res = shutdown(soc, SD_SEND);
  if (res == SOCKET_ERROR) {
      printf("[Error] shutdown() failed with error: %d\n", WSAGetLastError());
      WSACleanup();
      return 0;
  }  

  // Close the socket
  res = closesocket(soc);
  if (res == SOCKET_ERROR) {
      printf("[Error] closesocket() failed with error = %d\n", WSAGetLastError() );
      WSACleanup();
      return 0;
  }   

  // if everything fine, return OKAY
  return 1;
}

// Open Connection
SOCKET IR_connect(const int portNumber, string ip_address)
{
    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("[Error][IR_Connect] WSAStartup failed with error: %d\n", iResult);
        exit(EXIT_FAILURE);
    }

    // Create socket
    SOCKET socketHandle = INVALID_SOCKET;
    socketHandle = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(socketHandle == INVALID_SOCKET) {
        printf("[Error][IR_Connect] Error at socket(): %ld\n", WSAGetLastError());
        WSACleanup();
        exit(EXIT_FAILURE);
   }


    // Load system information into socket data structures
    struct sockaddr_in remoteSocketInfo;
    ZeroMemory( &remoteSocketInfo, sizeof(remoteSocketInfo) );
    remoteSocketInfo.sin_family = AF_INET;
    remoteSocketInfo.sin_port = htons((u_short)portNumber);
    remoteSocketInfo.sin_addr.S_un.S_addr = inet_addr(ip_address.c_str());

    // Establish the connection with the server
    iResult = connect( socketHandle, (SOCKADDR*) &remoteSocketInfo, (int)sizeof(remoteSocketInfo));
    if (iResult == SOCKET_ERROR) {
        printf("[Error][IR_Connect] Error at connect(): %ld\n", WSAGetLastError());
        closesocket(socketHandle);
        exit(EXIT_FAILURE);
    }

    // Connection created successful
    return socketHandle;
}

// Send a new request
void IR_send(SOCKET socketHandle, char msg[2], int msg_size)
{
    DPRINT("[%d]CLIENT -- SEND -- Sending message... ", ::count);
    int sd = send(socketHandle, msg, msg_size, 0);
    if (sd == -1) {
        cerr << "[" << ::count << "]CLIENT -- CONNECTION -- Lost." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        exit(EXIT_FAILURE);
    }
    DPRINT(" Message sent! \n");
}

// Receive next message
int IR_recv(SOCKET socketHandle, char *buffer, const int pckg_size)
{
    u_long data_size = 0;
    do
    { // wait for data to be available
        ioctlsocket(socketHandle, FIONREAD, &data_size);
        //cerr << "Error: " << data_size << endl;
    } while (data_size < pckg_size);
    int rc = recv(socketHandle, buffer, pckg_size, 0);
    DPRINT("[%d]CLIENT -- RECV -- Number of bytes read: %d \n", ::count, rc);

    return rc;
}

// Grab next image
// 1 - frame received
// 0 - unknown response
// -1 - connection lost
int IR_grab_img(SOCKET socketHandle, unsigned char *ir_img, char pixel_depth,
                unsigned short *ir_u16, const int w, const int h)
{
    // Frame definition
    int pckg_size = 2 + 4 + w * h * 2;// Header(MSG+CMD=2) + SensorTemp(4) + IR img(80*60*2)
    std::vector<char> pckg;
    pckg.resize(pckg_size);
    char* img = pckg.data();
    
    // Receive data from IR server
    int rc = IR_recv(socketHandle, img, pckg_size);

    // Check if connection is still open
    if (rc == -1) {
        cerr << "[" << ::count << "]CLIENT -- CONNECTION -- Lost." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        return -1;
    }

    // Check response header
    if (img[0] == FRAME_REQUEST) {
        DPRINT("[%d]CLIENT -- RECV -- FRAME_REQUEST response. \n", ::count);
        if (pixel_depth == U8) {
            memcpy(ir_img, (img + 2), w * h + 4);
        }
        else { // U16
            memcpy(ir_u16, (img + 6), 2 * w * h);
            ((int*)ir_img)[0] = ((int*)(img + 2))[0];
            //normalize(temp, ir_img, 0, 255, NORM_MINMAX, CV_8U);
            unsigned short minValue = 7600;
            unsigned short maxValue = 8100;
            double scale = 255.0 / (maxValue - minValue);
            unsigned short* u16_data = (unsigned short*)ir_u16;
            unsigned char* u8_data = ir_img+4;
            
            for (int l = 0; l < h; l++)
                for (int c = 0; c < w; c++) {
                    if (u16_data[l*w + c] <= minValue)
                        u8_data[l*w + c] = 0;
                    else if (u16_data[l*w + c] >= maxValue)
                        u8_data[l*w + c] = 255;
                    else
                        u8_data[l*w + c] = (unsigned char)((u16_data[l*w + c] - minValue) * scale);
                }
        }

        return 1;
    }
    else if (img[0] == I2C_CMD) {
        // set out frames to 0
        memset(ir_img, 0, w * h + 4);
        memset(ir_u16, 0, 2 * w * h);

        // Check if I2C cmd succeed
        if (img[1] == I2C_FAILED)
          return 0;

        // Check I2C cmd type
        if (1)
        {
          int *buf1 = (int *)(img+2);
          int *buf2 = (int *)(ir_img);
          // Save temp
          buf2[0] = buf1[0];
        }

        DPRINT("[%d]CLIENT -- RECV -- I2C_CMD response.", ::count);
        return 1;
    }
    else if (img[0] == UNKNOWN_MSG) {
        DPRINT("[%d]CLIENT -- Server did not recognize your request.", ::count);
        return 0;
    }
    else {
        DPRINT("[%d]CLIENT -- Unable to decode server message.", ::count);
        return 0;
    }
}


//// Main application
//int main()
//{
//    // Communication Config
//    const int portNumber = 5995;
//    string ip_address = "161.253.66.95";
//
//    // Connect to the server
//    SOCKET socketHandle = IR_connect(portNumber, ip_address);
//
//
//    // Send data
//    char msg[2];					// Header(MSG+Response=2)
//    Mat ir_img (60, 80, CV_8UC1);
//
//    while (1)
//    {
//
//    	////////////////////////////////////////////////////////////////////////
//    	///  Create Request
//    	////////////////////////////////////////////////////////////////////////
//        msg[0] = FRAME_REQUEST;
//        msg[1] = NO_CMD;
//
//    	////////////////////////////////////////////////////////////////////////
//    	///  Send Request
//    	////////////////////////////////////////////////////////////////////////
//        IR_send(socketHandle, msg, 2);
//
//        ////////////////////////////////////////////////////////////////////////
//        /// Receive response
//        ////////////////////////////////////////////////////////////////////////
//        IR_grab_img(socketHandle, ir_img);
//
//        ////////////////////////////////////////////////////////////////////////
//    	// Show image
//        ////////////////////////////////////////////////////////////////////////
//    	imshow("IR Img", ir_img);
//        cvWaitKey(10);
//
//        // Count frames
//        ::count++;
//    }
//
//    return 0;
//
//}
