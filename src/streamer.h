//============================================================================
// Name        : streamer.h
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Main types and defines for the streamer application
//============================================================================

#ifndef _STREAMER_H_
#define _STREAMER_H_

/////////////////////////////////////////////////////////////////////////////
///                            Includes                                    //
/////////////////////////////////////////////////////////////////////////////

// C/C++ Includes
#include <winsock2.h>
#include <string>


/////////////////////////////////////////////////////////////////////////////
///                      Typedefs and Defines                              //
/////////////////////////////////////////////////////////////////////////////

// Client message
enum STREAMER_MSG {	FRAME_REQUEST,	// Request a frame [IN/OUT]
					I2C_CMD,		// Sends an I2C command [IN/OUT]
					UNKNOWN_MSG		// Response msg for an unknown [OUT]
				};

// Server response
enum SERVER_RESP {	FRAME_READY,	// The response contains a valid frame
					NO_FRAME,		// No frame ready to be read [We can avoid this by repeating last frame]
					I2C_SUCCEED,	// I2C command was applied with success
          I2C_FAILED,	// I2C command failed
					RESEND			// If the client message was something unknown, ask for a resend
				};

// I2C commands
enum I2C_CMD {	RESET,		// Sensor reset
        FFC,              // Run flat filed corection
        SENSOR_TEMP_K,    // Get sensor temperature
				NO_CMD		// No command (just for frame request msg)
		     };

// Frame type
enum FRAME_TYPE {	U8,		// 8 bit per pixel
				    U16		// 16 bit per pixel
		        };

// Debug option
#define DPRINT //printf


/////////////////////////////////////////////////////////////////////////////
///                  IR connection lib interface                           //
/////////////////////////////////////////////////////////////////////////////

// Close connection
int IR_disconnect(SOCKET soc);

// Open Connection
SOCKET IR_connect(const int portNumber, std::string ip_address);

// Send data
void IR_send(SOCKET socketHandle, char msg[2], int msg_size);

// Receive data
int IR_recv(SOCKET socketHandle, char *buffer, const int pckg_size);

// Receive a frame
int IR_grab_img(SOCKET socketHandle, unsigned char *ir_img, char pixel_depth,
                unsigned short *ir_u16, const int w, const int h);

#endif
