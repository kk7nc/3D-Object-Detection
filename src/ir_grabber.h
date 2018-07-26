//============================================================================
// Name        : ir_grabber.h
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : IR sensor grabber interface
//============================================================================

#ifndef IR_GRABBER_H
#define IR_GRABBER_H


/////////////////////////////////////////////////////////////////////////////
///                            Includes                                    //
/////////////////////////////////////////////////////////////////////////////

// C/C++
#include <string.h>
#include <winsock2.h>

// Boost
#include <boost/format.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>

// Local modules
#include "streamer.h"


/////////////////////////////////////////////////////////////////////////////
///                 ir_grabber class definition                            //
/////////////////////////////////////////////////////////////////////////////
class ir_grabber
{
public:
    // Signal type for Callback function
    typedef boost::signals2::signal<void (unsigned char*, unsigned short*)>  signal_t;
    
    // IR image properties (This is what we recive as raw data from camera: Flir Lepton One)
    static const int  img_width_  = 80;
    static const int  img_height_ = 60;
    
    // Grabber constructor/destructor
    ir_grabber() { 
      img_id_ = 0; 
      do_grab_ = false;
      port_number_ = 5995;
      sensor_temp_ = -1;
      save_sensor_temp_ = false;
      //ip_address_ = "161.253.66.95";
      ip_address_ = "169.254.20.203";

      // message to be sent 
      std::vector<char> new_msg;
      new_msg.push_back(FRAME_REQUEST);
      new_msg.push_back(U8);
      msg_pipe_.push_back(new_msg);
    }
    ~ir_grabber() {}

    // Connecte callback to the IR grabber
    boost::signals2::connection connect(const signal_t::slot_type &subscriber);
    // Start IR grabber
    int start();
    // Stop IR grabber
    int stop();
    // Grabbing thread
    void run();
    // Get latest image ID
    const int getImgID();
    // Set message
    void setMessage(char msg[2]);

private:
    // Open IR connection
    int IRopenConnection(const int port_number, std::string ip_address);
    // Close IR connection
    int IRcloseConnection();
    // Get new frame from sensor
    int IRgrabFrame(char* msg, int msg_size);
    // Send I2C command to sensor
    int IRi2cCommand(char* msg, int msg_size);

    // Grabber internal flags and vars
    bool              do_grab_;       // used to trigger and stop the grabbing thread
    signal_t          img_sig_;       // grabber callback signal
    int               img_id_;        // current image id(number)
    boost::thread     img_thread_;    // grabbing thread
    unsigned char     img_buffer_u8_[4 + img_width_*img_height_]; // IR image buffer
    unsigned short    img_buffer_u16_[img_width_*img_height_]; // IR image buffer
    SOCKET            socket_handle_; // IR ethernet connection handle
    int               port_number_;   // port number used to connect the IR sensor
    int               sensor_temp_;   // camera sensor temperature
    bool              save_sensor_temp_;  // flag for signaling another line for the calibration file
    FILE              *log;           // Log file for sensor calibration; format HH MM SS TEMP IR_IMG
    std::string       ip_address_;    // ip address of the IR sensor (raspberryPI)
    std::vector<std::vector<char>> msg_pipe_;

};

#endif // IR_GRABBER_H