//============================================================================
// Name        : ir_grabber.cpp
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : IR sensor grabber implementation
//============================================================================


/////////////////////////////////////////////////////////////////////////////
///                            Includes                                    //
/////////////////////////////////////////////////////////////////////////////

// Module interface
#include "ir_grabber.h"

// C/C++
#include <string.h>
#include <winsock2.h>
#include <vector>

// Local Modules
#include "streamer.h"


///////////////////////////////////////////////////////////////////////////////
///                               IR Grbber                                 ///
///////////////////////////////////////////////////////////////////////////////

// Bind a function to the IR img grabber
boost::signals2::connection ir_grabber::connect(const ir_grabber::signal_t::slot_type &subscriber)
{
  return img_sig_.connect(subscriber);
}

// Start IR img grabber
 int ir_grabber::start()
{
  // Create communication with IR sensor
  if (!IRopenConnection(port_number_, ip_address_)) {
    printf("[Error][ir_grabber::start] Unable to open ethernet connection with the IR sensor.\n");
    return 0;
  }

  // Create grabbing thread
  do_grab_ = true;
  img_thread_ = boost::thread(&ir_grabber::run, this);

  // if everything is fine, return OKAY
  return 1;
}

// Stop IR img grabber
int ir_grabber::stop()
{
  // Flag stop grabbing
  do_grab_ = false;

  // Wait for thread to end
  img_thread_.join ();

  // Close IR connection
  if (!IRcloseConnection()) {
    printf("[Error][ir_grabber::stop] Unable to close ethernet connection with the IR sensor.\n");
    return 0;
  }

  // if everything fine, return OKAY
  return 1;
}

// IR img grabber run
void ir_grabber::run()
{
  // Message to send
  char msg[2];  // Header(MSG+Response=2)

  // Grab IR frames
  while(do_grab_) {
    // set message
    msg[0] = msg_pipe_[0][0];
    msg[1] = msg_pipe_[0][1];

    // Grab (ADD check for reconnection if needed)
    if ( !IRgrabFrame(msg, 2) ) continue;

    // New frame arrived
    img_id_++;
    if (msg_pipe_.size() > 1) msg_pipe_.erase(msg_pipe_.begin());
    img_sig_(img_buffer_u8_, img_buffer_u16_);
  }
}


///////////////////////////////////////////////////////////////////////////////
///                      IR Ethernet connection                             ///
///////////////////////////////////////////////////////////////////////////////

// Open IR connection
int ir_grabber::IRopenConnection(const int port_number, std::string ip_address)
{
  // Open connection
  socket_handle_ = IR_connect(port_number, ip_address);

  // Check if valid handle
  if (socket_handle_ == INVALID_SOCKET) return 0;

  // if everything fine, return OKAY
  return 1;
}

// Close IR connection
int ir_grabber::IRcloseConnection()
{
  // Shut down the transmision
  int res = IR_disconnect(socket_handle_);    

  return res;
}

// Get new frame from sensor
int ir_grabber::IRgrabFrame(char *msg, int msg_size)
{
  // Ask for a frame
  IR_send(socket_handle_, msg, 2);

  // Wait for the frame
  int res = IR_grab_img(socket_handle_, img_buffer_u8_, msg[1], img_buffer_u16_, img_width_, img_height_);

  if (res == -1) { // try reopen connection
    if (!IRopenConnection(port_number_, ip_address_))
      printf("[Error][ir_grabber::run::IRgrabFrame] Unable to re-open ethernet connection with the IR sensor.\n");
    return 0;
  }

  // if everything fine, return OKAY
  return res;
}

// Send I2C command to sensor
int ir_grabber::IRi2cCommand(char* msg, int msg_size)
{
  return 0;
}


///////////////////////////////////////////////////////////////////////////////
///                                Others                                   ///
///////////////////////////////////////////////////////////////////////////////

// Get current IR img id
const int ir_grabber::getImgID()
{
  return img_id_;
}

// Set message to be sent
void ir_grabber::setMessage(char msg[2])
{
  std::vector<char> new_msg;
  new_msg.push_back(msg[0]);
  new_msg.push_back(msg[1]);
  msg_pipe_.push_back(new_msg);
}