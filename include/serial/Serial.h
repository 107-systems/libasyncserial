/**
 * \author Alexander Entinger, MSc
 */

#ifndef SERIAL_H_
#define SERIAL_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include <string>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace serial
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Serial
{

public:

   Serial(std::string const & dev_node, size_t const baud_rate);
  ~Serial();


  typedef boost::function<void(const char*, size_t)> onSerialDataReceivedCallback;

  void    registerOnSerialDataReceivedCallback(onSerialDataReceivedCallback serial_data_received_callback_func);
  void    open                                ();
  size_t  send                                (uint8_t const * buffer, size_t const bytes);
  void    close                               ();

private:

  static size_t const RECEIVE_BUFFER_SIZE = 32;

  std::string                   _dev_node;
  size_t                        _baud_rate;

  boost::asio::io_service       _io_service;
  boost::asio::serial_port      _serial_port;

  boost::thread                 _io_service_thread;

  char                        	_receive_buffer[RECEIVE_BUFFER_SIZE];
  onSerialDataReceivedCallback 	_serial_data_received_callback_func;

  void read   ();
  void readEnd(boost::system::error_code const & error, size_t bytes_transferred);

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* serial */

#endif /* SERIAL_H_ */

