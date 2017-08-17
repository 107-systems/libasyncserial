/**
 * libserial provides an easy to use C++ interface for the serial interface using boost::asio.
 * Copyright (C) 2017 Alexander Entinger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

  typedef enum
  {
    None,
    Software,
    Hardware
  } eFlowControlSelect;

   Serial(std::string const & dev_node, size_t const baud_rate, eFlowControlSelect const flow_control);
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
  eFlowControlSelect            _flow_control;

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

