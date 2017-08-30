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

#include <mutex>
#include <future>
#include <string>
#include <vector>
#include <thread>
#include <condition_variable>

#include <boost/asio.hpp>

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

   Serial();
  ~Serial();

  void                              open    (std::string          const & dev_node, size_t const baud_rate);
  void                              transmit(std::vector<uint8_t> const & data                            );
  std::future<std::vector<uint8_t>> receive (size_t               const   num_bytes                       );
  void                              close   (                                                             );

private:

  static size_t const RECEIVE_BUFFER_SIZE = 32;

  uint8_t                       _asio_rx_buffer[RECEIVE_BUFFER_SIZE];

  boost::asio::io_service       _io_service;
  boost::asio::serial_port      _serial_port;

  std::thread                   _io_service_thread;

  std::mutex                    _mutex;
  std::condition_variable       _condition;
  std::vector<uint8_t>          _rx_buffer;

  void                 read   (                                                                         );
  void                 readEnd(boost::system::error_code const & error,         size_t bytes_transferred);
  void                 push   (std::vector<uint8_t>      const & received_data                          );
  std::vector<uint8_t> pop    (size_t                    const   num_bytes                              );

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* serial */

#endif /* SERIAL_H_ */

