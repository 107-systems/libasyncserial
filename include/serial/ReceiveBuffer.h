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

#ifndef LIB_SERIAL_INCLUDE_SERIAL_RECEIVEBUFFER_H_
#define LIB_SERIAL_INCLUDE_SERIAL_RECEIVEBUFFER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include <mutex>
#include <vector>
#include <thread>
#include <condition_variable>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace serial
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ReceiveBuffer
{

public:

  void                 push   (std::vector<uint8_t>      const & received_data                          );
  std::vector<uint8_t> pop    (size_t                    const   num_bytes                              );

private:

  std::mutex              _mutex;
  std::condition_variable _condition;
  std::vector<uint8_t>    _rx_buffer;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* serial */

#endif /* LIB_SERIAL_INCLUDE_SERIAL_RECEIVEBUFFER_H_ */
