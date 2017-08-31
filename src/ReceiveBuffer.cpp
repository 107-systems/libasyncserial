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

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <serial/ReceiveBuffer.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace serial
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ReceiveBuffer::push(std::vector<uint8_t> const & received_data)
{
  std::unique_lock<std::mutex> lock(_mutex);

  _rx_buffer.insert(std::end  (_rx_buffer   ),
                    std::begin(received_data),
                    std::end  (received_data));

  _condition.notify_all();
}

std::vector<uint8_t> ReceiveBuffer::pop(size_t const num_bytes)
{
  std::unique_lock<std::mutex> lock(_mutex);

  _condition.wait( lock, [this, num_bytes]{ return (_rx_buffer.size() >= num_bytes); } );

  std::vector<uint8_t> data;

  data.insert(std::begin(data      ),
              std::begin(_rx_buffer),
              std::begin(_rx_buffer) + num_bytes);

  _rx_buffer.erase(std::begin(_rx_buffer),
                   std::begin(_rx_buffer) + num_bytes);

  return data;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* serial */

