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

#include <serial/Serial.h>

#include <iostream>

#include <boost/bind.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace serial
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Serial::Serial(std::string const & dev_node, size_t const baud_rate, eFlowControlSelect const flow_control)
: _serial_port  (_io_service ),
  _dev_node     (dev_node    ),
  _baud_rate    (baud_rate   ),
  _flow_control (flow_control)
{
  _io_service.post(boost::bind(&Serial::read, this));
}

Serial::~Serial()
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Serial::registerOnSerialDataReceivedCallback(onSerialDataReceivedCallback serial_data_received_callback_func)
{
  _serial_data_received_callback_func = serial_data_received_callback_func;
}

void Serial::open()
{
  try
  {
    _serial_port.open(_dev_node);

    _serial_port.set_option(boost::asio::serial_port_base::baud_rate      (_baud_rate));
    _serial_port.set_option(boost::asio::serial_port_base::character_size (8));

    switch(_flow_control)
    {
    case Software : _serial_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::software)); break;
    case Hardware : _serial_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::hardware)); break;
    case None     :
    default       : _serial_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::none    )); break;
    }

    _serial_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::none));
    _serial_port.set_option(boost::asio::serial_port_base::parity         (boost::asio::serial_port_base::parity::none));
    _serial_port.set_option(boost::asio::serial_port_base::stop_bits      (boost::asio::serial_port_base::stop_bits::one));


    boost::thread t(boost::bind(&boost::asio::io_service::run, &_io_service));
    _io_service_thread.swap(t);
  }
  catch(boost::exception const &e)
  {
    std::cerr << "Error TSerial::open" << std::endl << boost::diagnostic_information(e) << std::endl;
  }
}

size_t Serial::send(uint8_t const * buffer, size_t const bytes)
{
  return boost::asio::write(_serial_port, boost::asio::buffer(buffer, bytes));
}

void Serial::close()
{
  _serial_port.close();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Serial::read()
{
  _serial_port.async_read_some(boost::asio::buffer(_receive_buffer, RECEIVE_BUFFER_SIZE), boost::bind(&Serial::readEnd, this, _1, _2));

}

void Serial::readEnd(boost::system::error_code const & error, size_t bytes_transferred)
{
  if (!error)
  {
    if(_serial_data_received_callback_func)
    {
      _serial_data_received_callback_func(_receive_buffer, bytes_transferred);
    }

    read();
  }
}


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* serial */
