/**
 * This software is distributed under the terms of the LGPL v2.1.
 * Copyright (c) 2022 Alexander Entinger, LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libasyncserial/graphs/contributors.
 */

#ifndef ASYNCSERIAL_ASYNCSERIAL_HPP_
#define ASYNCSERIAL_ASYNCSERIAL_HPP_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include <mutex>
#include <future>
#include <vector>
#include <thread>
#include <string>
#include <condition_variable>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AsyncSerial
{

private:

  class ReceiveBuffer
  {

  public:

    void push(std::vector<uint8_t> const & received_data)
    {
      std::unique_lock<std::mutex> lock(_mutex);

      _rx_buffer.insert(std::end  (_rx_buffer   ),
                        std::begin(received_data),
                        std::end  (received_data));

      _condition.notify_all();
    }

    std::vector<uint8_t> pop(size_t const num_bytes)
    {
      std::unique_lock<std::mutex> lock(_mutex);

      _condition.wait(lock, [this, num_bytes]{ return (_rx_buffer.size() >= num_bytes); } );

      std::vector<uint8_t> data;

      data.insert(std::begin(data      ),
                  std::begin(_rx_buffer),
                  std::begin(_rx_buffer) + num_bytes);

      _rx_buffer.erase(std::begin(_rx_buffer),
                      std::begin(_rx_buffer) + num_bytes);

      return data;
    }


  private:

    std::mutex _mutex;
    std::condition_variable _condition;
    std::vector<uint8_t> _rx_buffer;

  };

public:

  AsyncSerial()
  : _asio_receive_buffer{0}
  , _io_service{}
  , _serial_port{_io_service}
  , _io_service_thread{}
  {
    _io_service.post(boost::bind(&AsyncSerial::read, this));
  }

  ~AsyncSerial()
  {
    _io_service_thread.join();
  }

  void open(std::string const & dev_node, size_t const baud_rate)
  {
    _serial_port.open(dev_node);

    _serial_port.set_option(boost::asio::serial_port_base::baud_rate      (baud_rate));
    _serial_port.set_option(boost::asio::serial_port_base::character_size (8));
    _serial_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::none));
    _serial_port.set_option(boost::asio::serial_port_base::parity         (boost::asio::serial_port_base::parity::none));
    _serial_port.set_option(boost::asio::serial_port_base::stop_bits      (boost::asio::serial_port_base::stop_bits::one));

    std::thread t(boost::bind(&boost::asio::io_service::run, &_io_service));
    _io_service_thread.swap(t);
  }

  void transmit(std::vector<uint8_t> const & data)
  {
    boost::asio::write(_serial_port, boost::asio::buffer(data));
  }

  std::future<std::vector<uint8_t>> receive(size_t const num_bytes)
  {
    return std::async(std::launch::deferred, boost::bind(&ReceiveBuffer::pop, &(this->_receive_buffer), num_bytes));
  }

  void close()
  {
    _serial_port.close();
  }

private:

  static size_t constexpr RECEIVE_BUFFER_SIZE = 32;

  uint8_t _asio_receive_buffer[RECEIVE_BUFFER_SIZE];
  ReceiveBuffer _receive_buffer;
  boost::asio::io_service _io_service;
  boost::asio::serial_port _serial_port;
  std::thread _io_service_thread;

  void read()
  {
    _serial_port.async_read_some(boost::asio::buffer(_asio_receive_buffer, RECEIVE_BUFFER_SIZE),
                                 boost::bind(&AsyncSerial::readEnd, this, _1, _2));
  }

  void readEnd(boost::system::error_code const & error, size_t bytes_transferred)
  {
    if (!error)
    {
      std::vector<uint8_t> const received_data(_asio_receive_buffer, _asio_receive_buffer+ bytes_transferred);
      _receive_buffer.push(received_data);
      read();
    }
  }

};

#endif /* ASYNCSERIAL_ASYNCSERIAL_HPP_ */
