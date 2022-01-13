/**
 * This software is distributed under the terms of the LGPL v2.1.
 * Copyright (c) 2022 Alexander Entinger, LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libasyncserial/graphs/contributors.
 */

#ifndef ASYNCSERIAL_ASYNCSERIAL_H_
#define ASYNCSERIAL_ASYNCSERIAL_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <cstdint>

#include <mutex>
#include <future>
#include <vector>
#include <thread>
#include <condition_variable>

#include <boost/asio.hpp>

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define LIBASYNCSERIAL_BASE_MAJOR 0001
#define LIBASYNCSERIAL_BASE_MINOR 0000
#define LIBASYNCSERIAL_BASE_PATCH 0000

#define LIBASYNCSERIAL_BASE_CONCAT_VERSION_(a,b,c) a ## b ## c
#define LIBASYNCSERIAL_BASE_CONCAT_VERSION(a,b,c) LIBASYNCSERIAL_BASE_CONCAT_VERSION_(a,b,c)

#define LIBASYNCSERIAL_BASE_VERSION \
        LIBASYNCSERIAL_BASE_CONCAT_VERSION(LIBASYNCSERIAL_BASE_MAJOR, \
                                                LIBASYNCSERIAL_BASE_MINOR, \
                                                LIBASYNCSERIAL_BASE_PATCH)

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AsyncSerial
{
public:

  static size_t constexpr MAJOR = LIBASYNCSERIAL_BASE_MAJOR;
  static size_t constexpr MINOR = LIBASYNCSERIAL_BASE_MINOR;
  static size_t constexpr PATCH = LIBASYNCSERIAL_BASE_PATCH;


   AsyncSerial();
  ~AsyncSerial();


  void open(std::string const & dev_node, size_t const baud_rate);
  void close();

  void transmit(std::vector<uint8_t> const & data);
  std::future<std::vector<uint8_t>> receive(size_t const num_bytes);


private:

  class ReceiveBuffer
  {
  public:
    void push(std::vector<uint8_t> const & received_data);
    std::vector<uint8_t> pop(size_t const num_bytes);
  private:
    std::mutex _mutex;
    std::condition_variable _condition;
    std::vector<uint8_t> _rx_buffer;
  };

  static size_t constexpr RECEIVE_BUFFER_SIZE = 32;

  uint8_t _asio_receive_buffer[RECEIVE_BUFFER_SIZE];
  ReceiveBuffer _receive_buffer;
  boost::asio::io_service _io_service;
  boost::asio::serial_port _serial_port;
  std::thread _io_service_thread;

  void read();
  void readEnd(boost::system::error_code const & error, size_t bytes_transferred);

};

#endif /* ASYNCSERIAL_ASYNCSERIAL_H_ */
