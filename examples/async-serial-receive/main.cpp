/**
 * This software is distributed under the terms of the LGPL v2.1.
 * Copyright (c) 2022 Alexander Entinger, LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libasyncserial/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <vector>
#include <string>
#include <future>
#include <iomanip>
#include <iostream>
#include <algorithm>

#include <asyncserial/AsyncSerial.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  AsyncSerial serial;

  serial.open("/dev/ttyUSB0", 115200);

  std::future<std::vector<uint8_t>> future = serial.receive(10);
  std::vector<uint8_t> const received_data = future.get();
  std::for_each(std::begin(received_data),
                std::end  (received_data),
                [](uint8_t const ch) { std::cout << std::setw(2) << std::setfill('0') << static_cast<size_t>(ch) << " "; });
  std::cout << std::endl;

  serial.close();

  return EXIT_SUCCESS;
}
